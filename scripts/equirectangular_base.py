#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
import torch
import torch.nn.functional as F
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image


class EquirectangularBaseNode(Node):
    def __init__(self):
        super().__init__("equirectangular_node")

        self.params_changed = True
        self.maps_initialized = False

        self.img_height: int | None = None
        self.img_width: int | None = None

        self.front_mask: torch.Tensor | None = None
        self.back_mask: torch.Tensor | None = None
        self.front_mask_np: np.ndarray | None = None
        self.front_edge: np.ndarray | None = None
        self.front_distance: np.ndarray | None = None
        self.blend_kernel: np.ndarray = np.ones((5, 5), np.uint8)

        self.front_grid: torch.Tensor | None = None
        self.back_grid: torch.Tensor | None = None
        self.front_mask_gpu: torch.Tensor | None = None
        self.back_mask_gpu: torch.Tensor | None = None

        self.declare_parameters(
            namespace="",
            parameters=[
                ("cx_offset", 0.0),
                ("cy_offset", 0.0),
                ("front_cx_offset", 0.0),
                ("front_cy_offset", 0.0),
                ("back_cx_offset", 0.0),
                ("back_cy_offset", 0.0),
                ("crop_size", 960),
                ("translation", [0.0, 0.0, -0.105]),
                ("rotation_deg", [-0.5, 0.0, 1.1]),
                ("gpu", True),
                ("out_width", 1920),
                ("out_height", 960),
                ("target_fps", 0.0),
            ],
        )

        self.bridge = CvBridge()
        self.load_parameters()

        self.use_cuda = torch.cuda.is_available() and self.gpu_enabled
        self.device = torch.device("cuda" if self.use_cuda else "cpu")
        self.get_logger().info(
            f"GPU acceleration: requested={self.gpu_enabled}, "
            f"available={torch.cuda.is_available()}, using={self.use_cuda}"
        )

        self.update_camera_parameters()
        self.add_on_set_parameters_callback(self.parameters_callback)

        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )

        self.dual_fisheye_sub = self.create_subscription(
            Image, "/dual_fisheye/image", self.image_callback, qos
        )
        self.equirect_pub = self.create_publisher(Image, "/equirectangular/image", qos)

        self.latest_dual_fisheye_msg: Image | None = None
        self.latest_msg_ready = False
        self.rate_timer = None

        if self.target_fps > 0.0:
            period = 1.0 / self.target_fps
            self.rate_timer = self.create_timer(period, self.rate_timer_callback)
            self.get_logger().info(
                f"Fixed-rate mode enabled for equirectangular output: target_fps={self.target_fps:.2f}"
            )

    def load_parameters(self):
        self.cx_offset = self.get_parameter("cx_offset").value
        self.cy_offset = self.get_parameter("cy_offset").value
        self.front_cx_offset = self.get_parameter("front_cx_offset").value
        self.front_cy_offset = self.get_parameter("front_cy_offset").value
        self.back_cx_offset = self.get_parameter("back_cx_offset").value
        self.back_cy_offset = self.get_parameter("back_cy_offset").value
        self.crop_size = self.get_parameter("crop_size").value
        self.out_width = self.get_parameter("out_width").value
        self.out_height = self.get_parameter("out_height").value
        self.gpu_enabled = self.get_parameter("gpu").value
        self.target_fps = float(self.get_parameter("target_fps").value)

        translation = self.get_parameter("translation").value
        self.tx, self.ty, self.tz = translation

        rotation_deg = self.get_parameter("rotation_deg").value
        self.roll = math.radians(rotation_deg[0])
        self.pitch = math.radians(rotation_deg[1])
        self.yaw = math.radians(rotation_deg[2])

        self.get_logger().info(
            "Loaded parameters: "
            f"crop_size={self.crop_size}, "
            f"shared_center=[{self.cx_offset:.2f},{self.cy_offset:.2f}], "
            f"front_center=[{self.front_cx_offset:.2f},{self.front_cy_offset:.2f}], "
            f"back_center=[{self.back_cx_offset:.2f},{self.back_cy_offset:.2f}], "
            f"translation=[{self.tx:.4f},{self.ty:.4f},{self.tz:.4f}], "
            f"rotation_deg=[{rotation_deg[0]:.3f},{rotation_deg[1]:.3f},{rotation_deg[2]:.3f}], "
            f"out={self.out_width}x{self.out_height}, gpu={self.gpu_enabled}"
        )

    def parameters_callback(self, params):
        tracked = {
            "cx_offset",
            "cy_offset",
            "front_cx_offset",
            "front_cy_offset",
            "back_cx_offset",
            "back_cy_offset",
            "crop_size",
            "translation",
            "rotation_deg",
            "out_width",
            "out_height",
            "gpu",
            "target_fps",
        }
        if any(param.name in tracked for param in params):
            old_target_fps = self.target_fps
            self.load_parameters()
            self.use_cuda = torch.cuda.is_available() and self.gpu_enabled
            self.device = torch.device("cuda" if self.use_cuda else "cpu")
            self.update_camera_parameters()
            self.maps_initialized = False

            if self.target_fps != old_target_fps:
                if self.rate_timer is not None:
                    self.rate_timer.cancel()
                    self.rate_timer = None

                if self.target_fps > 0.0:
                    period = 1.0 / self.target_fps
                    self.rate_timer = self.create_timer(period, self.rate_timer_callback)
                    self.get_logger().info(
                        f"Updated fixed-rate mode: target_fps={self.target_fps:.2f}"
                    )
                else:
                    self.get_logger().info("Fixed-rate mode disabled; using input-driven processing")

        return SetParametersResult(successful=True)

    def rate_timer_callback(self):
        if not self.latest_msg_ready or self.latest_dual_fisheye_msg is None:
            return

        dual_fisheye_msg = self.latest_dual_fisheye_msg
        self.latest_msg_ready = False
        self.process_frame(dual_fisheye_msg)

    def update_camera_parameters(self):
        rx = torch.tensor(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(self.roll), -math.sin(self.roll)],
                [0.0, math.sin(self.roll), math.cos(self.roll)],
            ],
            device=self.device,
        )
        ry = torch.tensor(
            [
                [math.cos(self.pitch), 0.0, math.sin(self.pitch)],
                [0.0, 1.0, 0.0],
                [-math.sin(self.pitch), 0.0, math.cos(self.pitch)],
            ],
            device=self.device,
        )
        rz = torch.tensor(
            [
                [math.cos(self.yaw), -math.sin(self.yaw), 0.0],
                [math.sin(self.yaw), math.cos(self.yaw), 0.0],
                [0.0, 0.0, 1.0],
            ],
            device=self.device,
        )

        self.back_to_front_rotation = torch.matmul(torch.matmul(rz, ry), rx)
        self.back_to_front_translation = torch.tensor(
            [self.tx, self.ty, self.tz], device=self.device
        )

    def init_mapping(self, img_height: int, img_width: int):
        self.img_height = img_height
        self.img_width = img_width

        base_cx = img_width / 2 + self.cx_offset
        base_cy = img_height / 2 + self.cy_offset
        front_cx = base_cx + self.front_cx_offset
        front_cy = base_cy + self.front_cy_offset
        back_cx = base_cx + self.back_cx_offset
        back_cy = base_cy + self.back_cy_offset

        y, x = torch.meshgrid(
            torch.arange(self.out_height, dtype=torch.float32, device=self.device),
            torch.arange(self.out_width, dtype=torch.float32, device=self.device),
            indexing="ij",
        )

        longitude = (x / self.out_width) * 2 * math.pi - math.pi
        latitude = (y / self.out_height) * math.pi - math.pi / 2

        x_dir = torch.cos(latitude) * torch.sin(longitude)
        y_dir = torch.sin(latitude)
        z_dir = torch.cos(latitude) * torch.cos(longitude)

        self.front_mask = z_dir >= 0
        self.back_mask = z_dir < 0

        r_front = torch.sqrt(x_dir[self.front_mask] ** 2 + y_dir[self.front_mask] ** 2).clamp_(
            min=1e-6
        )
        theta_front = torch.atan2(r_front, torch.abs(z_dir[self.front_mask]))
        r_fisheye_front = 2 * theta_front / math.pi * (img_width / 2)

        self.front_map_x = torch.zeros(
            (self.out_height, self.out_width), dtype=torch.float32, device=self.device
        )
        self.front_map_y = torch.zeros(
            (self.out_height, self.out_width), dtype=torch.float32, device=self.device
        )
        self.front_map_x[self.front_mask] = front_cx + x_dir[self.front_mask] / r_front * r_fisheye_front
        self.front_map_y[self.front_mask] = front_cy + y_dir[self.front_mask] / r_front * r_fisheye_front

        back_points = torch.stack(
            [x_dir[self.back_mask], y_dir[self.back_mask], z_dir[self.back_mask]], dim=1
        )
        rotation = self.back_to_front_rotation.to(dtype=torch.float32)
        translation = self.back_to_front_translation.to(dtype=torch.float32)
        transformed_points = torch.matmul(back_points, rotation.transpose(0, 1)) + translation

        x_back = -transformed_points[:, 0]
        y_back = transformed_points[:, 1]
        z_back = transformed_points[:, 2]

        r_back = torch.sqrt(x_back**2 + y_back**2).clamp_(min=1e-6)
        theta_back = torch.atan2(r_back, torch.abs(z_back))
        r_fisheye_back = 2 * theta_back / math.pi * (img_width / 2)

        self.back_map_x = torch.zeros(
            (self.out_height, self.out_width), dtype=torch.float32, device=self.device
        )
        self.back_map_y = torch.zeros(
            (self.out_height, self.out_width), dtype=torch.float32, device=self.device
        )
        self.back_map_x[self.back_mask] = back_cx + x_back / r_back * r_fisheye_back
        self.back_map_y[self.back_mask] = back_cy + y_back / r_back * r_fisheye_back

        self.front_map_x_np = self.front_map_x.cpu().numpy()
        self.front_map_y_np = self.front_map_y.cpu().numpy()
        self.back_map_x_np = self.back_map_x.cpu().numpy()
        self.back_map_y_np = self.back_map_y.cpu().numpy()

        self.front_mask_np = self.front_mask.cpu().numpy()
        front_mask_uint8 = self.front_mask_np.astype(np.uint8)
        self.front_edge = cv2.dilate(front_mask_uint8, self.blend_kernel, iterations=2) - cv2.erode(
            front_mask_uint8, self.blend_kernel, iterations=2
        )
        self.front_distance = cv2.distanceTransform(front_mask_uint8, cv2.DIST_L2, 3)
        self.front_distance = np.clip(self.front_distance * 0.3, 0, 1)

        if self.use_cuda:
            try:
                front_map_x_norm = 2.0 * (self.front_map_x / img_width) - 1.0
                front_map_y_norm = 2.0 * (self.front_map_y / img_height) - 1.0
                self.front_grid = torch.stack([front_map_x_norm, front_map_y_norm], dim=-1).unsqueeze(0)

                back_map_x_norm = 2.0 * (self.back_map_x / img_width) - 1.0
                back_map_y_norm = 2.0 * (self.back_map_y / img_height) - 1.0
                self.back_grid = torch.stack([back_map_x_norm, back_map_y_norm], dim=-1).unsqueeze(0)

                self.front_mask_gpu = self.front_mask.float().unsqueeze(0).unsqueeze(0)
                self.back_mask_gpu = self.back_mask.float().unsqueeze(0).unsqueeze(0)
            except Exception as exc:
                self.use_cuda = False
                self.get_logger().warn(f"GPU map init failed, using CPU path: {exc}")

        self.maps_initialized = True

    def create_equirectangular_cpu(self, front_img: np.ndarray, back_img: np.ndarray) -> np.ndarray:
        front_result = cv2.remap(
            front_img,
            self.front_map_x_np,
            self.front_map_y_np,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )
        back_result = cv2.remap(
            back_img,
            self.back_map_x_np,
            self.back_map_y_np,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )

        equirect = np.zeros((self.out_height, self.out_width, 3), dtype=np.uint8)

        non_edge_front = self.front_mask_np & (self.front_edge == 0)
        non_edge_back = ~self.front_mask_np & (self.front_edge == 0)
        equirect[non_edge_front] = front_result[non_edge_front]
        equirect[non_edge_back] = back_result[non_edge_back]

        blend_region = self.front_edge == 1
        if np.any(blend_region):
            alpha = self.front_distance[blend_region][..., np.newaxis]
            equirect[blend_region] = (
                alpha * front_result[blend_region].astype(np.float32)
                + (1 - alpha) * back_result[blend_region].astype(np.float32)
            ).astype(np.uint8)

        return equirect

    def create_equirectangular_gpu(self, front_img: np.ndarray, back_img: np.ndarray) -> np.ndarray:
        front_tensor = (
            torch.from_numpy(front_img)
            .to(self.device, non_blocking=True)
            .float()
            .permute(2, 0, 1)
            .unsqueeze(0)
        )
        back_tensor = (
            torch.from_numpy(back_img)
            .to(self.device, non_blocking=True)
            .float()
            .permute(2, 0, 1)
            .unsqueeze(0)
        )

        front_remapped = F.grid_sample(
            front_tensor,
            self.front_grid,
            mode="bilinear",
            padding_mode="border",
            align_corners=True,
        )
        back_remapped = F.grid_sample(
            back_tensor,
            self.back_grid,
            mode="bilinear",
            padding_mode="border",
            align_corners=True,
        )

        if front_remapped.shape[2:] != (self.out_height, self.out_width):
            front_remapped = F.interpolate(
                front_remapped,
                size=(self.out_height, self.out_width),
                mode="bilinear",
                align_corners=True,
            )
        if back_remapped.shape[2:] != (self.out_height, self.out_width):
            back_remapped = F.interpolate(
                back_remapped,
                size=(self.out_height, self.out_width),
                mode="bilinear",
                align_corners=True,
            )

        output = front_remapped * self.front_mask_gpu + back_remapped * self.back_mask_gpu
        output_np = output.squeeze(0).permute(1, 2, 0).cpu().numpy()
        return np.clip(output_np, 0, 255).astype(np.uint8)

    def image_callback(self, dual_fisheye_msg: Image):
        if self.target_fps > 0.0:
            # Keep only the newest frame. Timer callback processes at fixed output rate.
            self.latest_dual_fisheye_msg = dual_fisheye_msg
            self.latest_msg_ready = True
            return

        self.process_frame(dual_fisheye_msg)

    def process_frame(self, dual_fisheye_msg: Image):
        try:
            dual_fisheye_img = self.bridge.imgmsg_to_cv2(dual_fisheye_msg, "rgb8")
            _, img_width_full, _ = dual_fisheye_img.shape
            midpoint = img_width_full // 2

            front_img_full = dual_fisheye_img[:, midpoint:]
            back_img_full = dual_fisheye_img[:, :midpoint]

            current_crop_size = self.crop_size
            orig_h, orig_w = front_img_full.shape[:2]
            if orig_h != current_crop_size or orig_w != current_crop_size:
                y_start = (orig_h - current_crop_size) // 2
                x_start = (orig_w - current_crop_size) // 2
                if (
                    y_start >= 0
                    and x_start >= 0
                    and y_start + current_crop_size <= orig_h
                    and x_start + current_crop_size <= orig_w
                ):
                    front_img = front_img_full[
                        y_start : y_start + current_crop_size,
                        x_start : x_start + current_crop_size,
                    ]
                    back_img = back_img_full[
                        y_start : y_start + current_crop_size,
                        x_start : x_start + current_crop_size,
                    ]
                else:
                    front_img = front_img_full
                    back_img = back_img_full
            else:
                front_img = front_img_full
                back_img = back_img_full

            if (
                not self.maps_initialized
                or self.params_changed
                or (self.img_height is not None and front_img.shape[0] != self.img_height)
                or (self.img_width is not None and front_img.shape[1] != self.img_width)
            ):
                self.init_mapping(front_img.shape[0], front_img.shape[1])
                self.params_changed = False

            if self.use_cuda and self.front_grid is not None and self.back_grid is not None:
                try:
                    equirect_img = self.create_equirectangular_gpu(front_img, back_img)
                except Exception as exc:
                    self.get_logger().warn(f"GPU processing error, switching to CPU: {exc}")
                    self.use_cuda = False
                    equirect_img = self.create_equirectangular_cpu(front_img, back_img)
            else:
                equirect_img = self.create_equirectangular_cpu(front_img, back_img)

            equirect_msg = self.bridge.cv2_to_imgmsg(equirect_img, encoding="rgb8")
            equirect_msg.header = dual_fisheye_msg.header
            self.equirect_pub.publish(equirect_msg)

        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge error: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Processing error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = EquirectangularBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
