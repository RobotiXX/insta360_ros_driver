# insta360_ros_driver

A ROS driver for the Insta360 cameras. This driver is tested on Ubuntu 22.04 with ROS2 Humble. The driver has also been verified on the Insta360 X2 and X3 cameras. The following resolutions are available, all at 30 FPS.
- 3840 x 1920
- 2560 x 1280
- 2304 x 1152
- 1920 x 960

You can change [this line](https://github.com/ai4ce/insta360_ros_driver/blob/79588d9e0e9d029c3371d4095ea718daaf1e06fb/src/main.cpp#L126) to edit the resolution.

## Installation
To use this driver, you need to first have Insta360 SDK. Please apply for the SDK from the [Insta360 website](https://www.insta360.com/sdk/home). 

For additional instructions, see this [post](https://github.com/ai4ce/insta360_ros_driver/issues/10#issuecomment-3371481987).

**Note: Please make you use the latest SDK. This package works with the SDK posted after April 23, 2025**

```
cd ~/ros2_ws/src
git clone -b humble https://github.com/ai4ce/insta360_ros_driver
cd ..
```
Then, the Insta360 libraries need to be installed as follows:
- add the <code>camera</code> and <code>stream</code> header files inside the <code>include</code> directory
- add the <code>libCameraSDK.so</code> library under the <code>lib</code> directory.

Afterwards, install the other required dependencies and build
```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Before continuing, **make sure the camera is set to dual-lens mode**

Additionally, **ensure the camera's USB mode is set to Android**:
1. On the camera, swipe down the screen to the main menu
2. Go to Settings -> General
3. Set USB Mode to **Android** (not Webcam or other modes)
4. This is required for the ROS driver to properly detect and communicate with the camera (see [Issue #4](https://github.com/ai4ce/insta360_ros_driver/issues/4))

The Insta360 requires sudo privilege to be accessed via USB. To compensate for this, a udev configuration can be automatically created that will only request for sudo once. The camera can thus be setup initially via:
```
cd ~/ros2_ws/src/insta360_ros_driver
./setup.sh
```
This creates a symlink  based on the vendor ID of Insta360 cameras. The symlink, in this case <code>/dev/insta</code> is used to grant permissions to the usb port used by the camera.

![setup](docs/setup.png)

**Sometimes, this does not work (e.g. you see "device /dev/insta not found" or something similar). You can try entering the commands manually, since that sometimes sees success, especially for the first time.**
```
echo SUBSYSTEM=='"usb"', ATTR{manufacturer}=='"Arashi Vision"', SYMLINK+='"insta"', MODE='"0777"' | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/insta
```

## Usage
The camera provides images natively in H264 compressed image format. This package now includes multiple decode backends to support desktop NVIDIA GPUs and Jetson Orin deployment.

### Camera Bringup
The camera can be brought up with the following launch file
```
ros2 launch insta360_ros_driver bringup.launch.xml
```
![bringup](docs/bringup_rqt.png)

A dual fisheye image will be published.

![dual_fisheye](docs/dual_fisheye.png)

#### Published Topics
- /dual_fisheye/image
- /dual_fisheye/image/compressed
- /equirectangular/image
- /imu/data
- /imu/data_raw

The launch file has the following optional arguments:
- equirectangular (default="false")

This publishes equirectangular images. You can configure these parameters in `config/equirectangular.yaml`.
![equirectangular](docs/equirectangular.png)

- imu_filter (default="true")

This uses the [imu_filter_madgwick](https://wiki.ros.org/imu_filter_madgwick) package to approximate orientation from the IMU. Note that by default, we publish `/imu/data_raw` which only contains linear acceleration and angular velocity. The madgwick filter uses this information to publish orientation to `/imu/data`. You can configure the filter in `config/imu_filter.yaml`. 

![IMU](https://github.com/user-attachments/assets/02b50cad-8415-4dde-9014-9ab3a4d415b9)

## Decoder Backends and Deployment Profiles

This package supports several decode approaches. All of them share the same equirectangular mapping logic and calibration parameters, but they differ in decoder backend, performance, and dependency footprint.

### Quick Summary

| Approach | Executable | Best For | Build Default |
| --- | --- | --- | --- |
| Combined FFmpeg (legacy/original) | `insta360_ros_driver_decoded_equirectangular` | Existing deployments, simple baseline | Yes |
| Combined FFmpeg (V4L2-first) | `insta360_ros_driver_decoded_equirectangular_v4l2ffmpeg` | Jetson with FFmpeg fallback strategy | Yes |
| Combined GStreamer (`nvv4l2decoder`) | `insta360_ros_driver_decoded_equirectangular_gst` | Jetson production path with good maintainability | Optional (auto if GStreamer dev libs found) |
| Combined Jetson MMAPI (`NvVideoDecoder`) | `insta360_ros_driver_decoded_equirectangular_mmapi` | Lowest-level Jetson path (experimental) | Optional (`OFF` by default) |
| Split pipeline for ISAAC ROS | `h264_publisher.cpp` + `isaac_ros_h264_decoder` | NITROS/GXF ecosystem integration | Source scaffold only (see notes) |

### Backend Decision Tree

Use this as a fast picker during deployment:

```mermaid
flowchart TD
  A[Start: choose decode backend] --> B{Running on Jetson Orin?}
  B -->|No| C[Use Combined FFmpeg Original\ninsta360_ros_driver_decoded_equirectangular]
  B -->|Yes| D{Need easiest Jetson production path?}
  D -->|Yes| E[Use Combined GStreamer\ninsta360_ros_driver_decoded_equirectangular_gst]
  D -->|No| F{Need lowest-level control\nor advanced tuning?}
  F -->|Yes| G[Use Combined MMAPI (experimental)\ninsta360_ros_driver_decoded_equirectangular_mmapi]
  F -->|No| H{Need minimal dependency footprint?}
  H -->|Yes| I[Use V4L2-first FFmpeg\ninsta360_ros_driver_decoded_equirectangular_v4l2ffmpeg]
  H -->|No| E
  A --> J{Need ISAAC ROS/NITROS ecosystem?}
  J -->|Yes| K[Use split pipeline\nh264_publisher + isaac_ros_h264_decoder]
  J -->|No| B
```

Practical recommendation for Jetson AGX Orin:
- Start with GStreamer (`nvv4l2decoder`) for best balance of performance and maintainability.
- Move to MMAPI only if you need lower-level controls not exposed by GStreamer.
- Keep V4L2-first FFmpeg as a robust fallback path.

### 1) Combined FFmpeg (Original)

- Launch file: `launch/decoded_equirectangular_cpp.launch.xml`
- Typical command:

```bash
ros2 launch insta360_ros_driver decoded_equirectangular_cpp.launch.xml
```

Notes:
- This path uses FFmpeg decode integrated in the same process as capture + equirect projection.
- Good fallback when Jetson-specific dependencies are not installed.

### 2) Combined FFmpeg (V4L2-first)

- Executable: `insta360_ros_driver_decoded_equirectangular_v4l2ffmpeg`
- Launch file: `launch/decoded_equirectangular_v4l2ffmpeg.launch.xml`
- Typical command:

```bash
ros2 launch insta360_ros_driver decoded_equirectangular_v4l2ffmpeg.launch.xml
```

Decoder selection order inside this variant:
1. `h264_nvv4l2dec`
2. `h264_cuvid`
3. FFmpeg software H.264 fallback

This is a practical Jetson-friendly FFmpeg path with minimal extra dependencies.

### 3) Combined GStreamer (`nvv4l2decoder`)

- Executable: `insta360_ros_driver_decoded_equirectangular_gst`
- Launch file: `launch/decoded_equirectangular_gst.launch.xml`

Pipeline architecture in this node:
- Camera SDK H.264 bytes -> `appsrc` -> `h264parse` -> `nvv4l2decoder` -> `nvvidconv`/`videoconvert` -> `appsink` -> equirectangular projection -> ROS publish

Recommended for Jetson when you want strong hardware decode performance without moving to the lowest-level MMAPI implementation.

Build requirements (Jetson):

```bash
sudo apt-get update
sudo apt-get install -y \
  nvidia-l4t-gstreamer \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev
```

Build:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_ros_driver
```

Run:

```bash
ros2 launch insta360_ros_driver decoded_equirectangular_gst.launch.xml
```

If GStreamer dev packages are missing, CMake skips this target and prints a warning.

### 4) Combined Jetson MMAPI (`NvVideoDecoder`) (Experimental)

- Executable: `insta360_ros_driver_decoded_equirectangular_mmapi`
- Launch file: `launch/decoded_equirectangular_mmapi.launch.xml`
- Source: `src/main_decoded_equirectangular_mmapi.cpp`

This variant uses Jetson Multimedia API directly (`NvVideoDecoder`) with explicit output/capture plane management.

Why use it:
- Most direct Jetson decode integration.
- Fine-grained control over decoder behavior.

Trade-off:
- More complex than GStreamer/FFmpeg paths.
- Marked experimental and should be validated on target hardware.

Enable build (disabled by default):

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_ros_driver \
  --cmake-args -DBUILD_JETSON_MMAPI_COMBINED_NODE=ON
```

Run:

```bash
ros2 launch insta360_ros_driver decoded_equirectangular_mmapi.launch.xml
```

If Jetson Multimedia API headers/libs are not found (for example on non-Jetson hosts), this target is skipped.

### 5) ISAAC ROS Split Pipeline (Optional Ecosystem Path)

Files:
- `src/h264_publisher.cpp`
- `launch/h264_publisher_isaac_decoder.launch.xml`

This path is for using `isaac_ros_h264_decoder` as a separate node.

Important:
- `src/h264_publisher.cpp` is currently provided as source scaffold and is not added to `CMakeLists.txt` by default.
- This was intentional to keep dependencies/build graph stable during backend exploration.

When you are ready, add the publisher target in `CMakeLists.txt`, install `isaac_ros_compression`, and enable the commented ISAAC node block in the launch file.

## Build Profiles

### Profile A: Generic build (desktop or baseline)

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_ros_driver
```

Builds:
- Core nodes
- Original combined node
- V4L2-first FFmpeg combined node
- GStreamer combined node only if GStreamer dev packages are available

### Profile B: Jetson with MMAPI enabled

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_ros_driver \
  --cmake-args -DBUILD_JETSON_MMAPI_COMBINED_NODE=ON
```

Builds Profile A + MMAPI combined node if Jetson MMAPI headers/libs are found.

### Profile C: Force-disable GStreamer combined node

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_ros_driver \
  --cmake-args -DBUILD_GSTREAMER_COMBINED_NODE=OFF
```

Useful for quick iteration when only FFmpeg/MMAPI variants are being tested.

## Runtime Validation Checklist

After launching any backend variant, run:

```bash
ros2 topic hz /equirectangular/image
ros2 topic hz /imu/data_raw
```

Then verify image quality in RViz/rqt:
- No black seam artifacts across front/back split.
- Stable cadence (or expected throughput behavior) for your selected backend.
- IMU topic remains active while decode load increases.

For fixed-rate output experiments, set `target_fps` in your calibration YAML for nodes that support timer-latched publishing.

## Equirectangular Calibration
You can adjust the extrinsic parameters used to improve the equirectangular image. 
```
# Run the camera driver
ros2 run insta360_ros_driver insta360_ros_driver
# Activate image decoding
ros2 run insta360_ros_driver decoder
# Run the equirectangular node in calibration mode
ros2 run insta360_ros_driver equirectangular.py --calibrate
```
This will open an app to adjust the extrinsics. You can press 's' to get the parameters in YAML format. **Note that you need to press 'a' to update the image preview after changing the intrinsics with the GUI**
![Equirectangular Calibration](docs/calibration.png)

Pressing 's' will return the parameters via the terminal. You can copy paste this onto the configuration file as needed. By default, the launch file reads this from `config/equirectangular.yaml`

```
==================================================
CALIBRATION PARAMETERS (YAML FORMAT)
==================================================
equirectangular_node:
  ros__parameters:
    cx_offset: 0.0
    cy_offset: 0.0
    crop_size: 960
    translation: [0.0, 0.0, -0.105]
    rotation_deg: [-0.5, 0.0, 1.1]
    gpu: True
    out_width: 1920
    out_height: 960
==================================================
```

Note that decode.py will most likely drop frames depending on your system. If you do not care about live processing, you can simply record the `/dual_fisheye/image/compressed` topic and decompress it later after recording.
```
ros2 bag record /dual_fisheye/image /imu/data_raw
```

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=ai4ce/insta360_ros_driver&type=Date)](https://star-history.com/#ai4ce/insta360_ros_driver&Date)
