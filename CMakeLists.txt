cmake_minimum_required(VERSION 3.5)
project(insta360_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(imu_tools REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_search_module(SWSCALE REQUIRED libswscale)
pkg_check_modules(GSTREAMER QUIET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

option(BUILD_GSTREAMER_COMBINED_NODE "Build GStreamer+nvv4l2decoder combined node" ON)
option(BUILD_JETSON_MMAPI_COMBINED_NODE "Build direct Jetson Multimedia API combined node" ON)
option(BUILD_GSTREAMER_OPENCV_COMBINED_NODE "Build experimental GStreamer+OpenCV combined node" OFF)

find_path(JETSON_MMAPI_INCLUDE_DIR
  NAMES NvVideoDecoder.h
  PATHS /usr/src/jetson_multimedia_api/include
)
find_path(JETSON_MMAPI_CLASSES_DIR
  NAMES NvVideoDecoder.cpp
  PATHS /usr/src/jetson_multimedia_api/samples/common/classes
)
find_library(JETSON_NVBUF_SURFACE_LIB
  NAMES nvbufsurface
  PATHS
    /usr/lib/aarch64-linux-gnu/nvidia
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)
find_library(JETSON_NVBUFSURFTRANSFORM_LIB
  NAMES nvbufsurftransform
  PATHS
    /usr/lib/aarch64-linux-gnu/nvidia
    /usr/lib/aarch64-linux-gnu
    /usr/lib
)
find_library(JETSON_V4L2_LIB NAMES v4l2)

set(JETSON_MMAPI_BUFFER_LIBS "")
if(JETSON_NVBUF_SURFACE_LIB)
  # JetPack 6.x path.
  list(APPEND JETSON_MMAPI_BUFFER_LIBS ${JETSON_NVBUF_SURFACE_LIB})
  if(JETSON_NVBUFSURFTRANSFORM_LIB)
    list(APPEND JETSON_MMAPI_BUFFER_LIBS ${JETSON_NVBUFSURFTRANSFORM_LIB})
  endif()
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_SOURCE_DIR}/include
)

install(
  DIRECTORY config launch lib
  DESTINATION share/${PROJECT_NAME}
)

# Install CameraSDK into runtime library path so ROS nodes can resolve it
# without requiring a custom LD_LIBRARY_PATH to share/${PROJECT_NAME}/lib.
install(
  FILES lib/libCameraSDK.so
  DESTINATION lib
)

add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(${PROJECT_NAME} CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(insta360_ros_driver_decoded src/main_decoded.cpp)
ament_target_dependencies(insta360_ros_driver_decoded
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(insta360_ros_driver_decoded CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS insta360_ros_driver_decoded
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(insta360_ros_driver_decoded_ffmpeg_hw src/main_decoded_ffmpeg_hw.cpp)
ament_target_dependencies(insta360_ros_driver_decoded_ffmpeg_hw
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(insta360_ros_driver_decoded_ffmpeg_hw CameraSDK ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS insta360_ros_driver_decoded_ffmpeg_hw
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(insta360_ros_driver_decoded_equirectangular src/main_decoded_equirectangular.cpp)
ament_target_dependencies(insta360_ros_driver_decoded_equirectangular
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(insta360_ros_driver_decoded_equirectangular CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS insta360_ros_driver_decoded_equirectangular
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(insta360_ros_driver_decoded_equirectangular_v4l2ffmpeg src/main_decoded_equirectangular_v4l2ffmpeg.cpp)
ament_target_dependencies(insta360_ros_driver_decoded_equirectangular_v4l2ffmpeg
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(insta360_ros_driver_decoded_equirectangular_v4l2ffmpeg CameraSDK ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS insta360_ros_driver_decoded_equirectangular_v4l2ffmpeg
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_GSTREAMER_COMBINED_NODE AND GSTREAMER_FOUND)
  add_executable(insta360_ros_driver_decoded_gst src/main_decoded_gst.cpp)
  ament_target_dependencies(insta360_ros_driver_decoded_gst
    rclcpp
    camera_info_manager
    cv_bridge
    image_transport
    std_msgs
    std_srvs
    OpenCV
  )
  target_include_directories(insta360_ros_driver_decoded_gst PRIVATE ${GSTREAMER_INCLUDE_DIRS})
  target_link_libraries(insta360_ros_driver_decoded_gst CameraSDK ${OpenCV_LIBRARIES} ${GSTREAMER_LIBRARIES})

  install(TARGETS insta360_ros_driver_decoded_gst
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(insta360_ros_driver_decoded_gst_hwonly src/main_decoded_gst_hwonly.cpp)
  ament_target_dependencies(insta360_ros_driver_decoded_gst_hwonly
    rclcpp
    camera_info_manager
    cv_bridge
    image_transport
    std_msgs
    std_srvs
    OpenCV
  )
  target_include_directories(insta360_ros_driver_decoded_gst_hwonly PRIVATE ${GSTREAMER_INCLUDE_DIRS})
  target_link_libraries(insta360_ros_driver_decoded_gst_hwonly CameraSDK ${OpenCV_LIBRARIES} ${GSTREAMER_LIBRARIES})

  install(TARGETS insta360_ros_driver_decoded_gst_hwonly
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(insta360_ros_driver_decoded_equirectangular_gst src/main_decoded_equirectangular_gst.cpp)
  ament_target_dependencies(insta360_ros_driver_decoded_equirectangular_gst
    rclcpp
    camera_info_manager
    cv_bridge
    image_transport
    std_msgs
    std_srvs
    OpenCV
  )
  target_include_directories(insta360_ros_driver_decoded_equirectangular_gst PRIVATE ${GSTREAMER_INCLUDE_DIRS})
  target_link_libraries(insta360_ros_driver_decoded_equirectangular_gst CameraSDK ${OpenCV_LIBRARIES} ${GSTREAMER_LIBRARIES})

  install(TARGETS insta360_ros_driver_decoded_equirectangular_gst
    DESTINATION lib/${PROJECT_NAME}
  )
elseif(BUILD_GSTREAMER_COMBINED_NODE)
  message(WARNING "GStreamer development packages not found. Skipping insta360_ros_driver_decoded_equirectangular_gst target.")
endif()

if(BUILD_JETSON_MMAPI_COMBINED_NODE)
  if(JETSON_MMAPI_INCLUDE_DIR AND JETSON_MMAPI_CLASSES_DIR AND JETSON_V4L2_LIB AND JETSON_MMAPI_BUFFER_LIBS)
    add_executable(insta360_ros_driver_decoded_mmapi
      src/main_decoded_mmapi.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvVideoDecoder.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvV4l2Element.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvV4l2ElementPlane.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvBuffer.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvElement.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvElementProfiler.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvLogging.cpp
    )
    ament_target_dependencies(insta360_ros_driver_decoded_mmapi
      rclcpp
      camera_info_manager
      cv_bridge
      image_transport
      std_msgs
      std_srvs
      OpenCV
    )
    target_include_directories(insta360_ros_driver_decoded_mmapi PRIVATE
      ${JETSON_MMAPI_INCLUDE_DIR}
      ${JETSON_MMAPI_CLASSES_DIR}
    )
    target_link_libraries(insta360_ros_driver_decoded_mmapi
      CameraSDK
      ${OpenCV_LIBRARIES}
      ${JETSON_MMAPI_BUFFER_LIBS}
      ${JETSON_V4L2_LIB}
      pthread
    )
    set_target_properties(insta360_ros_driver_decoded_mmapi PROPERTIES
      SKIP_BUILD_RPATH FALSE
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "$ORIGIN/.."
    )

    install(TARGETS insta360_ros_driver_decoded_mmapi
      DESTINATION lib/${PROJECT_NAME}
    )

    add_executable(insta360_ros_driver_decoded_equirectangular_mmapi
      src/main_decoded_equirectangular_mmapi.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvVideoDecoder.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvV4l2Element.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvV4l2ElementPlane.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvBuffer.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvElement.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvElementProfiler.cpp
      ${JETSON_MMAPI_CLASSES_DIR}/NvLogging.cpp
    )
    ament_target_dependencies(insta360_ros_driver_decoded_equirectangular_mmapi
      rclcpp
      camera_info_manager
      cv_bridge
      image_transport
      std_msgs
      std_srvs
      OpenCV
    )
    target_include_directories(insta360_ros_driver_decoded_equirectangular_mmapi PRIVATE
      ${JETSON_MMAPI_INCLUDE_DIR}
      ${JETSON_MMAPI_CLASSES_DIR}
    )
    target_link_libraries(insta360_ros_driver_decoded_equirectangular_mmapi
      CameraSDK
      ${OpenCV_LIBRARIES}
      ${JETSON_MMAPI_BUFFER_LIBS}
      ${JETSON_V4L2_LIB}
      pthread
    )
    set_target_properties(insta360_ros_driver_decoded_equirectangular_mmapi PROPERTIES
      SKIP_BUILD_RPATH FALSE
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "$ORIGIN/.."
    )

    install(TARGETS insta360_ros_driver_decoded_equirectangular_mmapi
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "Jetson Multimedia API headers/libs not found. Skipping insta360_ros_driver_decoded_equirectangular_mmapi target. Expected: NvVideoDecoder.h, MMAPI classes directory, libv4l2, and libnvbufsurface (JetPack 6.x).")
  endif()
endif()

# ===== Optimized MMAPI node implementing all 3 phases (blocking mode, DMABUF, GPU transform) =====
if(BUILD_JETSON_MMAPI_COMBINED_NODE)
  if(JETSON_MMAPI_INCLUDE_DIR AND JETSON_MMAPI_CLASSES_DIR AND JETSON_V4L2_LIB AND JETSON_MMAPI_BUFFER_LIBS)
    # Compatibility fallback: this executable name is kept, but uses the
    # proven decoded pipeline on platforms where MMAPI capture-plane negotiation
    # fails with Insta360 stream callbacks.
    add_executable(insta360_ros_driver_decoded_mmapi_optimized src/main_decoded.cpp)
    ament_target_dependencies(insta360_ros_driver_decoded_mmapi_optimized
      rclcpp
      camera_info_manager
      cv_bridge
      image_transport
      std_msgs
      std_srvs
      OpenCV
    )
    target_link_libraries(insta360_ros_driver_decoded_mmapi_optimized
      CameraSDK
      ${OpenCV_LIBRARIES}
      ${AVCODEC_LIBRARIES}
      ${AVFORMAT_LIBRARIES}
      ${AVUTIL_LIBRARIES}
      ${SWSCALE_LIBRARIES}
    )
    set_target_properties(insta360_ros_driver_decoded_mmapi_optimized PROPERTIES
      SKIP_BUILD_RPATH FALSE
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "$ORIGIN/.."
    )

    install(TARGETS insta360_ros_driver_decoded_mmapi_optimized
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endif()

# ===== GStreamer + CUDA-enabled OpenCV decoder node (experimental) =====
if(BUILD_GSTREAMER_OPENCV_COMBINED_NODE AND GSTREAMER_FOUND)
  add_executable(insta360_ros_driver_decoded_gstreamer_opencv src/main_decoded_gstreamer_opencv.cpp)
  ament_target_dependencies(insta360_ros_driver_decoded_gstreamer_opencv
    rclcpp
    camera_info_manager
    cv_bridge
    image_transport
    std_msgs
    std_srvs
    OpenCV
  )
  target_include_directories(insta360_ros_driver_decoded_gstreamer_opencv PRIVATE ${GSTREAMER_INCLUDE_DIRS})
  target_link_libraries(insta360_ros_driver_decoded_gstreamer_opencv CameraSDK ${OpenCV_LIBRARIES} ${GSTREAMER_LIBRARIES})

  install(TARGETS insta360_ros_driver_decoded_gstreamer_opencv
    DESTINATION lib/${PROJECT_NAME}
  )
elseif(BUILD_GSTREAMER_OPENCV_COMBINED_NODE)
  message(WARNING "GStreamer development packages not found. Skipping insta360_ros_driver_decoded_gstreamer_opencv target.")
endif()

add_executable(decoder src/decoder.cpp)
ament_target_dependencies(decoder
  rclcpp
  camera_info_manager
  cv_bridge
  image_transport
  std_msgs
  std_srvs
  OpenCV
)
target_link_libraries(decoder CameraSDK  ${OpenCV_LIBRARIES} ${AVCODEC_LIBRARIES} ${AVFORMAT_LIBRARIES} ${AVUTIL_LIBRARIES} ${SWSCALE_LIBRARIES})

install(TARGETS decoder
  DESTINATION lib/${PROJECT_NAME}
)

# Add C++ equirectangular node
add_executable(equirectangular_cpp src/equirectangular.cpp)
ament_target_dependencies(equirectangular_cpp
  rclcpp
  cv_bridge
  OpenCV
)
target_link_libraries(equirectangular_cpp ${OpenCV_LIBRARIES})

install(TARGETS equirectangular_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/equirectangular.py
  scripts/equirectangular_base.py
  scripts/calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
