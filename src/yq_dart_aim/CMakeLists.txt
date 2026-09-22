cmake_minimum_required(VERSION 3.8)
project(yq_dart_aim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(dart_aim_node
  src/dart_aim_node.cpp
  src/image_processor.cpp
  src/target_detector.cpp
  src/model_detector.cpp
  src/coordinate_calculator.cpp
  src/serial_manager.cpp
)

# ==================== 推理后端（二选一）====================
#   ORT (默认) : ONNX Runtime 加速，跑切好的 .onnx，CPU 方案
#                需要 ONNX Runtime 的 C++ 头文件/库：-DORT_ROOT=/path/to/onnxruntime
#   BPU        : RDK X5 BPU，跑 .bin（hobot_dnn），板子上用这个
#   另有 DNN 可选：OpenCV DNN，仅当 OpenCV >= 5.x 时可用（4.5.4 不支持本模型的 ArgMax）
#
# 用法：
#   colcon build --cmake-args -DDART_INFER_BACKEND=BPU              # RDK X5
#   colcon build --cmake-args -DDART_INFER_BACKEND=ORT -DORT_ROOT=~/onnxruntime
set(DART_INFER_BACKEND "ORT" CACHE STRING "推理后端: ORT | BPU | DNN")
set_property(CACHE DART_INFER_BACKEND PROPERTY STRINGS ORT BPU DNN)

if(DART_INFER_BACKEND STREQUAL "BPU")
  target_sources(dart_aim_node PRIVATE src/model_backend_bpu.cpp)
  target_compile_definitions(dart_aim_node PRIVATE WITH_BPU)
  # hobot_dnn 头文件默认在 /usr/include/hobot/dnn（系统默认搜索路径内）。
  # 若装在别处，取消注释并按实际路径修改：
  # target_include_directories(dart_aim_node PRIVATE /usr/include/hobot/dnn)
  target_link_libraries(dart_aim_node dnn)
  message(STATUS "yq_dart_aim: 推理后端 = BPU (hobot_dnn, RDK X5)")
elseif(DART_INFER_BACKEND STREQUAL "ORT")
  target_sources(dart_aim_node PRIVATE src/model_backend_ort.cpp)
  set(ORT_ROOT "" CACHE PATH "ONNX Runtime 根目录（含 include/ 与 lib/）")
  if(ORT_ROOT)
    # 用官方 release 包解压出来的目录
    target_include_directories(dart_aim_node PRIVATE ${ORT_ROOT}/include)
    target_link_directories(dart_aim_node PRIVATE ${ORT_ROOT}/lib)
    set(ORT_LIB onnxruntime)
  else()
    # 退回系统安装路径
    find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
      PATHS /usr/include /usr/local/include /usr/include/onnxruntime /usr/local/include/onnxruntime)
    find_library(ORT_LIB onnxruntime PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu)
    if(NOT ORT_LIB OR NOT ORT_INCLUDE_DIR)
      message(FATAL_ERROR
        "找不到 ONNX Runtime（头文件 onnxruntime_cxx_api.h / 库 libonnxruntime.so）。\n"
        "  方案一：下载官方 C++ release 包后指定路径：\n"
        "      colcon build --cmake-args -DDART_INFER_BACKEND=ORT -DORT_ROOT=/path/to/onnxruntime\n"
        "  方案二：改用 BPU 后端（RDK X5 上）：\n"
        "      colcon build --cmake-args -DDART_INFER_BACKEND=BPU")
    endif()
    target_include_directories(dart_aim_node PRIVATE ${ORT_INCLUDE_DIR})
  endif()
  target_link_libraries(dart_aim_node ${ORT_LIB})
  message(STATUS "yq_dart_aim: 推理后端 = ONNX Runtime")
elseif(DART_INFER_BACKEND STREQUAL "DNN")
  target_sources(dart_aim_node PRIVATE src/model_backend_dnn.cpp)
  message(STATUS "yq_dart_aim: 推理后端 = cv::dnn（需 OpenCV >= 5.x）")
else()
  message(FATAL_ERROR "DART_INFER_BACKEND 只能是 ORT / BPU / DNN，当前=${DART_INFER_BACKEND}")
endif()

# include directories for the target
target_include_directories(dart_aim_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(dart_aim_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  OpenCV
)

# link OpenCV libs (use plain signature to match ament_target_dependencies usage)
target_link_libraries(dart_aim_node ${OpenCV_LIBS} opencv_dnn)

install(TARGETS
  dart_aim_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 web_tuner Python 包（安装到 share 目录，launch 文件可直接引用）
install(DIRECTORY web_tuner
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
