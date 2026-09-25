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
  src/model_backend.cpp
  src/coordinate_calculator.cpp
  src/serial_manager.cpp
)

# ==================== 推理后端（可多选，运行期用 infer_backend 参数切换）====================
#   CPU(ORT) : ONNX Runtime，跑切好后的 .onnx
#   BPU      : RDK X5 hobot_dnn，跑量化 .bin（板子部署用）
#   DNN      : OpenCV DNN，跑 .onnx（备选，比 ORT 慢）
#
# 依赖能找到就把后端编进程序（自动探测），也可以显式关闭：
#   colcon build --cmake-args -DDART_ENABLE_BPU=OFF -DDART_ENABLE_ORT=OFF
# 下面这条只决定 params.yaml 没给 infer_backend 时的默认值：
#   colcon build --cmake-args -DDART_INFER_BACKEND=BPU
set(DART_INFER_BACKEND "CPU" CACHE STRING "默认推理后端: CPU | BPU | DNN")
set_property(CACHE DART_INFER_BACKEND PROPERTY STRINGS CPU BPU DNN)
set(DART_ENABLE_BPU "AUTO" CACHE STRING "编译 BPU 后端: AUTO|ON|OFF")
set(DART_ENABLE_ORT "AUTO" CACHE STRING "编译 ORT 后端: AUTO|ON|OFF")
set(DART_ENABLE_DNN "AUTO" CACHE STRING "编译 DNN 后端: AUTO|ON|OFF")
set(ORT_ROOT "" CACHE PATH "ONNX Runtime 根目录（含 include/ 与 lib/），留空则从系统路径找")

# ── BPU: hobot_dnn ──
set(_bpu_ok OFF)
if(NOT DART_ENABLE_BPU STREQUAL "OFF")
  find_path(DART_BPU_INCLUDE_DIR hobot/dnn/hb_dnn.h
    PATHS /usr/include /usr/local/include /usr/include/hobot /opt/ros/humble/include)
  find_library(DART_BPU_LIBRARY NAMES dnn
    PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu /usr/hobot/lib)
  if(DART_BPU_INCLUDE_DIR AND DART_BPU_LIBRARY)
    set(_bpu_ok ON)
  elseif(DART_ENABLE_BPU STREQUAL "ON")
    message(FATAL_ERROR "DART_ENABLE_BPU=ON 但找不到 hobot_dnn（头文件 ${DART_BPU_INCLUDE_DIR} / 库 ${DART_BPU_LIBRARY}）")
  else()
    message(STATUS "yq_dart_aim: 未找到 hobot_dnn，跳过 BPU 后端")
  endif()
endif()
if(_bpu_ok)
  target_sources(dart_aim_node PRIVATE src/model_backend_bpu.cpp)
  target_include_directories(dart_aim_node PRIVATE ${DART_BPU_INCLUDE_DIR})
  target_link_libraries(dart_aim_node ${DART_BPU_LIBRARY})
  target_compile_definitions(dart_aim_node PRIVATE DART_HAVE_BPU)
  message(STATUS "yq_dart_aim: BPU 后端 = 启用 (${DART_BPU_LIBRARY})")
endif()

# ── CPU: ONNX Runtime ──
set(_ort_ok OFF)
if(NOT DART_ENABLE_ORT STREQUAL "OFF")
  if(ORT_ROOT AND EXISTS "${ORT_ROOT}/include/onnxruntime_cxx_api.h")
    set(ORT_LIB onnxruntime)
    set(_ort_include "${ORT_ROOT}/include")
    set(_ort_linkdir "${ORT_ROOT}/lib")
  else()
    find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
      PATHS /usr/include /usr/local/include /usr/include/onnxruntime /usr/local/include/onnxruntime)
    find_library(ORT_LIB onnxruntime
      PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu)
    set(_ort_include ${ORT_INCLUDE_DIR})
    set(_ort_linkdir "")
  endif()
  if(ORT_LIB AND _ort_include)
    set(_ort_ok ON)
  elseif(DART_ENABLE_ORT STREQUAL "ON")
    message(FATAL_ERROR
      "DART_ENABLE_ORT=ON 但找不到 ONNX Runtime（头文件 onnxruntime_cxx_api.h / 库 libonnxruntime.so）。\n"
      "  指定路径：-DORT_ROOT=/path/to/onnxruntime-linux-aarch64\n"
      "  或关掉：  -DDART_ENABLE_ORT=OFF")
  else()
    message(STATUS "yq_dart_aim: 未找到 ONNX Runtime，跳过 CPU(ORT) 后端")
  endif()
endif()
if(_ort_ok)
  target_sources(dart_aim_node PRIVATE src/model_backend_ort.cpp)
  target_include_directories(dart_aim_node PRIVATE ${_ort_include})
  if(_ort_linkdir)
    target_link_directories(dart_aim_node PRIVATE ${_ort_linkdir})
    # 把 ORT 库目录写进 RPATH，保证运行时（含 systemd 启动）能找到 libonnxruntime.so
    set_target_properties(dart_aim_node PROPERTIES
      BUILD_RPATH "${_ort_linkdir}"
      INSTALL_RPATH "${_ort_linkdir}")
  endif()
  target_link_libraries(dart_aim_node ${ORT_LIB})
  target_compile_definitions(dart_aim_node PRIVATE DART_HAVE_ORT)
  message(STATUS "yq_dart_aim: CPU(ORT) 后端 = 启用 (${ORT_LIB})")
endif()

# ── DNN: OpenCV ──
set(_dnn_ok OFF)
if(NOT DART_ENABLE_DNN STREQUAL "OFF")
  set(_dnn_ok ON)
endif()
if(_dnn_ok)
  target_sources(dart_aim_node PRIVATE src/model_backend_dnn.cpp)
  target_compile_definitions(dart_aim_node PRIVATE DART_HAVE_DNN)
  message(STATUS "yq_dart_aim: DNN(OpenCV) 后端 = 启用")
endif()

if(NOT _bpu_ok AND NOT _ort_ok AND NOT _dnn_ok)
  message(FATAL_ERROR "没有任何推理后端可用，dart_aim_node 无法编译")
endif()

# 编译期指定的默认后端（params.yaml 没写 infer_backend 时生效）
target_compile_definitions(dart_aim_node PRIVATE
  DART_DEFAULT_INFER_BACKEND="${DART_INFER_BACKEND}")
message(STATUS "yq_dart_aim: 默认推理后端 = ${DART_INFER_BACKEND}")

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
