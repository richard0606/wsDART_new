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

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
