cmake_minimum_required(VERSION 3.8)
project(vortex_pipeline_image_endpoints)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Core library (ROS-independent)
add_library(image_endpoints_core
  src/core/detector.cpp
  src/core/lowest_pixel.cpp
)
target_include_directories(image_endpoints_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(image_endpoints_core ${OpenCV_LIBS})

# ROS node executable
add_executable(image_endpoints_node
  src/ros/detector_node.cpp
  src/detector_node_main.cpp
)
target_link_libraries(image_endpoints_node image_endpoints_core ${OpenCV_LIBS})
ament_target_dependencies(image_endpoints_node
  rclcpp sensor_msgs std_msgs cv_bridge vortex_msgs
)

# Install targets
install(TARGETS image_endpoints_core
  EXPORT image_endpoints_core-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS image_endpoints_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export library for downstream packages
ament_export_targets(image_endpoints_core-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)

ament_package()
