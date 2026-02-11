cmake_minimum_required(VERSION 3.8)
project(vortex_pipeline_3d)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

# Geometry library (ROS-independent backprojection)
add_library(pipeline_geometry
  src/core/geometry.cpp
)
target_include_directories(pipeline_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pipeline_geometry ${OpenCV_LIBS})

# ROS node executable
add_executable(localizer_node
  src/ros/localizer_node.cpp
  src/localizer_node_main.cpp
  src/visualization/image_overlay.cpp
  src/visualization/marker_publisher.cpp
)
target_link_libraries(localizer_node pipeline_geometry ${OpenCV_LIBS})
ament_target_dependencies(localizer_node
  rclcpp sensor_msgs geometry_msgs std_msgs nav_msgs vortex_msgs
  cv_bridge visualization_msgs
)

# Install targets
install(TARGETS pipeline_geometry
  EXPORT pipeline_geometry-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export library for downstream packages
ament_export_targets(pipeline_geometry-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)

ament_package()
