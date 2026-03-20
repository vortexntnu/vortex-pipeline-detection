cmake_minimum_required(VERSION 3.8)
project(vortex_pipeline_position_estimator)

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
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Geometry library (pure math, no ROS dependencies)
add_library(position_estimator_geometry
  src/core/backproject_ground_plane.cpp
)
target_include_directories(position_estimator_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(position_estimator_geometry ${OpenCV_LIBS})

# Node component (composable)
add_library(position_estimator_component SHARED
  src/ros/position_estimator_node.cpp
)
target_link_libraries(position_estimator_component position_estimator_geometry ${OpenCV_LIBS})
ament_target_dependencies(position_estimator_component
  rclcpp rclcpp_components sensor_msgs geometry_msgs std_msgs vortex_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
rclcpp_components_register_node(position_estimator_component
  PLUGIN "vortex_pipeline_position_estimator::PositionEstimatorNode"
  EXECUTABLE position_estimator_node
)

# Install targets
install(TARGETS position_estimator_geometry
  EXPORT position_estimator_geometry-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS position_estimator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS position_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export library for downstream packages
ament_export_targets(position_estimator_geometry-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)

ament_package()
