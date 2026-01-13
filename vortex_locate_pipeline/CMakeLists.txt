cmake_minimum_required(VERSION 3.8)
project(vortex_locate_pipeline)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Core library (ROS-independent implementation)
add_library(locator_core src/core/locator_core.cpp)
target_include_directories(locator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(locator_core ${OpenCV_LIBS})

# ROS node binary (thin wrapper)
add_executable(pipeline_locator_node src/ros/locate_pipeline_node.cpp src/ros/locator.cpp)
target_include_directories(pipeline_locator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(pipeline_locator_node rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(pipeline_locator_node locator_core ${OpenCV_LIBS})

# Install node binary
install(TARGETS
  pipeline_locator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install core library and export targets for downstream packages
install(TARGETS locator_core
  EXPORT locator_core-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT locator_core-targets FILE locator_core-targets.cmake DESTINATION lib/cmake/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# Export for ament consumers
ament_export_targets(locator_core-targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(OpenCV)

# install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
