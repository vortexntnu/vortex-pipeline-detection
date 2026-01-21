cmake_minimum_required(VERSION 3.8)
project(irls_line_fitter_2x)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vortex_msgs REQUIRED)


add_executable(irls_line_node src/irls_line_node.cpp)
ament_target_dependencies(irls_line_node rclcpp sensor_msgs cv_bridge vortex_msgs)
target_include_directories(irls_line_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(irls_line_node ${OpenCV_LIBS})

install(TARGETS irls_line_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
