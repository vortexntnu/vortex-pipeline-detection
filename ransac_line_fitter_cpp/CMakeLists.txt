cmake_minimum_required(VERSION 3.8)
project(ransac_line_fitter_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ransac_line_node src/ransac_line_node.cpp)
ament_target_dependencies(ransac_line_node rclcpp sensor_msgs cv_bridge)
target_include_directories(ransac_line_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ransac_line_node ${OpenCV_LIBS})

install(TARGETS ransac_line_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
