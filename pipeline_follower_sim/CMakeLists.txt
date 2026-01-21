cmake_minimum_required(VERSION 3.8)
project(pipeline_follower_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(pipeline_follower_node src/pipeline_follower_node.cpp)
ament_target_dependencies(pipeline_follower_node rclcpp sensor_msgs cv_bridge vortex_msgs tf2 tf2_geometry_msgs nav_msgs)
target_include_directories(pipeline_follower_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pipeline_follower_node ${OpenCV_LIBS})

install(TARGETS pipeline_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
