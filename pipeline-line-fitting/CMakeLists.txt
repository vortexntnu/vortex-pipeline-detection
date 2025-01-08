cmake_minimum_required(VERSION 3.5)
project(pipeline-detection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Armadillo REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

# Define the randsac library
add_library(randsac src/randsac.cpp)
ament_target_dependencies(randsac rclcpp)


# Define the LinedetectorPipe library and link it with randsac, OpenCV, and Armadillo
add_library(LinedetectorPipe src/linedetectorPipe.cpp)
target_link_libraries(LinedetectorPipe randsac ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES})

# Define an executable that uses the linefitting library
add_executable(linefitting src/linefitting.cpp)
target_link_libraries(linefitting LinedetectorPipe)
ament_target_dependencies(linefitting rclcpp geometry_msgs sensor_msgs cv_bridge)

#another executable for a debug reciver
add_executable(debugreciver src/debugreciver.cpp)
ament_target_dependencies(debugreciver rclcpp geometry_msgs)

install(TARGETS
  randsac
  LinedetectorPipe
  linefitting
  debugreciver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME})

ament_package()