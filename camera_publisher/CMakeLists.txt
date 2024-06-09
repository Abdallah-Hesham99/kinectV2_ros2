cmake_minimum_required(VERSION 3.5)
project(camera_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs cv_bridge OpenCV)


add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp sensor_msgs cv_bridge OpenCV)

install(DIRECTORY include/
  DESTINATION include/)


target_link_libraries(camera_publisher k4w2)
target_link_libraries(pub k4w2)

install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pub
  DESTINATION lib/${PROJECT_NAME})
ament_package()

