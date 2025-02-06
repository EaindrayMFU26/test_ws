cmake_minimum_required(VERSION 3.5)
project(test_five)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(test_five_msgs REQUIRED)

add_executable(custom_publisher src/custom_publisher.cpp)
ament_target_dependencies(custom_publisher rclcpp test_five_msgs)

add_executable(custom_subscriber src/custom_subscriber.cpp)
ament_target_dependencies(custom_subscriber rclcpp test_five_msgs)

install(TARGETS
  custom_publisher
  custom_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
