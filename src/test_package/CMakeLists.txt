cmake_minimum_required(VERSION 3.8)
project(test_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# add_service_files(
#   SquareNumber.srv

# )

rosidl_generate_interfaces(${PROJECT_NAME} 
            "srv/SquareNumber.srv")

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
   ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(node_a src/node_a.cpp)
ament_target_dependencies(node_a rclcpp )
target_link_libraries(node_a "${cpp_typesupport_target}")

add_executable(node_b src/node_b.cpp)
ament_target_dependencies(node_b rclcpp )
target_link_libraries(node_b "${cpp_typesupport_target}")

add_executable(turtle src/turtle.cpp)
ament_target_dependencies(turtle rclcpp geometry_msgs)

add_executable(turtle_follower src/turtle_follower.cpp)
ament_target_dependencies(
    turtle_follower
    rclcpp
    turtlesim
    tf2
    tf2_ros
)

add_executable(turtle_tf_publisher src/turtle_tf_publisher.cpp)
ament_target_dependencies(turtle_tf_publisher rclcpp tf2_ros geometry_msgs)

install(TARGETS 
    talker
    listener
    node_a
    node_b
    turtle
    turtle_tf_publisher
    turtle_follower
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
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
