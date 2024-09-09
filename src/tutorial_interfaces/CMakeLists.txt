cmake_minimum_required(VERSION 3.8)
project(tutorial_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(tutorial_interfaces "srv/AddThreeInts.srv")
find_package(tutorial_interfaces REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(server src/add_three_ints_server.cpp)
ament_target_dependencies(server rclcpp tutorial_interfaces)

add_executable(client src/add_three_ints_client.cpp)
ament_target_dependencies(client rclcpp tutorial_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
