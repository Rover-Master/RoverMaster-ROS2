cmake_minimum_required(VERSION 3.8)
project(rover_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# Driver Dependencies
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  lib
  ${OpenCV_INCLUDE_DIRS}
)
# Project Local Library
file(GLOB LIB_SRCS lib/*/*.cpp)

# Library linkables
link_directories(
  ${LIBUSB_INCLUDE_DIR}
  ${OpenCV_LIB_DIRS}
)

set(NODES base platform)

foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cpp ${LIB_SRCS})
  ament_target_dependencies(${NODE}
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    Threads
    OpenCV
  )
endforeach()

install(TARGETS ${NODES}
  DESTINATION lib/${PROJECT_NAME}
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
