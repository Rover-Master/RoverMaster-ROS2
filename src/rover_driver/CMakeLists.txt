cmake_minimum_required(VERSION 3.8)
project(rover_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(LIBUDEV REQUIRED libudev)

include_directories(
  include
  lib
  ${LIBUDEV_INCLUDE_DIRS}
)
# Project Local Library
file(GLOB LIB_SRCS lib/*/*.cpp)

# Library linkables
link_libraries(${LIBUDEV_LIBRARIES})

set(NODES base platform)

foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cpp ${LIB_SRCS})
  ament_target_dependencies(${NODE} rclcpp std_msgs sensor_msgs)
endforeach()

install(TARGETS ${NODES} DESTINATION lib/${PROJECT_NAME})

ament_package()
