cmake_minimum_required(VERSION 3.8)
project(rover)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)

set(DEPS rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)
foreach(DEP ${DEPS})
  find_package(${DEP} REQUIRED)
endforeach()
find_package(Threads REQUIRED)

pkg_check_modules(LIBUDEV REQUIRED libudev)

include_directories(
  include
  lib
  ${LIBUDEV_INCLUDE_DIRS}
)
link_libraries(${LIBUDEV_LIBRARIES})

file(GLOB SRCS src/*.cpp src/**/*.cpp)
file(GLOB LIB_SRCS lib/**/*.cpp)
file(GLOB NODES nodes/*.cpp)

foreach(NODE ${NODES})
  get_filename_component(NODE_NAME ${NODE} NAME)
  string(REPLACE ".cpp" "" NODE_NAME ${NODE_NAME})
  add_executable(${NODE_NAME} ${NODE} ${SRCS} ${LIB_SRCS})
  ament_target_dependencies(${NODE_NAME} ${DEPS})
  install(TARGETS ${NODE_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
