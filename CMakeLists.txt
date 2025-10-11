cmake_minimum_required(VERSION 3.5)
project(lawn_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(Hello_Lawn_Robot src/Hello_Lawn_Robot.cpp)
ament_target_dependencies(Hello_Lawn_Robot rclcpp std_msgs)

install(TARGETS
  Hello_Lawn_Robot
  DESTINATION lib/${PROJECT_NAME})

ament_package()