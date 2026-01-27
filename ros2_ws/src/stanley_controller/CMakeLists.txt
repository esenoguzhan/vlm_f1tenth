cmake_minimum_required(VERSION 3.8)
project(stanley_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(stanley_node src/StanleyController.cpp)
ament_target_dependencies(stanley_node 
    rclcpp nav_msgs std_msgs ackermann_msgs tf2 visualization_msgs ament_index_cpp)

install(TARGETS stanley_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY racelines
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()