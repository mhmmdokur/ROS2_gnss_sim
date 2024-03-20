cmake_minimum_required(VERSION 3.8)
project(gnss_nav_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gnss_nav src/linear_route_node.cpp)
ament_target_dependencies(gnss_nav rclcpp sensor_msgs swri_transform_util visualization_msgs geometry_msgs)

install(TARGETS 
  gnss_nav
  DESTINATION lib/${PROJECT_NAME}
)

# install launch directory
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
