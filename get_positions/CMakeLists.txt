cmake_minimum_required(VERSION 3.8)
project(get_positions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(get_positions 
  src/get_positions.cpp
)
ament_target_dependencies(get_positions 
  rclcpp 
  geometry_msgs
  tf2_ros
  std_srvs
)

install(TARGETS
  get_positions
  DESTINATION lib/${PROJECT_NAME})

ament_package()
