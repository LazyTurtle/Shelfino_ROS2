cmake_minimum_required(VERSION 3.5)
project(roadmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(roadmap_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(dubins_planner REQUIRED)
find_package(dubins_planner_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(manager src/roadmap_manager.cpp)
ament_target_dependencies(manager 
  rclcpp 
  nav_msgs 
  std_msgs 
  geometry_msgs 
  obstacles_msgs 
  roadmap_interfaces 
  std_srvs 
  visualization_msgs
  dubins_planner
  dubins_planner_msgs
  )

add_executable(driver src/roadmap_driver.cpp)
ament_target_dependencies(driver
  rclcpp
  geometry_msgs
  roadmap_interfaces
  std_srvs
  tf2_geometry_msgs
)

target_include_directories(manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  manager
  driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
