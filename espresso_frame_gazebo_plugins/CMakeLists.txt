cmake_minimum_required(VERSION 3.8)
project(espresso_frame_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mavros_move_plugin SHARED
  src/mavros_move_plugin.cpp
)
target_include_directories(mavros_move_plugin PUBLIC include)
ament_target_dependencies(mavros_move_plugin
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "gazebo_ros"
  "rclcpp"
  "tf2_ros"
)
ament_export_libraries(mavros_move_plugin)
install(TARGETS
  mavros_move_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_library(feetech_turret_plugin SHARED
  src/feetech_turret_plugin.cpp
)
target_include_directories(feetech_turret_plugin PUBLIC include)
ament_target_dependencies(feetech_turret_plugin
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "gazebo_ros"
  "rclcpp"
  "tf2_ros"
)
ament_export_libraries(feetech_turret_plugin)
install(TARGETS
  feetech_turret_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
