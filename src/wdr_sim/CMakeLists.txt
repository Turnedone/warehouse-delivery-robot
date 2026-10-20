cmake_minimum_required(VERSION 3.16)
project(wdr_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(wdr_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# ROS-free models, unit tested on their own
add_library(wdr_sim_models STATIC src/battery_model.cpp src/path_player.cpp)
set_target_properties(wdr_sim_models PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(wdr_sim_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(battery_sim src/battery_sim_node.cpp)
target_link_libraries(battery_sim wdr_sim_models rclcpp::rclcpp tf2_ros::tf2_ros
  ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${wdr_interfaces_TARGETS})

add_executable(obstacle_mover src/obstacle_mover_node.cpp)
target_link_libraries(obstacle_mover wdr_sim_models rclcpp::rclcpp yaml-cpp
  ${ros_gz_interfaces_TARGETS} ${visualization_msgs_TARGETS})

install(TARGETS battery_sim obstacle_mover DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_battery_model test/test_battery_model.cpp)
  target_link_libraries(test_battery_model wdr_sim_models)
  ament_add_gtest(test_path_player test/test_path_player.cpp)
  target_link_libraries(test_path_player wdr_sim_models)
endif()

ament_package()
