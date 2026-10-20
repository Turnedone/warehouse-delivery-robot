cmake_minimum_required(VERSION 3.16)
project(wdr_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(wdr_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Order queue, station registry, state -> error code mapping (no ROS graph needed)
add_library(wdr_mission_core SHARED src/station_registry.cpp src/mission_state.cpp src/energy.cpp)
target_include_directories(wdr_mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wdr_mission_core PUBLIC yaml-cpp ${wdr_interfaces_TARGETS})

# BT plugin library, loaded by name ("wdr_mission_bt_nodes") next to Nav2's BT plugins
add_library(wdr_mission_bt_nodes SHARED src/bt_nodes.cpp)
target_compile_definitions(wdr_mission_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(wdr_mission_bt_nodes PUBLIC wdr_mission_core behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp ${geometry_msgs_TARGETS})

add_executable(task_manager src/task_manager.cpp)
target_link_libraries(task_manager wdr_mission_core rclcpp::rclcpp rclcpp_action::rclcpp_action tf2_ros::tf2_ros
  behaviortree_cpp::behaviortree_cpp ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_srvs_TARGETS}
  ${nav2_msgs_TARGETS} ${wdr_interfaces_TARGETS})
# nav2_behavior_tree exports no CMake targets in Jazzy, only the classic variables
ament_target_dependencies(task_manager nav2_behavior_tree)

install(TARGETS wdr_mission_core wdr_mission_bt_nodes
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS task_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_order_queue test/test_order_queue.cpp)
  target_link_libraries(test_order_queue wdr_mission_core)
  ament_add_gtest(test_energy test/test_energy.cpp)
  target_link_libraries(test_energy wdr_mission_core)
  ament_add_gtest(test_station_registry test/test_station_registry.cpp)
  target_link_libraries(test_station_registry wdr_mission_core)
  # Compiles the BT nodes into the test directly instead of loading the plugin
  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp src/bt_nodes.cpp)
  target_compile_definitions(test_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(test_bt_nodes wdr_mission_core behaviortree_cpp::behaviortree_cpp rclcpp::rclcpp
    ${geometry_msgs_TARGETS})
endif()

ament_package()
