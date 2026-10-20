cmake_minimum_required(VERSION 3.16)
project(wdr_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps worlds urdf rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_warehouse.py scripts/initial_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Unit-level checks of the generated world/map/masks (fast, no ROS graph)
  ament_add_pytest_test(test_generated_files test/test_generated_files.py TIMEOUT 120)

  # End-to-end mission tests against Nav2's loopback simulator (headless, no Gazebo):
  # bring up Nav2 + the task manager, submit orders, check they are delivered.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_delivery_loopback.py TIMEOUT 600)

  # The same end-to-end check in Gazebo. Off by default (needs a GPU-capable renderer and
  # several minutes); enable with -DWDR_GAZEBO_TESTS=ON.
  option(WDR_GAZEBO_TESTS "Run the Gazebo end-to-end test" OFF)
  if(WDR_GAZEBO_TESTS)
    add_launch_test(test/test_delivery_gazebo.py TIMEOUT 1200)
  endif()
endif()

ament_package()
