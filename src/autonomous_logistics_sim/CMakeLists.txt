cmake_minimum_required(VERSION 3.8)
project(autonomous_logistics_sim)

find_package(ament_cmake REQUIRED)

# Install the Gazebo world, Nav2 configs, maps, models and launch files.
install(
  DIRECTORY
    launch
    worlds
    config
    maps
    params
    behavior_trees
    models
  DESTINATION share/${PROJECT_NAME}
)

# Python ROS 2 executables, available through "ros2 run autonomous_logistics_sim ...".
install(
  PROGRAMS
    scripts/simulation_master.py
    scripts/dispatcher_node.py
    scripts/obstacle_pacer.py
    scripts/telemetry_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
