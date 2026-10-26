cmake_minimum_required(VERSION 3.8)
project(autonomous_logistics_sim)

find_package(ament_cmake REQUIRED)

# This command tells ROS 2 to copy your folders to the 'install' area
install(DIRECTORY
  launch
  worlds
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install maps
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python dispatcher script
install(PROGRAMS
  scripts/dispatcher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Behavior Tree XML
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install Python scripts (The dispatcher)
install(PROGRAMS
  scripts/dispatcher_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/obstacle_pacer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/simulation_master.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
