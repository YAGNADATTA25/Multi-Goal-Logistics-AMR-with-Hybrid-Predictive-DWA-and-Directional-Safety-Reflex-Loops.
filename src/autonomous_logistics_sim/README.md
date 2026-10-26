# Autonomous Logistics AMR with Hybrid Predictive DWA and Safety Reflex Loops

This repository contains the software package for an Autonomous Mobile Robot (AMR) designed for dense, dynamic warehouse logistics. The framework is built on **ROS 2 Humble** and features a hybrid navigation pipeline that couples the standard Nav2 stack with a custom predictive local controller and 360-degree directional safety reflexes to handle high-speed dynamic obstacles.

---

## 🛠️ Subsystem Architecture

The system is fully modular, splitting duties into three asynchronous pipelines over the ROS 2 DDS middleware:
1. **Perception & Mapping:** Processes 360° LiDAR data via SLAM to build and serve static occupancy grid blueprints.
2. **Navigation & Tracking (Nav2):** Handles global path routing and nominal lane tracking through an optimized, high-frequency controller.
3. **Safety Reflex Node (Our Custom Code):** Intercepts raw sensor ranges, runs a forward trajectory simulation, and enforces instantaneous overrides if dynamic threats are detected.

---

## 🚀 Launch & Execution Sequence

Always remember to build your workspace cleanly and source your bash environment prior to executing commands.

### 1. Build and Setup Workspace
```bash
# Clean previous caches if necessary, build, and source
colcon build --symlink-install
source install/setup.bash

2. Launch the Master Simulation
PYTHONUNBUFFERED=1 ros2 launch your_custom_package_name simulation_master.launch.py navigation:=true
3. Generate Dynamic Obstacle Traffic
In a separate terminal:
ros2 run your_custom_package_name dynamic_obstacle_generator
