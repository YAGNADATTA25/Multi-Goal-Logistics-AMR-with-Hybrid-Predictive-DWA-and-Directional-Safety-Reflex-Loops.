### `Multi-Goal-Logistics-AMR`

```markdown
# Multi-Goal Logistics AMR: Hybrid Predictive DWA & Directional Safety Reflex Loops

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)
![C++17](https://img.shields.io/badge/Language-C%2B%2B17-green.svg)
![Python 3.10](https://img.shields.io/badge/Language-Python_3.10-yellow.svg)
![Nav2 Stack](https://img.shields.io/badge/Stack-Nav2-brightgreen.svg)
![Gazebo Simulator](https://img.shields.io/badge/Simulator-Gazebo_Classic-orange.svg)
![License](https://img.shields.io/badge/License-Apache_2.0-red.svg)

An industrial-grade ROS 2 software architecture engineered for Autonomous Mobile Robots (AMRs) operating in dynamic, highly unconstrained warehouse environments. The system couples multi-goal dispatching with predictive Dynamic Window Approach (DWA) local trajectory generation and a low-latency safety reflex override loop.

```

---

## 🏗 System Architecture & ROS 2 Data Pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      High-Level Mission Control                         │
│   Sequential Workstation Coordinate Dispatcher (Action Client Loop)    │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │ nav2_msgs/action/NavigateThroughPoses
┌────────────────────────────────────▼────────────────────────────────────┐
│                    ROS 2 Nav2 Asynchronous Core                         │
│          Global Path Generation (nav_msgs/msg/Path via /plan)           │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │ /plan Vector Intercept
┌────────────────────────────────────▼────────────────────────────────────┐
│                       Simulation Master Node                            │
│  ┌─────────────────────────────────┴─────────────────────────────────┐  │
│  │ 10 Hz Predictive Control Loop                                     │  │
│  │   ├─ 1.8s Predictive Horizon Horizon Evaluation (9 Steps)         │  │
│  │   └─ Dynamic Obstacle Velocity Window Sampling (DWA)              │  │
│  │ 0.9 s Telemetry & State Monitor Clock                             │  │
│  │   ├─ Cross-Track Error (cm) & Heading Deviation Calculation       │  │
│  │   └─ Goal Proximity & Kinematic Limit Verification                │  │
│  └─────────────────────────────────┬─────────────────────────────────┘  │
└────────────────────────────────────┼────────────────────────────────────┘
                                     │ /scan (sensor_msgs/msg/LaserScan)
                                     │ /odom (nav_msgs/msg/Odometry)
┌────────────────────────────────────▼────────────────────────────────────┐
│                  Directional Safety Reflex Loop                         │
│  Low-Latency Emergency Deceleration / Velocity Scaling Intercept       │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │ Topic: /cmd_vel (geometry_msgs/msg/Twist)
┌────────────────────────────────────▼────────────────────────────────────┐
│                     Target Hardware / Gazebo Sim                        │
│                 (TurtleBot3 Waffle Pi / Differential Drive)             │
└─────────────────────────────────────────────────────────────────────────┘

```

---

## 🔑 Key Engineering & R&D Highlights

* **Asynchronous Multi-Goal Dispatching:** Implements non-blocking action invocation via `nav2_simple_commander` (`BasicNavigator`) to queue and execute waypoint vectors across discrete warehouse workstation coordinates.
* **Predictive Dynamic Window Approach (DWA):** Evaluates admissible velocity pairs $(v, \omega)$ over a **1.8s predictive horizon (9 discrete integration steps)** to ensure collision-free local trajectories around dynamic obstacles.
* **Directional Safety Reflex Override:** Deterministic safety layer intercepting velocity outputs when dynamic obstacles cross critical angular LiDAR safety zones (asymmetric shielding), ensuring zero-collision guarantees under unpredictable motion profiles.
* **Closed-Loop Telemetry Stream:** Asynchronous 0.9s diagnostics module tracking real-time **Cross-Track Error (cm)**, **Heading Deviation (deg)**, and **Distance-to-Goal** metrics for real-time control system feedback.
* **Modular Software Engineering:** Clean workspace separation with parameterized YAML files, modular ROS 2 launch pipelines, and asynchronous execution threads.

---

## 📊 Technical Specifications & Parameters

| Metric / Parameter | Value | R&D Description |
| --- | --- | --- |
| **ROS 2 Middleware** | Humble Hawksbill | LTS Production Target |
| **Control Loop Frequency** | 10 Hz (100 ms loop) | Local trajectory evaluation cycle |
| **Predictive Horizon** | 1.8 seconds | Forward kinetic trajectory projection |
| **Prediction Steps** | 9 discrete steps | Trajectory sampling resolution |
| **Telemetry Clock** | 0.9 seconds | Independent monitoring thread |
| **Target Platform** | TurtleBot3 Waffle Pi | Differential Drive Kinematics |

---

## 💻 Tech Stack & Dependencies

* **Middleware & Frameworks:** ROS 2 Humble, Nav2 Stack, TF2 Transformation Tree
* **Programming Languages:** C++17, Python 3.10
* **ROS 2 Interface Types:** `geometry_msgs/msg/Twist`, `sensor_msgs/msg/LaserScan`, `nav_msgs/msg/Odometry`, `nav_msgs/msg/Path`
* **Simulation Environment:** Gazebo Classic 11 / RViz2

---

## 🚀 Build & Execution Guide

### Prerequisites

Ensure ROS 2 Humble and Gazebo Classic are sourced on Ubuntu 22.04 LTS.

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone [https://github.com/YAGNADATTA25/Multi-Goal-Logistics-AMR-with-Hybrid-Predictive-DWA-and-Directional-Safety-Reflex-Loops..git](https://github.com/YAGNADATTA25/Multi-Goal-Logistics-AMR-with-Hybrid-Predictive-DWA-and-Directional-Safety-Reflex-Loops..git)

# 2. Resolve workspace dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build workspace
colcon build --symlink-install --packages-select amr_logistics
source install/setup.bash

# 4. Launch Simulation Environment & Navigation Nodes
ros2 launch amr_logistics main_simulation.launch.py

---
----
2. **Explicit Data Types:** Lists exact ROS 2 message types (`nav_msgs/msg/Path`, `sensor_msgs/msg/LaserScan`), proving you understand underlying ROS 2 middleware structures.
3. **Structured Parameter Table:** Highlighting frequency, prediction steps, and horizon time in a clean matrix immediately proves engineering control rigor.
4. **Clean ASCII Diagram:** Features a full closed-loop architecture chart separating high-level mission planning, the 10Hz DWA control loop, and low-level motor commands.
