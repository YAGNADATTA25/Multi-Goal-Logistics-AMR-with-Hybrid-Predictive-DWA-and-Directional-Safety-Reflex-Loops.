# Warehouse AMR Navigation with Hybrid Predictive DWA

**ROS 2 Humble · Nav2 · Gazebo Classic · TurtleBot3 Waffle Pi · Python**

A simulated autonomous mobile robot (AMR) for multi-goal warehouse navigation in narrow aisles with moving obstacles. The system uses **Nav2 for obstacle-aware route planning and path following**, while a custom Python **Simulation Master** supervises velocity commands using LiDAR-based obstacle tracking and predictive Dynamic Window Approach (DWA) trajectory evaluation.

> **Project status:** Validated in **Gazebo simulation**, not on physical hardware. One recorded run verified all six goals. The results are from a locally tested configuration. The committed source is being synchronized with that configuration; exact reproduction still needs runtime validation on Ubuntu 22.04 / ROS 2 Humble.

## Demonstration highlights

- **6/6 navigation goals verified** in a warehouse mission.
- **Four moving obstacles** provide repeatable dynamic-crossing scenarios.
- The robot navigated **31.94 m in 155 s** and traversed narrow passages in the successful recorded run.
- The master checks predicted robot motion against LiDAR detections and keeps its output close to Nav2's requested movement when safe.
- Gazebo and RViz2 visualize robot motion, costmaps, and planned paths.

**[Watch the full Gazebo + RViz demonstration](https://yagnadatta25.github.io/logistics-amr.html)** · [Portfolio](https://yagnadatta25.github.io/) · [Recorded run summary](demo_results/warehouse_run_01_summary.txt)

## System architecture

```text
                 Six warehouse goals
                         |
                         v
             Mission / BasicNavigator
                         |
                         v
               ROS 2 Nav2 stack
         Global planner + DWB controller
                  |          ^
             /cmd_vel_nav    | map + LiDAR costmaps
                  |          |
                  v          |
         Custom Simulation Master <---- /scan, /odom, TF
          - LiDAR clustering
          - Obstacle motion estimation
          - Predictive DWA supervision
          - Motion / mission telemetry
                  |
           /cmd_vel_supervised
                  |
                  v
        Collision Monitor (if enabled)
                  |
               /cmd_vel
                  |
                  v
       TurtleBot3 Waffle Pi in Gazebo
                  |
            sensor feedback
                  +--------------------------> Nav2 + master

        AMCL estimates map-relative pose using a known map.
        Gazebo also simulates four moving obstacles.
```

**Responsibilities:** Gazebo simulates the environment and sensors; AMCL estimates localisation in the known map; Nav2 updates costmaps, plans routes and proposes robot velocities; the Simulation Master evaluates predicted safety; Collision Monitor, when launched and correctly connected, provides an additional stop/slowdown layer. The master **does not replace Nav2's global planner**.

## Predictive controller

1. Receive 360° LiDAR scans and transform detections into the `odom` frame using TF2.
2. Group nearby scan points and associate detections across successive scans to estimate obstacle velocities.
3. Receive Nav2's proposed linear and angular velocity on `/cmd_vel_nav`.
4. Evaluate candidate differential-drive motions, including predicted separation from moving obstacles.
5. Reject unsafe candidates; prefer an admissible command that stays close to Nav2's request.
6. Publish the supervised command to the configured output topic. Repeat with new sensor data; stop if required inputs are stale or clearance is insufficient.

### Selected configuration

| Parameter | Value |
|---|---:|
| Control frequency | **10 Hz** (100 ms nominal period) |
| Linear velocity samples | **25** |
| Angular velocity samples | **25** |
| Grid combinations | **625** |
| Prediction horizon | **3.0 s** |
| Prediction step | **0.2 s** |
| Steps per candidate | **15** |
| Maximum configured linear speed | **0.26 m/s** |

The controller may also add explicit command/stop/turn candidates, depending on the committed master-node version; **625 refers to the sampling grid**.

Earlier development used **15 × 15 = 225** grid candidates and a **1.8 s** horizon with the same **0.2 s** step. The selected configuration explores more possible motions and a longer future interval, at greater computational cost. This is a **design comparison**, not a controlled performance benchmark between versions.

## Results: recorded warehouse mission

The following measurements are from **one successful Gazebo run**, analyzed from ROS 2 bag data and Simulation Master telemetry. **These numbers are not estimates or manually assigned scores:** they are the output of a Python analysis script run against `warehouse_run_01`, as reported by the project author. The exact stdout is preserved in [`demo_results/warehouse_run_01_summary.txt`](demo_results/warehouse_run_01_summary.txt).

> **Evidence status:** The summary output was supplied by the project author. The original `.db3` bag has not yet been published with this README, so an external reviewer can inspect the method but cannot independently recalculate the run until the raw recording is shared. The verified working source-code versions should also be committed alongside it.

### How the data were collected

1. Launch the **locally validated** Gazebo + Nav2 configuration and start `simulation_master.py`. Use the launch/YAML/master files from the successful run, not an older repository revision.
2. Start a ROS 2 bag recording **before** the six-goal mission, keeping `/clock` and enabling simulated timestamps. Capture at least `/odom`, `/scan`, `/plan`, and `/rosout`, because these four topics are read by the results script. Additional topics are useful for later verification.
3. Execute the complete six-goal mission and stop recording only **after** the final goal-completion log. The file analyzed for the results below was `warehouse_run_01`.
4. Read the recorded messages offline with `rosbag2_py`, deserialize the ROS 2 message types, and compute the statistics shown below. The analysis program is included as [`demo_results/analyze_run.py`](demo_results/analyze_run.py).

Example recording command (run in a separate terminal before starting a new mission):

```bash
mkdir -p ~/ros2_ws/demo_results
ros2 bag record --use-sim-time -o ~/ros2_ws/demo_results/warehouse_run_01 \
  /clock /odom /scan /amcl_pose /tf /tf_static /plan \
  /cmd_vel_nav /cmd_vel_supervised /cmd_vel /rosout \
  /navigate_to_pose/_action/status
```

Stop the recorder with `Ctrl+C`, then inspect and analyze the saved bag:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 bag info ~/ros2_ws/demo_results/warehouse_run_01
python3 demo_results/analyze_run.py ~/ros2_ws/demo_results/warehouse_run_01 \
  | tee demo_results/warehouse_run_01_summary.txt
```

> Run the analysis command from your ROS workspace root (for example `~/ros2_ws`) after placing the script in `demo_results/`. The script requires ROS 2 Humble's Python packages, including `rosbag2_py`, `rclpy`, `rosidl_runtime_py`, and the ROS message definitions. Do not replay `/tf` from this bag into a live Gazebo navigation run.

### Exactly how each metric was calculated

| Reported metric | Data source | Calculation in `analyze_run.py` |
|---|---|---|
| **6 verified goals** | `/rosout` | Count distinct `Goal N/6 verified` messages; the six unique goal indices were found. This depends on the master's position-verification log logic. |
| **155 s mission time** | `/rosout` + bag record timestamps | Time difference between the first `Navigating to goal 1/6` (or `Following 6 goals`) log and the final `All 6 goals verified` (or other recognized completion) log. |
| **31.94 m distance** | `/odom` | Sum consecutive 2D odometry displacement lengths; discard increments at or below `0.001 m` and at or above `0.25 m` to limit noise and discontinuities. |
| **0.235 m/s mean moving speed** | `/odom` | Mean of `abs(twist.twist.linear.x)` for odometry samples with speed above `0.01 m/s`. This excludes stationary samples. |
| **0.271 m/s maximum speed** | `/odom` | Maximum sampled absolute odometry linear speed. |
| **3.26 cm mean cross-track error** | `/rosout` | Arithmetic mean of values parsed from the master's `XT=... cm` telemetry messages. Not independent ground-truth error. |
| **24.54 ms mean DWA time** | `/rosout` | Arithmetic mean of values parsed from `DWA=... ms` telemetry messages; sampled reporting rather than tracing every cycle. |
| **95.00 ms maximum DWA time** | `/rosout` | Largest `DWA=... ms` value in the captured telemetry logs; **not guaranteed to be the worst individual control cycle**. |
| **155 path messages** | `/plan` | Count of all recorded `nav_msgs/msg/Path` messages. |
| **150 changed path geometries** | `/plan` | Compare successive path-coordinate sequences after rounding positions to two decimal places; the first observed path also counts as a change. **Not the number of obstacle-triggered replans.** |
| **0.120 m minimum valid scan range** | `/scan` | Minimum finite LiDAR range across scans within each message's declared valid range. This is scanner distance, **not robot-body clearance**. |
| **0 stop / 0 slowdown log matches** | `/rosout` | Search for exact Collision Monitor stop/slowdown log phrases. **No matching recorded logs does not mean no stopping, collisions, or safety interventions.** |

### Recorded measurement summary

| Metric | Result |
|---|---:|
| Verified goals | **6 / 6** |
| Mission duration | **155 s (2 min 35 s)** |
| Total odometry distance | **31.94 m** |
| Mean speed while moving | **0.235 m/s** |
| Maximum measured speed | **0.271 m/s** |
| Mean reported cross-track error | **3.26 cm** |
| Mean reported DWA computation time | **24.54 ms** |
| Maximum reported DWA computation time | **95.00 ms** |
| Nav2 global-path messages | **155** |
| Changed path geometries | **150** |
| Minimum valid LiDAR range | **0.120 m** |
| Collision Monitor stop / slowdown log events found in bag | **0 / 0** |

### Interpretation and measurement limits

- Six waypoint completions were **individually verified in the recorded mission logs**; the result is not merely an action-status count.
- **3.26 cm** is the mean of *logged* cross-track-error samples, not a separately calibrated ground-truth positioning error.
- **150 changed path geometries are not 150 obstacle-triggered replans**: periodic planning updates can alter path geometry.
- **0.120 m is a LiDAR range**, not verified clearance between the robot body and an obstacle.
- **0 captured Collision Monitor events does not prove zero collisions or zero safety interventions**; those events may not have been present in the recorded logs. Collision/contact outcomes require dedicated verification.
- At a nominal **10 Hz** control rate, the **24.54 ms** mean computation time is below the 100 ms period; the **95 ms** maximum approaches that limit. Jitter and deadline misses have not yet been characterized.
- Figures represent **one recorded simulation run**, not statistical performance over repeated trials or a comparison with baseline Nav2.

## ROS 2 interfaces

| Topic / interface | Purpose |
|---|---|
| `/scan` | LiDAR measurements used by Nav2 and the master |
| `/odom` | Robot odometry / travel estimation |
| `/amcl_pose` and TF | Localisation and coordinate transformations |
| `/plan` | Nav2 global path; monitoring and cross-track feedback |
| `/cmd_vel_nav` | Nav2-requested velocity (in the supervised configuration) |
| `/cmd_vel_supervised` | Master output to Collision Monitor (if enabled) |
| `/cmd_vel` | Final robot velocity input |
| `/cmd_vel_obstacle_1` … `/cmd_vel_obstacle_4` | Commands for four simulated moving obstacles |

**Routing matters:** The exact velocity topics depend on the *working launch file*. Check `ros2 topic info -v` and confirm that the master, Collision Monitor (if present), and Gazebo drive plugin form **one unambiguous control path**.

## Repository layout

```text
src/
  autonomous_logistics_sim/
    launch/warehouse_navigation.launch.py
    params/warehouse_params.yaml
    scripts/simulation_master.py
    worlds/warehouse.world
    maps/warehouse_map.yaml
    maps/warehouse_map.pgm
    behavior_trees/
    models/
    CMakeLists.txt
    package.xml
  autonomous_dispatcher/
```

The source repository also contains TurtleBot3-related components. Install or source their ROS 2 packages as appropriate for your workspace. Git submodule pointers included under `src/` may require separate checkout and are not needed if those packages are already installed.

## Requirements

- Ubuntu 22.04 with **ROS 2 Humble**
- **Nav2** (`navigation2`, `nav2_bringup`, `nav2_simple_commander`)
- **Gazebo Classic** with `gazebo_ros` integration
- **TurtleBot3 Waffle Pi** robot description and simulation assets
- Python 3 with **NumPy**, TF2, RViz2, colcon, and rosdep

The project was validated in simulation. Hardware operation would require additional robot-specific safety and integration work.

## Build and run

Clone the repository into a ROS 2 workspace (the repository name includes a trailing period, so choose a clean local folder name):

```bash
git clone 'https://github.com/YAGNADATTA25/Multi-Goal-Logistics-AMR-with-Hybrid-Predictive-DWA-and-Directional-Safety-Reflex-Loops..git' ~/warehouse_amr_ws
cd ~/warehouse_amr_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select autonomous_logistics_sim
source install/setup.bash
```

Ensure the required TurtleBot3 packages and Gazebo models are available. **Do not assume these commands reproduce the metrics until the ROS topic wiring and validated local version have been checked.**

**Terminal 1 — launch the warehouse simulator and Nav2:**

```bash
source /opt/ros/humble/setup.bash
source ~/warehouse_amr_ws/install/setup.bash
ros2 launch autonomous_logistics_sim warehouse_navigation.launch.py
```

**Terminal 2 — run the supervisor after Nav2 is active (only after verifying Nav2 outputs `/cmd_vel_nav` and a downstream consumer exists):**

```bash
source /opt/ros/humble/setup.bash
source ~/warehouse_amr_ws/install/setup.bash
ros2 run autonomous_logistics_sim simulation_master.py --ros-args -p use_sim_time:=true -p output_cmd_topic:=/cmd_vel_supervised
```

The master command shown is for a **Collision Monitor pipeline subscribing to `/cmd_vel_supervised`**. The included `warehouse_navigation.launch.py` is the original simulator/Nav2 launcher; it does **not** independently demonstrate that the supervisor or Collision Monitor command remapping is active. Follow the topics below before driving the robot. If your validated launch uses a different routing, retain that routing. Do not allow Nav2 and the supervisor to publish competing commands on `/cmd_vel`.

**Verify topic connections:**

```bash
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_supervised -v
ros2 topic info /cmd_vel -v
ros2 param get /controller_server use_sim_time
ros2 param get /amcl use_sim_time
```

## Publishing reproducible evidence

Include the following small files in the GitHub repository alongside the current source code:

```text
demo_results/
  analyze_run.py
  warehouse_run_01_summary.txt
```

For **independent verification**, also make the original `warehouse_run_01` recording (`metadata.yaml` and its `.db3` file), the exact working launch/parameter/controller files, and the simulation version or Git commit publicly accessible. Large ROS bag files are usually better attached to a GitHub Release or external dataset archive rather than committed directly to Git. Link that archive here once uploaded. A short Gazebo/RViz demo video provides visual context but cannot substitute for the recorded data. See [How the data were collected](#how-the-data-were-collected) for the recording commands and [Exactly how each metric was calculated](#exactly-how-each-metric-was-calculated) for the formulas and logging criteria.

## Current limitations and future work

- **Current research — SLAM:** Investigate simultaneous localisation and mapping for changing warehouse layouts; evaluate a SLAM localisation/mapping pipeline in place of the current known-map AMCL setup.
- **Obstacle trajectory tracking:** Improve persistent obstacle association and future-path estimates through temporary occlusions to reduce late reactions and unnecessary stops.
- **Fleet management:** Extend from one AMR to multiple robots with task allocation, traffic coordination, and shared-aisle scheduling.
- **Validation:** Run repeatable scenario comparisons (baseline Nav2 vs predictive supervision, varied obstacle speed and aisle width), measure actual collision/contact events, and profile controller deadline misses.

## Scope

This is a **research and educational Gazebo simulation**. Reported results should not be interpreted as proof of industrial safety certification, collision-free operation in all conditions, or real-robot performance.