# Mixed pedestrian and cart scene

This optional Gazebo Classic world replaces the four moving cylinders with two person meshes and two loaded four-wheel carts. The original warehouse.world remains the default for the existing launch files.

## Run

From your existing ROS 2 Humble workspace with the project's TurtleBot3/Gazebo dependencies installed:

```bash
colcon build --symlink-install --packages-select autonomous_logistics_sim
source install/setup.bash
ros2 launch autonomous_logistics_sim warehouse_mixed.launch.py
```

In another terminal, source the same ROS/workspace environment, then start the existing motion and mission controller:

```bash
ros2 run autonomous_logistics_sim simulation_master.py --ros-args -p use_sim_time:=true
```

The mixed-world launcher starts Gazebo, the robot, Nav2 and RViz. Obstacle movement starts when simulation_master publishes commands. Its existing six-goal mission also starts; launch the master once.

Both original launch files now accept an absolute world-file path through their world argument. They register the installed package models directory in GAZEBO_MODEL_PATH, preserving any existing search paths.

## Obstacles and nominal routes

| ID | Model | Command topic | Starting XY | Direction |
|---|---|---|---|---|
| 1 | warehouse_person_1 | /cmd_vel_obstacle_1 | (2.3, -1.3775) | +Y, then reverse |
| 2 | warehouse_cart_2 | /cmd_vel_obstacle_2 | (-1.12955, -0.9) | +X, then reverse |
| 3 | warehouse_person_3 | /cmd_vel_obstacle_3 | (-2.3, 1.3775) | -Y, then reverse |
| 4 | warehouse_cart_4 | /cmd_vel_obstacle_4 | (1.12955, 0.9) | -X, then reverse |

The starting points center the existing controller's nominal legs: 2.755 m for people and 2.2591 m for carts. They are positioned within the central warehouse area rather than through its corner walls. The existing controller uses wall-clock timers, so these are ideal-motion estimates, not closed-loop endpoint guarantees. Simulation slowdowns, contact, and delayed commands can change actual travel.

Each person uses the existing OSRF person_walking mesh in a fixed walking pose, carried by a velocity-controlled model with a simplified cylindrical collision volume. There is no skeletal footstep animation. Carts are adapted from the OSRF cart_rigid_suspension model; payload boxes provide visible and collision geometry at laser height. Model attribution is in ../models/THIRD_PARTY.md.

The world retains the original walls and four static pillars, so the saved static map is retained. New moving models are intended to be observed through live sensor/costmap updates.

## Required simulation checks

This change has been reviewed structurally, but has not been launched in Gazebo in the authoring environment.

1. Confirm all four models render, person textures resolve, and carts stay upright.
2. Start the master once; confirm each model moves on its documented topic and reverses.
3. Inspect /scan and the RViz local costmap as each person/cart passes the robot. Confirm detection with the actual laser plugin in use.
4. Verify the nominal routes are clear of walls and static pillars for several cycles at the computer's achieved simulation rate.
5. Verify collision/contact behavior before treating absence of visual overlap as a successful avoidance test.
6. Inspect `ros2 topic info /cmd_vel --verbose`. The existing master/Nav2 command-routing issue must be resolved before using this scene to make claims about custom avoidance performance.
7. Confirm the original warehouse_navigation.launch.py still opens the original cylinder world.

## Navigation work is separate

This asset integration does not change goal tolerances, global replanning, the shared-centroid obstacle-velocity estimate, or command arbitration. The controller currently accepts a 0.25 m goal tolerance in warehouse_params.yaml. A tighter tolerance is only a requested acceptance threshold; verified arrival accuracy requires localization/ground-truth measurements, stopping checks, and repeated trials.

For dynamic avoidance experiments, track each obstacle separately with timestamped, frame-consistent measurements. Keep Gazebo ground truth for evaluation, or explicitly label any controller that uses it as a simulation-only mode.
