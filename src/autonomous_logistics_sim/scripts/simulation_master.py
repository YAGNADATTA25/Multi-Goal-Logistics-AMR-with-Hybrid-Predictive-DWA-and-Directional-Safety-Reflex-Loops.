#!/usr/bin/env python3
"""
Simulation Master Robotics Controller.
Implements:
 - Asynchronous multi-waypoint loop tracking via Nav2
 - Dual-Layer Safety: 360-Degree Raw Laser Bumper + Local Costmap Window Filtering
 - Defensive Active Avoidance Maneuver (Pivoting away from dynamic threats)
 - Proactive 1.8s DWA Trajectory Future Projection Time Horizon
 - Tuned 1.00m localized radial sensing proximity filter
 - Mathematical 2.9m corridor pacing for 4 dynamic obstacles
 - Independent 0.9s Telemetry Clock (Guaranteed Continuous Feedback Loop)
 - Advanced Dynamic Error-Tracking Metrics (Cross-Track, Heading, and Goal Distance)
 - Event-Driven Global Plan Path Modification Alerts
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator
import threading
import time
import numpy as np

class SimulationMaster(Node):
    def __init__(self):
        super().__init__('simulation_master')
        
        # --- 1. INFRASTRUCTURE & PUBLISHERS ---
        self.pub_robot_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_obs1 = self.create_publisher(Twist, '/cmd_vel_obstacle_1', 10)
        self.pub_obs2 = self.create_publisher(Twist, '/cmd_vel_obstacle_2', 10)
        self.pub_obs3 = self.create_publisher(Twist, '/cmd_vel_obstacle_3', 10)
        self.pub_obs4 = self.create_publisher(Twist, '/cmd_vel_obstacle_4', 10)
        self.pub_init_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Intercepting Nav2 Commanded Actions & Global Path Vectors
        self.sub_nav_vel = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_vel_callback, 10)
        self.sub_global_plan = self.create_subscription(Path, '/plan', self.global_plan_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)
        
        # INTEGRATED: 360-Degree Raw Laser Scan Subscriber for Side/Rear Blindspot protection
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # --- 2. KINEMATIC CONFIGURATION & OPTIMIZED PERIMETER BUBBLE ---
        self.current_vel = Twist()       
        self.target_nav_vel = Twist()    
        self.max_speed = 0.26            
        self.max_accel = 0.90            
        self.robot_radius = 0.40         
        
        # --- TUNED SAFETY AND SCANNED OVERRIDE ZONE ---
        self.safety_margin = 0.09        
        self.detection_range = 1.00      
        self.mission_completed = False
        self.latest_global_path = None
        
        # INTEGRATED: 360 Emergency Reactive Registers
        self.emergency_laser_alert = False
        self.emergency_laser_bearing = 0.0
        
        # --- 3. FIXED 2.9M PACING SYSTEM CONFIGURATION ---
        self.obs_directions = np.array([1.0, 1.0, -1.0, -1.0])
        self.obs_timestamps = np.array([time.time()] * 4)
        
        # Lock math parameters permanently: 0.19 m/s * 14.50 s = EXACTLY 2.755 Meters travel
        self.TARGET_DURATION = 14.50
        self.FIXED_SPEED = 0.19
        self.obs_target_durations = np.array([self.TARGET_DURATION] * 4)
        self.obs_base_speeds = np.array([self.FIXED_SPEED] * 4)

        # Costmap registers
        self.current_obstacle_points = []
        self.previous_obstacle_points = []
        self.obstacle_velocities = np.array([0.0, 0.0])
        
        # --- 4. HIGH-PRECISION LIVE HEARTBEAT TELEMETRY REGISTERS ---
        self.is_handling_override = False
        self.mutex = threading.Lock()
        
        # Thread-safe pose state caches
        self.cached_x = 0.0
        self.cached_y = 0.0
        self.cached_yaw = 0.0
        self.active_waypoint_idx = 0
        
        # Incoming telemetry update cache pipeline
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cache_callback, 10)

        # --- 5. TIMER MANAGEMENT ---
        # 10Hz Orchestration Loop for kinematic control calculations
        self.control_timer = self.create_timer(0.1, self.orchestration_loop)
        # Dedicated 0.9-second telemetry clock to guarantee an unbroken terminal stream
        self.telemetry_timer = self.create_timer(0.9, self.publish_heartbeat_telemetry)
        
        # --- 6. NAV2 ENVIRONMENT MISSION ENGINE THREADING ---
        self.navigator = BasicNavigator()
        self.nav_thread = threading.Thread(target=self.run_robot_mission, daemon=True)
        self.nav_thread.start()
        
        self.get_logger().info('=== 4-AGENT CONTROLLER (HYBRID 360 SAFETY) INITIALIZED ===')
    
    def nav_vel_callback(self, msg):
        self.target_nav_vel = msg

    def global_plan_callback(self, msg):
        if msg.poses and len(msg.poses) > 0:
            self.latest_global_path = msg.poses
            self.get_logger().info("🔀 [PATH UPDATE] A change in the navigation path has been detected and registered!")

    def amcl_cache_callback(self, msg):
        with self.mutex:
            self.cached_x = msg.pose.pose.position.x
            self.cached_y = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.cached_yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))

    def scan_callback(self, msg):
        """
        INTEGRATED: Processes full 360-degree raw laser scans.
        Catches side and rear blindspot clipping bounds bypassed by the forward local costmap grid.
        """
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges) | (ranges <= 0.0)] = 99.0
        
        # Critical proximity envelope (0.38m means a side/rear bumper collision is imminent)
        critical_indices = np.where(ranges < 0.38)[0]
        
        with self.mutex:
            if len(critical_indices) > 0:
                self.emergency_laser_alert = True
                closest_idx = critical_indices[np.argmin(ranges[critical_indices])]
                self.emergency_laser_bearing = msg.angle_min + (closest_idx * msg.angle_increment)
            else:
                self.emergency_laser_alert = False

    def publish_heartbeat_telemetry(self):
        with self.mutex:
            current_x = self.cached_x
            current_y = self.cached_y
            current_yaw = self.cached_yaw
            path_snapshot = list(self.latest_global_path) if self.latest_global_path else None
            is_complete = self.mission_completed

        if is_complete:
            self.get_logger().info("🏁 [FEEDBACK SYSTEM] | Mission Complete! Final states stabilized at origin destination.")
            return

        if not path_snapshot or len(path_snapshot) == 0:
            self.get_logger().info("⏳ [FEEDBACK SYSTEM] | Boot Sequence: Awaiting Navigation Target Matrix Execution...")
            return

        target_goal_pose = path_snapshot[-1].pose.position
        distance_to_goal = np.hypot(target_goal_pose.x - current_x, target_goal_pose.y - current_y)

        distances_to_path = []
        for pose_stamped in path_snapshot[:50]:  
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            distances_to_path.append(np.hypot(px - current_x, py - current_y))
        
        closest_idx = np.argmin(distances_to_path)
        cross_track_error_cm = distances_to_path[closest_idx] * 100.0  

        if closest_idx < len(path_snapshot) - 1:
            next_pose = path_snapshot[closest_idx + 1].pose.position
            curr_pose = path_snapshot[closest_idx].pose.position
            path_heading_rad = np.arctan2(next_pose.y - curr_pose.y, next_pose.x - curr_pose.x)
            path_heading_deg = np.degrees(path_heading_rad)
            heading_error = path_heading_deg - current_yaw
            heading_error = (heading_error + 180) % 360 - 180  
        else:
            heading_error = 0.0

        if self.is_handling_override:
            status_flag = "⚠️  [AVOID_TRK]"
        else:
            status_flag = f"🎯 [WP_#{self.active_waypoint_idx}_TRK]"
            
        self.get_logger().info(
            f"{status_flag:<15} | "
            f"XT_Err: {cross_track_error_cm:5.1f} cm | "
            f"Head_Err: {heading_error:6.1f}° | "
            f"Goal_Dist: {distance_to_goal:5.2f} m | "
            f"V_Lin: {self.current_vel.linear.x:4.2f} m/s"
        )

    def costmap_callback(self, msg):
        grid_data = np.array(msg.data)
        lethal_indices = np.where(grid_data == 100)[0]
        
        width, height, resolution = msg.info.width, msg.info.height, msg.info.resolution
        robot_x_idx, robot_y_idx = width // 2, height // 2
        
        local_pts = []
        for index in lethal_indices:
            cell_x, cell_y = index % width, index // width
            dx = (cell_x - robot_x_idx) * resolution
            dy = (cell_y - robot_y_idx) * resolution
            
            cell_dist = np.hypot(dx, dy)
            if 0.31 <= cell_dist <= self.detection_range:
                local_pts.append([dx, dy])
        
        with self.mutex:
            self.previous_obstacle_points = self.current_obstacle_points
            self.current_obstacle_points = local_pts
            
            if len(self.previous_obstacle_points) > 0 and len(self.current_obstacle_points) > 0:
                prev_center = np.mean(self.previous_obstacle_points, axis=0)
                curr_center = np.mean(self.current_obstacle_points, axis=0)
                self.obstacle_velocities = (curr_center - prev_center) / 0.1
            else:
                self.obstacle_velocities = np.array([0.0, 0.0])

    def process_advanced_predictive_dwa(self):
        best_vel = Twist()
        best_vel.linear.x = min(self.target_nav_vel.linear.x, self.max_speed)
        best_vel.angular.z = self.target_nav_vel.angular.z
        min_cost = float('inf')
        
        with self.mutex:
            # INTEGRATED: PRIORITIZE 360 REAR/SIDE REFLEX OVER STANDARD CONTROLLER FOR IMMINENT CLIPS
            if self.emergency_laser_alert:
                if not self.is_handling_override:
                    self.is_handling_override = True
                    self.get_logger().warn("🚨 [360 BLINDSPOT INTRUSION] Rear or side threat caught on raw LaserScan bumper loop!")
                best_vel.linear.x = 0.0
                best_vel.angular.z = -1.20 if self.emergency_laser_bearing >= 0 else 1.20
                return best_vel

            threats = list(self.current_obstacle_points)
            obs_v = np.copy(self.obstacle_velocities)
            
        closest_threat_dist = float('inf')
        closest_threat_vector = np.array([0.0, 0.0])
        for obs in threats:
            d = np.hypot(obs[0], obs[1])
            if d < closest_threat_dist:
                closest_threat_dist = d
                closest_threat_vector = np.array([obs[0], obs[1]])
                
        critical_threshold = self.robot_radius + self.safety_margin
        obstacle_speed = np.hypot(obs_v[0], obs_v[1])

        # --- ANGULAR CONE FILTERING FOR CORRIDOR WALL EXCLUSION ---
        # Calculate direction of the closest threat in radians, then convert to degrees
        obstacle_bearing = np.arctan2(closest_threat_vector[1], closest_threat_vector[0])
        bearing_deg = np.degrees(obstacle_bearing)

        # Define explicit directional safety cones (Ignore strict left/right wall signatures)
        is_in_front_cone = (-45.0 <= bearing_deg <= 45.0)
        is_in_rear_cone  = (bearing_deg >= 135.0 or bearing_deg <= -135.0)

        # Execute Layer 2 reflex ONLY if threat breaches threshold AND sits inside the front/rear active cones
        if closest_threat_dist <= critical_threshold and obstacle_speed > 0.16 and (is_in_front_cone or is_in_rear_cone):
            if not self.is_handling_override:
                self.is_handling_override = True
                self.get_logger().warn(f"!!! [CRITICAL INTRUSION] Threat confirmed at {closest_threat_dist:.2f}m moving at {obstacle_speed:.2f}m/s")
            
            best_vel.linear.x = 0.0
            # Snappy turn adjustment (increased to 1.20 for faster wall-hugging escapes)
            best_vel.angular.z = -1.20 if obstacle_bearing >= 0 else 1.20
            return best_vel

        with self.mutex:
            if not self.emergency_laser_alert and self.is_handling_override and (closest_threat_dist > (critical_threshold + 0.15) or obstacle_speed <= 0.22):
                self.is_handling_override = False
        
        # --- PROACTIVE HORIZON SAMPLING ---
        v_min = max(-self.max_speed, self.current_vel.linear.x - self.max_accel * 0.1)
        v_max = min(self.max_speed, self.current_vel.linear.x + self.max_accel * 0.1)
        w_min = max(-1.5, self.current_vel.angular.z - 2.5 * 0.1)
        w_max = min(1.5, self.current_vel.angular.z + 2.5 * 0.1)
        
        v_samples = np.linspace(v_min, v_max, 15)
        w_samples = np.linspace(w_min, w_max, 15)
        
        t_horizon = 1.8  
        dt = 0.2
        steps = int(t_horizon / dt)
        

        for v in v_samples:
            for w in w_samples:
                sim_x, sim_y, sim_theta = 0.0, 0.0, 0.0
                step_closest_threat = float('inf')
                is_colliding = False
                
                for step in range(steps):
                    t = step * dt
                    sim_x += v * np.cos(sim_theta) * dt
                    sim_y += v * np.sin(sim_theta) * dt
                    sim_theta += w * dt
                    
                    for obs in threats:
                        proj_obs_x = obs[0] + (obs_v[0] * t)
                        proj_obs_y = obs[1] + (obs_v[1] * t)
                        
                        d = np.hypot(sim_x - proj_obs_x, sim_y - proj_obs_y)
                        if d < step_closest_threat:
                            step_closest_threat = d
                        if d <= (self.robot_radius + 0.04):
                            is_colliding = True
                            break
                    if is_colliding:
                        break
                
                if is_colliding:
                    continue  
                
                vel_cost = abs(self.target_nav_vel.linear.x - v)
                rot_cost = abs(self.target_nav_vel.angular.z - w)
                prox_cost = 1.0 / (step_closest_threat + 0.001)
                
                total_cost = (2.0 * vel_cost) + (0.8 * rot_cost) + (1.2 * prox_cost)
                
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_vel.linear.x = v
                    best_vel.angular.z = w
                    
        return best_vel

    def orchestration_loop(self):
        msg1, msg2, msg3, msg4 = Twist(), Twist(), Twist(), Twist()
        if self.mission_completed:
            self.pub_robot_vel.publish(Twist())
            self.pub_obs1.publish(msg1); self.pub_obs2.publish(msg2)
            self.pub_obs3.publish(msg3); self.pub_obs4.publish(msg4)
            return

        dt = 0.1
        now = time.time()
        
        for i in range(4):
            active_duration = self.TARGET_DURATION if (i == 0 or i == 2) else (self.TARGET_DURATION * 0.82)
            if now - self.obs_timestamps[i] >= active_duration:
                self.obs_directions[i] *= -1.0
                self.obs_timestamps[i] = now
                self.obs_target_durations[i] = active_duration
                self.obs_base_speeds[i] = self.FIXED_SPEED

        msg1.linear.y = self.obs_base_speeds[0] * self.obs_directions[0]
        msg3.linear.y = self.obs_base_speeds[2] * self.obs_directions[2]
        msg2.linear.x = self.obs_base_speeds[1] * self.obs_directions[1]
        msg4.linear.x = self.obs_base_speeds[3] * self.obs_directions[3]

        self.pub_obs1.publish(msg1); self.pub_obs2.publish(msg2)
        self.pub_obs3.publish(msg3); self.pub_obs4.publish(msg4)

        # --- ROBOT STEERING EXECUTION MATRIX ---
        safe_steer_vel = self.process_advanced_predictive_dwa()
        diff_v = safe_steer_vel.linear.x - self.current_vel.linear.x
        self.current_vel.linear.x += np.clip(diff_v, -self.max_accel * dt, self.max_accel * dt)
        self.current_vel.angular.z = safe_steer_vel.angular.z
        self.pub_robot_vel.publish(self.current_vel)

    def run_robot_mission(self):
        self.get_logger().info("Initializing AMCL Coordinate Tracking Reference Frames...")
        amcl_init = PoseWithCovarianceStamped()
        amcl_init.header.frame_id, amcl_init.header.stamp = 'map', self.navigator.get_clock().now().to_msg()
        amcl_init.pose.pose.position.x, amcl_init.pose.pose.position.y, amcl_init.pose.pose.orientation.w = 0.0, 0.0, 1.0
        
        covariance = np.zeros(36)
        covariance[0], covariance[7], covariance[35] = 0.1, 0.1, 0.03
        amcl_init.pose.covariance = covariance.tolist()

        for _ in range(5):
            self.pub_init_pose.publish(amcl_init)
            time.sleep(0.1)
            
        init_pose_stamped = PoseStamped()
        init_pose_stamped.header, init_pose_stamped.pose = amcl_init.header, amcl_init.pose.pose
        self.navigator.setInitialPose(init_pose_stamped)
        self.navigator.waitUntilNav2Active()

        mission_goals = [
            [3.34, 3.14], [3.19, -3.06], [-3.34, -2.96], [-3.41, 3.11], [-1.09, 1.08], [0.0, 0.0]
        ]
        
        goal_poses = []
        for pt in mission_goals:
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = 'map', self.navigator.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = pt[0], pt[1], 1.0
            goal_poses.append(pose)

        self.navigator.followWaypoints(goal_poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.active_waypoint_idx = feedback.current_waypoint + 1
            time.sleep(0.1)

        self.get_logger().info('=== SCHEDULING COMPLETE: ALL WAYPOINTS VISITED ===')
        with self.mutex:
            self.mission_completed = True

def main(args=None):
    rclpy.init(args=args)
    node = SimulationMaster()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub_robot_vel.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
