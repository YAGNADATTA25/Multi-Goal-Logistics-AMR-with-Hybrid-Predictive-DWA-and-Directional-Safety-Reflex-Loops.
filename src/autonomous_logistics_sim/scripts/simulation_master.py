#!/usr/bin/env python3
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener, TransformException


class SimulationMaster(Node):
    def __init__(self):
        super().__init__('simulation_master')

        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('linear_samples', 25)
        self.declare_parameter('angular_samples', 25)
        self.declare_parameter('prediction_horizon', 3.0)
        self.declare_parameter('prediction_step', 0.2)

        output_topic = str(self.get_parameter('output_cmd_topic').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.linear_samples = max(3, int(self.get_parameter('linear_samples').value))
        self.angular_samples = max(3, int(self.get_parameter('angular_samples').value))
        self.prediction_horizon = float(self.get_parameter('prediction_horizon').value)
        self.prediction_step = float(self.get_parameter('prediction_step').value)
        if self.control_frequency <= 0 or self.prediction_horizon <= 0 or self.prediction_step <= 0:
            raise ValueError('Control frequency and prediction settings must be greater than zero')

        self.control_group = MutuallyExclusiveCallbackGroup()
        self.scan_group = MutuallyExclusiveCallbackGroup()
        self.io_group = ReentrantCallbackGroup()
        self.telemetry_group = MutuallyExclusiveCallbackGroup()

        self.pub_robot_vel = self.create_publisher(Twist, output_topic, 10)
        self.pub_obs1 = self.create_publisher(Twist, '/cmd_vel_obstacle_1', 10)
        self.pub_obs2 = self.create_publisher(Twist, '/cmd_vel_obstacle_2', 10)
        self.pub_obs3 = self.create_publisher(Twist, '/cmd_vel_obstacle_3', 10)
        self.pub_obs4 = self.create_publisher(Twist, '/cmd_vel_obstacle_4', 10)
        self.pub_init_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.sub_nav_vel = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_vel_callback, 10, callback_group=self.io_group)
        self.sub_global_plan = self.create_subscription(Path, '/plan', self.global_plan_callback, 10, callback_group=self.io_group)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data, callback_group=self.io_group)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cache_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.io_group
        )
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data, callback_group=self.io_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mutex = threading.Lock()

        self.current_vel = Twist()
        self.target_nav_vel = Twist()
        self.max_speed = 0.26
        self.max_accel = 0.90
        self.max_decel = 0.90
        self.max_angular_accel = 2.5
        self.max_angular_speed = 1.5
        self.robot_radius = 0.15
        self.safety_margin = 0.09
        self.detection_range = 2.0
        self.emergency_distance = 0.38
        self.max_scan_points = 180
        self.scan_timeout = 0.65
        self.command_timeout = 0.80
        self.tf_retry_timeout = 0.20

        self.scan_points = np.empty((0, 2), dtype=float)
        self.scan_velocities = np.empty((0, 2), dtype=float)
        self.scan_time = None
        self.last_nav_time = None
        self.robot_pose = np.zeros(3, dtype=float)
        self.odom_pose = np.zeros(3, dtype=float)
        self.odom_time = None
        self.odom_speed = 0.0
        self.odom_distance = 0.0
        self.last_odom_xy = None
        self.closest_laser_dist = float('inf')
        self.tracks = {}
        self.next_track_id = 1
        self.pending_scan = None
        self.pending_scan_received = None
        self.scans_received = 0
        self.scans_processed = 0
        self.last_scan_stamp = None
        self.last_tf_error = ''
        self.processing_scan = False
        self.last_tf_warning = 0.0
        self.last_input_warning = 0.0
        self.no_safe_path = False
        self.is_handling_override = False
        self.control_state = 'STARTING'
        self.latest_dwa_ms = 0.0
        self.last_control_at = time.monotonic()
        self.plan_updates = 0

        self.mission_completed = False
        self.mission_succeeded = False
        self.latest_global_path = None
        self.cached_x = 0.0
        self.cached_y = 0.0
        self.cached_yaw = 0.0
        self.pose_time = None
        self.active_waypoint_idx = 0

        self.obs_directions = np.array([1.0, 1.0, -1.0, -1.0])
        self.obs_timestamps = np.array([time.monotonic()] * 4)
        self.TARGET_DURATION = 14.50
        self.FIXED_SPEED = 0.20
        self.obs_target_durations = np.array([self.TARGET_DURATION] * 4)
        self.obs_base_speeds = np.array([self.FIXED_SPEED] * 4)

        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.orchestration_loop, callback_group=self.control_group)
        self.scan_retry_timer = self.create_timer(0.01, self.process_pending_scan, callback_group=self.scan_group)
        self.telemetry_timer = self.create_timer(0.9, self.publish_heartbeat_telemetry, callback_group=self.telemetry_group)

        self.navigator = BasicNavigator()
        self.navigator.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        self.nav_thread = threading.Thread(target=self.run_robot_mission, daemon=True)
        self.nav_thread.start()
        self.get_logger().info(f'Simulation master started; output={output_topic}')

    def nav_vel_callback(self, msg):
        with self.mutex:
            self.target_nav_vel = msg
            self.last_nav_time = time.monotonic()

    def global_plan_callback(self, msg):
        if msg.poses:
            with self.mutex:
                self.latest_global_path = list(msg.poses)
                self.plan_updates += 1

    def amcl_cache_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.mutex:
            self.cached_x = msg.pose.pose.position.x
            self.cached_y = msg.pose.pose.position.y
            self.cached_yaw = float(np.degrees(yaw))
            self.pose_time = time.monotonic()

    def odom_callback(self, msg):
        if msg.header.frame_id.lstrip('/') != 'odom':
            return
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.mutex:
            self.odom_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw], dtype=float)
            self.odom_time = time.monotonic()
            self.odom_speed = float(msg.twist.twist.linear.x)
            xy = self.odom_pose[:2].copy()
            if self.last_odom_xy is not None:
                increment = float(np.linalg.norm(xy - self.last_odom_xy))
                if 0.001 < increment < 0.25:
                    self.odom_distance += increment
            self.last_odom_xy = xy

    def read_tf_pose(self, target, source, stamp):
        transform = self.tf_buffer.lookup_transform(target, source, stamp)
        q = transform.transform.rotation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = transform.transform.translation
        return np.array([p.x, p.y, yaw], dtype=float)

    def find_scan_groups(self, points, valid):
        indices = np.flatnonzero(valid)
        if not len(indices):
            return []
        groups = []
        group = [int(indices[0])]
        for index in indices[1:]:
            index = int(index)
            previous = group[-1]
            if index == previous + 1 and np.linalg.norm(points[index] - points[previous]) < 0.18:
                group.append(index)
            else:
                groups.append(np.array(group, dtype=int))
                group = [index]
        groups.append(np.array(group, dtype=int))
        if len(groups) > 1 and groups[0][0] == 0 and groups[-1][-1] == len(valid) - 1:
            if np.linalg.norm(points[groups[0][0]] - points[groups[-1][-1]]) < 0.18:
                groups[0] = np.concatenate((groups[-1], groups[0]))
                groups.pop()
        return groups

    def update_obstacle_tracks(self, detections, now):
        previous = {key: value for key, value in self.tracks.items() if now - value['time'] < 0.6}
        matches_available = []
        for key, track in previous.items():
            dt = max(0.0, now - track['time'])
            predicted = track['position'] + track['velocity'] * dt
            for index, detection in enumerate(detections):
                distance = np.linalg.norm(detection['position'] - predicted)
                if distance <= 0.25 + 0.6 * dt:
                    matches_available.append((float(distance), key, index))

        chosen_tracks = set()
        chosen_detections = set()
        matches = {}
        for distance, key, index in sorted(matches_available):
            if key not in chosen_tracks and index not in chosen_detections:
                matches[index] = key
                chosen_tracks.add(key)
                chosen_detections.add(index)

        updated = dict(previous)
        for index, detection in enumerate(detections):
            position = detection['position']
            if index in matches:
                key = matches[index]
                old = previous[key]
                dt = now - old['time']
                velocity = old['velocity'].copy()
                if dt > 0.03:
                    measured = (position - old['position']) / dt
                    speed = np.linalg.norm(measured)
                    if speed > 1.2:
                        measured *= 1.2 / speed
                    velocity = 0.45 * velocity + 0.55 * measured
                seen = old['seen'] + 1
            else:
                key = self.next_track_id
                self.next_track_id += 1
                velocity = np.zeros(2, dtype=float)
                seen = 1
            if seen >= 3 and np.linalg.norm(velocity) < 0.055:
                velocity[:] = 0.0
            updated[key] = {'position': position, 'velocity': velocity, 'seen': seen, 'time': now}
            detection['velocity'] = velocity if seen >= 3 else np.zeros(2, dtype=float)

        self.tracks = updated
        return detections

    def scan_callback(self, msg):
        with self.mutex:
            self.pending_scan = msg
            self.pending_scan_received = time.monotonic()
            self.scans_received += 1
            self.last_scan_stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def process_pending_scan(self):
        with self.mutex:
            if self.processing_scan or self.pending_scan is None:
                return
            msg = self.pending_scan
            received = self.pending_scan_received
            self.processing_scan = True
        try:
            now = time.monotonic()
            if received is None or now - received > self.tf_retry_timeout:
                with self.mutex:
                    if self.pending_scan is msg:
                        self.pending_scan = None
                if now - self.last_tf_warning > 3.0:
                    with self.mutex:
                        error = self.last_tf_error
                    self.get_logger().warn(f'LiDAR scan expired waiting for TF: {error or "transform not available"}')
                    self.last_tf_warning = now
                return

            stamp_seconds = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            clock_seconds = self.get_clock().now().nanoseconds * 1e-9
            if clock_seconds > 0 and stamp_seconds > 0 and abs(clock_seconds - stamp_seconds) > 2.0:
                with self.mutex:
                    self.last_tf_error = (f'/scan={stamp_seconds:.3f}s, node_clock={clock_seconds:.3f}s; '
                                          f'check use_sim_time and duplicate /clock publishers')
                return
            stamp = Time.from_msg(msg.header.stamp)
            try:
                laser_pose = self.read_tf_pose('odom', msg.header.frame_id, stamp)
                robot_pose = self.read_tf_pose('odom', 'base_link', stamp)
            except TransformException as exc:
                with self.mutex:
                    self.last_tf_error = f'{msg.header.frame_id} -> odom at {stamp_seconds:.3f}s: {exc}'
                return

            with self.mutex:
                if self.pending_scan is not msg:
                    return
                self.pending_scan = None

            ranges = np.asarray(msg.ranges, dtype=float)
            if len(ranges) == 0:
                return
            max_range = min(float(msg.range_max), self.detection_range)
            valid = np.isfinite(ranges) & (ranges >= max(float(msg.range_min), 0.01)) & (ranges <= max_range)
            angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
            points = np.full((len(ranges), 2), np.nan, dtype=float)
            points[valid, 0] = ranges[valid] * np.cos(angles[valid])
            points[valid, 1] = ranges[valid] * np.sin(angles[valid])
            c, s = np.cos(laser_pose[2]), np.sin(laser_pose[2])
            rotation = np.array([[c, -s], [s, c]])
            points[valid] = points[valid] @ rotation.T + laser_pose[:2]

            groups = self.find_scan_groups(points, valid)
            detections = []
            for group in groups:
                if len(group) < 3:
                    continue
                segment = points[group]
                if np.linalg.norm(np.ptp(segment, axis=0)) <= 0.65:
                    detections.append({'position': np.mean(segment, axis=0), 'indices': group})

            now = time.monotonic()
            with self.mutex:
                detections = self.update_obstacle_tracks(detections, now)
                velocities = np.zeros_like(points)
                for detection in detections:
                    velocities[detection['indices']] = detection['velocity']
                valid_indices = np.flatnonzero(valid)
                if len(valid_indices) > self.max_scan_points:
                    chosen = []
                    for block in np.array_split(valid_indices, self.max_scan_points):
                        chosen.append(int(block[np.argmin(ranges[block])]))
                    valid_indices = np.array(chosen, dtype=int)
                self.scan_points = points[valid_indices].copy()
                self.scan_velocities = velocities[valid_indices].copy()
                self.scan_time = now
                self.robot_pose = robot_pose
                self.scans_processed += 1
                self.last_tf_error = ''
                self.closest_laser_dist = float(np.min(ranges[valid])) if np.any(valid) else float('inf')
        finally:
            with self.mutex:
                self.processing_scan = False

    def moving_threats(self, pose, current_v, now):
        robot_velocity = current_v * np.array([np.cos(pose[2]), np.sin(pose[2])])
        threats = []
        for track in self.tracks.values():
            if track['seen'] < 3 or now - track['time'] > self.scan_timeout:
                continue
            speed = np.linalg.norm(track['velocity'])
            if speed < 0.07:
                continue
            dt = max(0.0, now - track['time'])
            position = track['position'] + track['velocity'] * dt
            relative = position - pose[:2]
            relative_velocity = track['velocity'] - robot_velocity
            squared_speed = float(np.dot(relative_velocity, relative_velocity))
            if squared_speed < 1e-5:
                continue
            closest_time = np.clip(-np.dot(relative, relative_velocity) / squared_speed,
                                   0.0, self.prediction_horizon)
            closest_distance = np.linalg.norm(relative + relative_velocity * closest_time)
            if np.dot(relative, relative_velocity) < -0.02 and closest_distance < 0.85:
                threats.append((position, track['velocity'].copy()))
        return threats

    def warn_input(self, message):
        now = time.monotonic()
        if now - self.last_input_warning > 2.0:
            self.get_logger().warn(message)
            self.last_input_warning = now

    def process_advanced_predictive_dwa(self):
        start = time.perf_counter()
        now = time.monotonic()
        with self.mutex:
            scan_time = self.scan_time
            nav_time = self.last_nav_time
            points = self.scan_points.copy()
            velocities = self.scan_velocities.copy()
            scan_pose = self.robot_pose.copy()
            odom_pose = self.odom_pose.copy()
            odom_time = self.odom_time
            nav_v = float(self.target_nav_vel.linear.x)
            nav_w = float(self.target_nav_vel.angular.z)
            current_v = float(self.current_vel.linear.x)
            current_w = float(self.current_vel.angular.z)

            pose = odom_pose if odom_time is not None and now - odom_time < 0.4 else scan_pose
            risk_tracks = self.moving_threats(pose, current_v, now)

        self.no_safe_path = False
        self.is_handling_override = False
        if scan_time is None or now - scan_time > self.scan_timeout:
            self.no_safe_path = True
            self.control_state = 'WAIT_LASER'
            with self.mutex:
                rx = self.scans_received
                ok = self.scans_processed
                tf_error = self.last_tf_error
            if rx == 0:
                self.warn_input('No /scan messages received; check LiDAR publisher and QoS')
            else:
                self.warn_input(f'LiDAR received ({rx}) but valid TF scans ({ok}) are stale: '
                                f'{tf_error or "check /scan and odom/base_scan timestamps"}')
            return Twist()
        if nav_time is None or now - nav_time > self.command_timeout:
            self.no_safe_path = True
            self.control_state = 'WAIT_NAV'
            self.warn_input('No recent /cmd_vel_nav; check Nav2 topic and command routing')
            return Twist()

        age = max(0.0, now - scan_time)
        points = points + velocities * age
        closest_distance = float(np.min(np.linalg.norm(points - pose[:2], axis=1))) if len(points) else float('inf')
        close_alert = closest_distance < self.emergency_distance
        self.is_handling_override = bool(risk_tracks) or close_alert

        if closest_distance <= self.robot_radius + 0.02:
            self.no_safe_path = True
            self.control_state = 'STOP_CLOSE'
            self.warn_input('Object inside robot clearance; robot stopped')
            return Twist()

        target_v = float(np.clip(nav_v, -self.max_speed, self.max_speed))
        target_w = float(np.clip(nav_w, -self.max_angular_speed, self.max_angular_speed))
        clear_for_passthrough = max(0.75, abs(target_v) * self.prediction_horizon + self.robot_radius + self.safety_margin)
        if not risk_tracks and closest_distance > clear_for_passthrough:
            self.control_state = 'FOLLOW'
            command = Twist()
            command.linear.x = target_v
            command.angular.z = target_w
            self.latest_dwa_ms = (time.perf_counter() - start) * 1000.0
            return command

        linear_values = np.linspace(-self.max_speed, self.max_speed, self.linear_samples)
        angular_values = np.linspace(-self.max_angular_speed, self.max_angular_speed, self.angular_samples)
        linear = np.concatenate((np.repeat(linear_values, len(angular_values)), [target_v, 0.0]))
        angular = np.concatenate((np.tile(angular_values, len(linear_values)), [target_w, 0.0]))
        x = np.full(linear.shape, pose[0], dtype=float)
        y = np.full(linear.shape, pose[1], dtype=float)
        heading = np.full(linear.shape, pose[2], dtype=float)
        simulated_v = np.full(linear.shape, current_v, dtype=float)
        simulated_w = np.full(linear.shape, current_w, dtype=float)
        safe = np.ones(linear.shape, dtype=bool)
        min_distance = np.full(linear.shape, float('inf'), dtype=float)

        steps = max(1, int(np.ceil(self.prediction_horizon / self.prediction_step)))
        for step in range(1, steps + 1):
            dt = min(self.prediction_step, self.prediction_horizon - (step - 1) * self.prediction_step)
            simulated_v += np.clip(linear - simulated_v, -self.max_decel * dt, self.max_accel * dt)
            simulated_w += np.clip(angular - simulated_w,
                                   -self.max_angular_accel * dt, self.max_angular_accel * dt)
            midpoint = heading + 0.5 * simulated_w * dt
            x += simulated_v * np.cos(midpoint) * dt
            y += simulated_v * np.sin(midpoint) * dt
            heading += simulated_w * dt
            clearance = (self.robot_radius + self.safety_margin + 0.15 * np.abs(simulated_v)
                         + simulated_v * simulated_v / (2.0 * self.max_decel))
            if len(points):
                future_points = points + velocities * min(step * self.prediction_step, self.prediction_horizon)
                dx = x[:, None] - future_points[None, :, 0]
                dy = y[:, None] - future_points[None, :, 1]
                distance = np.sqrt(np.min(dx * dx + dy * dy, axis=1))
                min_distance = np.minimum(min_distance, distance)
                safe &= distance > clearance
                if not np.any(safe):
                    break

        if not np.any(safe):
            self.no_safe_path = True
            self.control_state = 'STOP_NO_PATH'
            self.warn_input('DWA found no safe movement; robot stopped')
            self.latest_dwa_ms = (time.perf_counter() - start) * 1000.0
            return Twist()

        cost = 4.0 * np.abs(linear - target_v) + 1.5 * np.abs(angular - target_w)
        if len(points):
            comfort_gap = np.maximum(0.0, 0.48 - min_distance)
            cost += 5.0 * comfort_gap * comfort_gap
        if risk_tracks:
            for position, velocity in risk_tracks:
                final_position = position + velocity * self.prediction_horizon
                final_distance = np.hypot(x - final_position[0], y - final_position[1])
                cost -= 0.3 * np.minimum(final_distance, 1.0)

        cost[~safe] = np.inf
        best = int(np.argmin(cost))
        result = Twist()
        result.linear.x = float(linear[best])
        result.angular.z = float(angular[best])
        self.control_state = 'AVOID' if self.is_handling_override else 'FOLLOW'
        self.latest_dwa_ms = (time.perf_counter() - start) * 1000.0
        return result

    @staticmethod
    def path_error(poses, x, y):
        if len(poses) < 2:
            return float('nan'), 0.0
        coordinates = np.array([[p.pose.position.x, p.pose.position.y] for p in poses], dtype=float)
        start = coordinates[:-1]
        segment = coordinates[1:] - start
        lengths_sq = np.einsum('ij,ij->i', segment, segment)
        valid = lengths_sq > 1e-8
        if not np.any(valid):
            return float(np.linalg.norm(coordinates[0] - np.array([x, y]))), 0.0
        start = start[valid]
        segment = segment[valid]
        lengths_sq = lengths_sq[valid]
        relative = np.array([x, y]) - start
        progress = np.clip(np.einsum('ij,ij->i', relative, segment) / lengths_sq, 0.0, 1.0)
        nearest = start + segment * progress[:, None]
        distances = np.linalg.norm(nearest - np.array([x, y]), axis=1)
        index = int(np.argmin(distances))
        heading = float(np.degrees(np.arctan2(segment[index, 1], segment[index, 0])))
        return float(distances[index]), heading

    def publish_heartbeat_telemetry(self):
        with self.mutex:
            current_x = self.cached_x
            current_y = self.cached_y
            current_yaw = self.cached_yaw
            pose_age = None if self.pose_time is None else time.monotonic() - self.pose_time
            path_snapshot = list(self.latest_global_path) if self.latest_global_path else []
            completed = self.mission_completed
            succeeded = self.mission_succeeded
            active_waypoint = self.active_waypoint_idx
            nav_v = self.target_nav_vel.linear.x
            output_v = self.current_vel.linear.x
            odom_speed = self.odom_speed
            state = self.control_state
            last_scan = self.scan_time
            updates = self.plan_updates
            scans_rx = self.scans_received
            scans_ok = self.scans_processed
            tf_error = self.last_tf_error
        if completed:
            return
        if not path_snapshot:
            tf_detail = f' | TF={tf_error[:115]}' if tf_error else ''
            self.get_logger().info(f'[{state}] Waiting for Nav2 path | Cmd={output_v:.2f} m/s | '
                                   f'ScanRx={scans_rx} ScanOK={scans_ok}{tf_detail}')
            return
        if pose_age is None or pose_age > 2.0:
            try:
                map_pose = self.read_tf_pose('map', 'base_link', Time())
                current_x = float(map_pose[0])
                current_y = float(map_pose[1])
                current_yaw = float(np.degrees(map_pose[2]))
            except TransformException:
                self.get_logger().warn('No recent AMCL pose or map-to-base TF for feedback')
                return
        goal = path_snapshot[-1].pose.position
        goal_distance = float(np.hypot(goal.x - current_x, goal.y - current_y))
        error, path_heading = self.path_error(path_snapshot, current_x, current_y)
        heading_error = (path_heading - current_yaw + 180.0) % 360.0 - 180.0
        scan_age = float('inf') if last_scan is None else time.monotonic() - last_scan
        self.get_logger().info(
            f'[{state} WP_{active_waypoint}] XT={error * 100.0:.1f} cm | '
            f'Heading={heading_error:.1f} deg | Goal={goal_distance:.2f} m | '
            f'Cmd={output_v:.2f} | Odom={odom_speed:.2f} | Nav={nav_v:.2f} m/s | '
            f'ScanAge={scan_age:.2f} s | DWA={self.latest_dwa_ms:.1f} ms | Plan={updates}'
        )

    def orchestration_loop(self):
        if not rclpy.ok():
            return
        now = time.monotonic()
        dt = float(np.clip(now - self.last_control_at, 0.01, 0.20))
        self.last_control_at = now
        obstacle_msgs = [Twist() for _ in range(4)]
        with self.mutex:
            completed = self.mission_completed
        if completed:
            self.current_vel = Twist()
            self.pub_robot_vel.publish(Twist())
            for pub, msg in zip((self.pub_obs1, self.pub_obs2, self.pub_obs3, self.pub_obs4), obstacle_msgs):
                pub.publish(msg)
            return

        for index in range(4):
            duration = self.TARGET_DURATION if index in (0, 2) else self.TARGET_DURATION * 0.82
            if now - self.obs_timestamps[index] >= duration:
                self.obs_directions[index] *= -1.0
                self.obs_timestamps[index] = now

        obstacle_msgs[0].linear.y = float(self.FIXED_SPEED * self.obs_directions[0])
        obstacle_msgs[2].linear.y = float(self.FIXED_SPEED * self.obs_directions[2])
        obstacle_msgs[1].linear.x = float(self.FIXED_SPEED * self.obs_directions[1])
        obstacle_msgs[3].linear.x = float(self.FIXED_SPEED * self.obs_directions[3])
        for pub, msg in zip((self.pub_obs1, self.pub_obs2, self.pub_obs3, self.pub_obs4), obstacle_msgs):
            pub.publish(msg)

        command = self.process_advanced_predictive_dwa()
        if self.no_safe_path:
            self.current_vel = Twist()
        else:
            dv = float(command.linear.x - self.current_vel.linear.x)
            dw = float(command.angular.z - self.current_vel.angular.z)
            self.current_vel.linear.x += float(np.clip(dv, -self.max_decel * dt, self.max_accel * dt))
            self.current_vel.angular.z += float(np.clip(dw,
                                                        -self.max_angular_accel * dt,
                                                        self.max_angular_accel * dt))
        self.pub_robot_vel.publish(self.current_vel)

    def run_robot_mission(self):
        try:
            if not rclpy.ok():
                return
            self.get_logger().info('Setting initial pose')
            start = time.monotonic()
            while rclpy.ok() and self.get_clock().now().nanoseconds == 0 and time.monotonic() - start < 15.0:
                time.sleep(0.1)
            amcl_init = PoseWithCovarianceStamped()
            amcl_init.header.frame_id = 'map'
            amcl_init.header.stamp = self.get_clock().now().to_msg()
            amcl_init.pose.pose.position.x = 0.0
            amcl_init.pose.pose.position.y = 0.0
            amcl_init.pose.pose.orientation.w = 1.0
            covariance = np.zeros(36)
            covariance[0], covariance[7], covariance[35] = 0.1, 0.1, 0.03
            amcl_init.pose.covariance = covariance.tolist()
            for _ in range(5):
                if not rclpy.ok():
                    return
                self.pub_init_pose.publish(amcl_init)
                time.sleep(0.1)
            initial = PoseStamped()
            initial.header = amcl_init.header
            initial.pose = amcl_init.pose.pose
            self.navigator.setInitialPose(initial)
            self.navigator.waitUntilNav2Active()

            next_notice = time.monotonic() + 5.0
            while rclpy.ok():
                with self.mutex:
                    ready = self.scan_time is not None and time.monotonic() - self.scan_time < self.scan_timeout
                if ready:
                    break
                if time.monotonic() >= next_notice:
                    self.get_logger().warn('Mission waiting for first valid LaserScan with odom TF')
                    next_notice = time.monotonic() + 5.0
                time.sleep(0.1)
            if not rclpy.ok():
                return

            mission_goals = [
                [3.34, 3.14], [3.19, -3.06], [-3.34, -2.96],
                [-3.41, 3.11], [-1.09, 1.08], [0.0, 0.0]
            ]
            goal_poses = []
            for x, y in mission_goals:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                goal_poses.append(pose)

            self.navigator.followWaypoints(goal_poses)
            while rclpy.ok() and not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback is not None:
                    with self.mutex:
                        self.active_waypoint_idx = feedback.current_waypoint + 1
                time.sleep(0.1)

            if not rclpy.ok():
                return
            task_result = self.navigator.getResult()
            result_future = getattr(self.navigator, 'result_future', None)
            response = result_future.result() if result_future is not None and result_future.done() else None
            action_result = getattr(response, 'result', None)
            missed = list(action_result.missed_waypoints) if hasattr(action_result, 'missed_waypoints') else None
            with self.mutex:
                travelled = float(self.odom_distance)
            succeeded = (task_result == TaskResult.SUCCEEDED
                         and missed is not None and len(missed) == 0
                         and travelled >= 10.0)
            with self.mutex:
                self.mission_succeeded = succeeded
                self.mission_completed = True
            if succeeded:
                self.get_logger().info(f'Six-waypoint action completed; odometry travel={travelled:.2f} m')
            else:
                self.get_logger().error(
                    f'Waypoint completion NOT verified | result={task_result} | '
                    f'missed={missed} | odometry travel={travelled:.2f} m'
                )
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().error(f'Waypoint mission error: {exc}')
            with self.mutex:
                self.mission_completed = True
                self.mission_succeeded = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = SimulationMaster()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                try:
                    node.pub_robot_vel.publish(Twist())
                except Exception:
                    pass
            if executor is not None:
                executor.shutdown()
            node.destroy_node()
            node.navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()