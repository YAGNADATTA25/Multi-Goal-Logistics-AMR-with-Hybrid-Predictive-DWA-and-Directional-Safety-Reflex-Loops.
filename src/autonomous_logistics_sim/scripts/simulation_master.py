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

        self.declare_parameter('output_cmd_topic', '/cmd_vel_supervised')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('linear_samples', 25)
        self.declare_parameter('angular_samples', 25)
        self.declare_parameter('prediction_horizon', 3.0)
        self.declare_parameter('prediction_step', 0.2)
        self.declare_parameter('goal_retry_limit', 3)

        output_topic = str(self.get_parameter('output_cmd_topic').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.linear_samples = max(3, int(self.get_parameter('linear_samples').value))
        self.angular_samples = max(3, int(self.get_parameter('angular_samples').value))
        self.horizon = float(self.get_parameter('prediction_horizon').value)
        self.step = float(self.get_parameter('prediction_step').value)
        self.retry_limit = max(1, int(self.get_parameter('goal_retry_limit').value))

        if min(self.control_frequency, self.horizon, self.step) <= 0.0:
            raise ValueError('Control frequency and prediction periods must be positive')

        self.control_group = MutuallyExclusiveCallbackGroup()
        self.scan_group = MutuallyExclusiveCallbackGroup()
        self.io_group = ReentrantCallbackGroup()
        self.telemetry_group = MutuallyExclusiveCallbackGroup()
        self.lock = threading.Lock()

        self.pub_robot_vel = self.create_publisher(Twist, output_topic, 10)
        self.pub_obs = [
            self.create_publisher(Twist, f'/cmd_vel_obstacle_{i}', 10)
            for i in range(1, 5)
        ]
        self.pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav_callback,
            10,
            callback_group=self.io_group
        )

        self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10,
            callback_group=self.io_group
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=self.io_group
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data,
            callback_group=self.io_group
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
            callback_group=self.io_group
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = BasicNavigator()
        self.navigator.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.max_speed = 0.26
        self.max_accel = 0.90
        self.max_decel = 0.90
        self.max_turn = 1.5
        self.max_turn_accel = 2.5

        self.radius = 0.15
        self.margin = 0.09
        self.scan_timeout = 0.65
        self.nav_timeout = 0.80
        self.tf_retry_timeout = 0.25
        self.emergency_radius = 0.175
        self.detection_range = 2.0
        self.max_scan_points = 180

        self.target_nav = Twist()
        self.current_cmd = Twist()
        self.last_nav_time = None
        self.last_scan_time = None
        self.odom_time = None
        self.pose_time = None

        self.odom_pose = np.zeros(3, dtype=float)
        self.scan_pose = np.zeros(3, dtype=float)
        self.map_pose = np.zeros(3, dtype=float)

        self.odom_speed = 0.0
        self.distance_travelled = 0.0
        self.previous_odom_xy = None

        self.points = np.empty((0, 2), dtype=float)
        self.velocities = np.empty((0, 2), dtype=float)

        self.tracks = {}
        self.next_track_id = 0

        self.pending_scan = None
        self.pending_at = None
        self.scan_rx = 0
        self.scan_ok = 0
        self.last_tf_error = ''

        self.plan = []
        self.plan_updates = 0
        self.waypoint = 0
        self.mission_finished = False
        self.mission_success = False
        self.recovery_attempt = 0
        self.control_state = 'STARTING'

        self.last_warning = 0.0
        self.last_control = time.monotonic()
        self.dwa_ms = 0.0

        self.obstacle_directions = np.array([1.0, 1.0, -1.0, -1.0])
        self.obstacle_times = np.full(4, time.monotonic())

        self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop,
            callback_group=self.control_group
        )

        self.create_timer(
            0.01,
            self.process_pending_scan,
            callback_group=self.scan_group
        )

        self.create_timer(
            0.9,
            self.telemetry,
            callback_group=self.telemetry_group
        )

        self.mission_thread = threading.Thread(
            target=self.run_mission,
            daemon=True
        )
        self.mission_thread.start()

        self.get_logger().info(
            f'Simulation Master ready | command output={output_topic}'
        )

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warning > 2.5:
            self.get_logger().warn(message)
            self.last_warning = now

    def nav_callback(self, msg):
        with self.lock:
            self.target_nav = msg
            self.last_nav_time = time.monotonic()

    def plan_callback(self, msg):
        if msg.poses:
            with self.lock:
                self.plan = list(msg.poses)
                self.plan_updates += 1

    @staticmethod
    def quaternion_yaw(q):
        return float(
            np.arctan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )
        )

    def amcl_callback(self, msg):
        with self.lock:
            self.map_pose = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                self.quaternion_yaw(msg.pose.pose.orientation)
            ], dtype=float)

            self.pose_time = time.monotonic()

    def odom_callback(self, msg):
        if msg.header.frame_id.lstrip('/') != 'odom':
            return

        pose = msg.pose.pose
        xy = np.array([
            pose.position.x,
            pose.position.y
        ])

        with self.lock:
            if self.previous_odom_xy is not None:
                delta = np.linalg.norm(
                    xy - self.previous_odom_xy
                )

                if 0.001 < delta < 0.25:
                    self.distance_travelled += float(delta)

            self.previous_odom_xy = xy

            self.odom_pose = np.array([
                xy[0],
                xy[1],
                self.quaternion_yaw(pose.orientation)
            ])

            self.odom_speed = float(msg.twist.twist.linear.x)
            self.odom_time = time.monotonic()

    def read_tf_pose(self, target, source, stamp):
        transform = self.tf_buffer.lookup_transform(
            target,
            source,
            stamp
        )

        p = transform.transform.translation

        return np.array([
            p.x,
            p.y,
            self.quaternion_yaw(transform.transform.rotation)
        ], dtype=float)

    def scan_callback(self, msg):
        with self.lock:
            self.pending_scan = msg
            self.pending_at = time.monotonic()
            self.scan_rx += 1

    @staticmethod
    def group_scan(points, valid):
        indices = np.flatnonzero(valid)

        if len(indices) == 0:
            return []

        result = []
        group = [int(indices[0])]

        for index in indices[1:]:
            index = int(index)

            if (
                index == group[-1] + 1
                and np.linalg.norm(
                    points[index] - points[group[-1]]
                ) < 0.18
            ):
                group.append(index)
            else:
                result.append(np.array(group, dtype=int))
                group = [index]

        result.append(np.array(group, dtype=int))

        if (
            len(result) > 1
            and result[0][0] == 0
            and result[-1][-1] == len(valid) - 1
        ):
            if np.linalg.norm(
                points[result[0][0]] - points[result[-1][-1]]
            ) < 0.18:
                result[0] = np.concatenate(
                    (result[-1], result[0])
                )
                result.pop()

        return result

    def update_tracks(self, detections, now):
        previous = {
            k: v
            for k, v in self.tracks.items()
            if now - v['time'] < 0.65
        }

        possible = []

        for key, old in previous.items():
            elapsed = max(0.0, now - old['time'])
            predicted = (
                old['position']
                + old['velocity'] * elapsed
            )

            for i, item in enumerate(detections):
                separation = np.linalg.norm(
                    predicted - item['position']
                )

                if separation < 0.25 + elapsed * 0.6:
                    possible.append((
                        float(separation), key, i
                    ))

        matches = {}
        used_tracks = set()
        used_items = set()

        for _, key, index in sorted(possible):
            if (
                key not in used_tracks
                and index not in used_items
            ):
                matches[index] = key
                used_tracks.add(key)
                used_items.add(index)

        current = {}

        for index, item in enumerate(detections):
            if index in matches:
                key = matches[index]
                old = previous[key]
                elapsed = now - old['time']
                velocity = old['velocity'].copy()

                if elapsed > 0.03:
                    measured = (
                        item['position'] - old['position']
                    ) / elapsed

                    speed = np.linalg.norm(measured)

                    if speed > 1.0:
                        measured *= 1.0 / speed

                    velocity = (
                        0.45 * velocity
                        + 0.55 * measured
                    )

                count = old['count'] + 1

            else:
                key = self.next_track_id
                self.next_track_id += 1
                velocity = np.zeros(2)
                count = 1

            if (
                count >= 3
                and np.linalg.norm(velocity) < 0.055
            ):
                velocity[:] = 0.0

            current[key] = {
                'position': item['position'],
                'velocity': velocity,
                'count': count,
                'time': now
            }

            item['velocity'] = (
                velocity
                if count >= 3
                else np.zeros(2)
            )

        self.tracks = current
        return detections

    def process_pending_scan(self):
        with self.lock:
            msg = self.pending_scan
            queued = self.pending_at

        if msg is None:
            return

        now = time.monotonic()

        if (
            queued is None
            or now - queued > self.tf_retry_timeout
        ):
            with self.lock:
                if self.pending_scan is msg:
                    self.pending_scan = None

            self.warn(
                f'LiDAR TF timeout: {self.last_tf_error}'
            )
            return

        scan_seconds = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        clock_seconds = (
            self.get_clock().now().nanoseconds * 1e-9
        )

        if (
            scan_seconds
            and clock_seconds
            and abs(clock_seconds - scan_seconds) > 2.0
        ):
            with self.lock:
                self.last_tf_error = (
                    f'Clock mismatch: '
                    f'scan={scan_seconds:.2f} '
                    f'clock={clock_seconds:.2f}'
                )
            return

        try:
            stamp = Time.from_msg(msg.header.stamp)

            laser_pose = self.read_tf_pose(
                'odom',
                msg.header.frame_id,
                stamp
            )

            robot_pose = self.read_tf_pose(
                'odom',
                'base_link',
                stamp
            )

        except TransformException as exc:
            with self.lock:
                self.last_tf_error = str(exc)
            return

        with self.lock:
            if self.pending_scan is not msg:
                return
            self.pending_scan = None

        ranges = np.asarray(msg.ranges, dtype=float)

        if len(ranges) == 0:
            return

        valid = (
            np.isfinite(ranges)
            & (ranges >= max(msg.range_min, 0.01))
            & (
                ranges <= min(
                    msg.range_max,
                    self.detection_range
                )
            )
        )

        angles = (
            msg.angle_min
            + np.arange(len(ranges))
            * msg.angle_increment
        )

        points = np.full(
            (len(ranges), 2),
            np.nan,
            dtype=float
        )

        points[valid, 0] = (
            ranges[valid] * np.cos(angles[valid])
        )

        points[valid, 1] = (
            ranges[valid] * np.sin(angles[valid])
        )

        cos_yaw = np.cos(laser_pose[2])
        sin_yaw = np.sin(laser_pose[2])

        rotation = np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw]
        ])

        points[valid] = (
            points[valid] @ rotation.T
            + laser_pose[:2]
        )

        detections = []

        for group in self.group_scan(points, valid):
            if len(group) < 3:
                continue

            segment = points[group]

            if np.linalg.norm(
                np.ptp(segment, axis=0)
            ) <= 0.65:
                detections.append({
                    'position': np.mean(segment, axis=0),
                    'indices': group
                })

        with self.lock:
            detections = self.update_tracks(
                detections,
                time.monotonic()
            )

            velocities = np.zeros_like(points)

            for item in detections:
                velocities[item['indices']] = (
                    item['velocity']
                )

            indices = np.flatnonzero(valid)

            if len(indices) > self.max_scan_points:
                indices = np.array([
                    int(
                        block[
                            np.argmin(ranges[block])
                        ]
                    )
                    for block in np.array_split(
                        indices,
                        self.max_scan_points
                    )
                ])

            self.points = points[indices].copy()
            self.velocities = velocities[indices].copy()
            self.scan_pose = robot_pose

            self.last_scan_time = time.monotonic()
            self.scan_ok += 1
            self.last_tf_error = ''

    def moving_threats(self, pose, robot_v, now):
        robot_velocity = robot_v * np.array([
            np.cos(pose[2]),
            np.sin(pose[2])
        ])

        threats = []

        for track in self.tracks.values():
            if (
                track['count'] < 3
                or now - track['time'] > 0.65
            ):
                continue

            relative = track['position'] - pose[:2]

            relative_velocity = (
                track['velocity']
                - robot_velocity
            )

            magnitude = np.dot(
                relative_velocity,
                relative_velocity
            )

            if (
                magnitude < 1e-4
                or np.linalg.norm(
                    track['velocity']
                ) < 0.07
            ):
                continue

            t = np.clip(
                -np.dot(
                    relative,
                    relative_velocity
                ) / magnitude,
                0.0,
                self.horizon
            )

            if (
                np.dot(relative, relative_velocity) < 0.0
                and np.linalg.norm(
                    relative + relative_velocity * t
                ) < 0.85
            ):
                threats.append(track)

        return threats

    def predictive_dwa(self):
        began = time.perf_counter()
        now = time.monotonic()

        with self.lock:
            scan_at = self.last_scan_time
            nav_at = self.last_nav_time
            odom_at = self.odom_time

            points = self.points.copy()
            velocities = self.velocities.copy()

            scan_pose = self.scan_pose.copy()
            odom_pose = self.odom_pose.copy()

            pose = (
                odom_pose
                if (
                    odom_at is not None
                    and now - odom_at < 0.5
                )
                else scan_pose
            )

            nav_v = float(
                np.clip(
                    self.target_nav.linear.x,
                    -self.max_speed,
                    self.max_speed
                )
            )

            nav_w = float(
                np.clip(
                    self.target_nav.angular.z,
                    -self.max_turn,
                    self.max_turn
                )
            )

            current_v = (
                self.odom_speed
                if (
                    odom_at is not None
                    and now - odom_at < 0.5
                )
                else self.current_cmd.linear.x
            )

            current_w = self.current_cmd.angular.z

            threats = self.moving_threats(
                pose,
                current_v,
                now
            )

            count_rx = self.scan_rx
            count_ok = self.scan_ok
            error = self.last_tf_error

        if (
            scan_at is None
            or now - scan_at > self.scan_timeout
        ):
            self.control_state = 'WAIT_LASER'

            self.warn(
                f'Waiting for usable LiDAR/TF | '
                f'ScanRx={count_rx} '
                f'ScanOK={count_ok} | {error}'
            )

            return Twist(), False

        if (
            nav_at is None
            or now - nav_at > self.nav_timeout
        ):
            self.control_state = 'WAIT_NAV'

            self.warn(
                'No recent Nav2 velocity command '
                'on /cmd_vel_nav'
            )

            return Twist(), False

        if (
            abs(nav_v) < 0.005
            and abs(nav_w) < 0.015
        ):
            self.control_state = 'NAV_HOLD'
            return Twist(), False

        points += (
            velocities
            * max(0.0, now - scan_at)
        )

        start_dist = (
            np.linalg.norm(
                points - pose[:2],
                axis=1
            )
            if len(points)
            else np.empty(0)
        )

        nearest = (
            float(np.min(start_dist))
            if len(start_dist)
            else float('inf')
        )

        if nearest <= self.emergency_radius:
            self.control_state = 'STOP_CLOSE'

            self.warn(
                f'Obstacle at {nearest:.2f} m: '
                f'waiting for clearance'
            )

            return Twist(), False

        safe_bypass = max(
            0.90,
            abs(nav_v) * self.horizon
            + self.radius
            + self.margin
            + 0.20
        )

        if nearest > safe_bypass and not threats:
            command = Twist()
            command.linear.x = nav_v
            command.angular.z = nav_w

            self.control_state = 'FOLLOW'

            self.dwa_ms = (
                1000
                * (time.perf_counter() - began)
            )

            return command, True

        speeds = np.linspace(
            -self.max_speed,
            self.max_speed,
            self.linear_samples
        )

        turns = np.linspace(
            -self.max_turn,
            self.max_turn,
            self.angular_samples
        )

        linear = np.concatenate((
            np.repeat(
                speeds,
                len(turns)
            ),
            [nav_v, 0.0, 0.0, 0.0]
        ))

        angular = np.concatenate((
            np.tile(
                turns,
                len(speeds)
            ),
            [nav_w, 0.0, 0.8, -0.8]
        ))

        x = np.full(
            len(linear),
            pose[0]
        )

        y = np.full(
            len(linear),
            pose[1]
        )

        heading = np.full(
            len(linear),
            pose[2]
        )

        v = np.full(
            len(linear),
            current_v
        )

        w = np.full(
            len(linear),
            current_w
        )

        valid = np.ones(
            len(linear),
            dtype=bool
        )

        closest = np.full(
            len(linear),
            np.inf
        )

        static_mask = (
            np.linalg.norm(
                velocities,
                axis=1
            ) < 0.07
            if len(points)
            else np.empty(0, dtype=bool)
        )

        static_start_min = (
            float(
                np.min(
                    start_dist[static_mask]
                )
            )
            if np.any(static_mask)
            else float('inf')
        )

        elapsed = 0.0

        while elapsed < self.horizon - 1e-8:
            dt = min(
                self.step,
                self.horizon - elapsed
            )

            elapsed += dt

            v += np.clip(
                linear - v,
                -self.max_decel * dt,
                self.max_accel * dt
            )

            w += np.clip(
                angular - w,
                -self.max_turn_accel * dt,
                self.max_turn_accel * dt
            )

            middle = (
                heading
                + 0.5 * w * dt
            )

            x += (
                v
                * np.cos(middle)
                * dt
            )

            y += (
                v
                * np.sin(middle)
                * dt
            )

            heading += w * dt

            if not len(points):
                continue

            predicted = (
                points
                + velocities * elapsed
            )

            dx = (
                x[:, None]
                - predicted[None, :, 0]
            )

            dy = (
                y[:, None]
                - predicted[None, :, 1]
            )

            distances = np.sqrt(
                dx * dx + dy * dy
            )

            closest = np.minimum(
                closest,
                np.min(distances, axis=1)
            )

            hard_radius = (
                self.radius + 0.025
            )

            comfort = (
                self.radius
                + self.margin
                + 0.12 * np.abs(v)
            )

            physical_collision = np.any(
                distances <= hard_radius,
                axis=1
            )

            if np.any(static_mask):
                nearest_static = np.min(
                    distances[:, static_mask],
                    axis=1
                )

                near_static = (
                    nearest_static < comfort
                )

                parallel_or_escaping = (
                    (static_start_min < comfort)
                    & (
                        nearest_static
                        >= static_start_min - 0.008
                    )
                )

                static_violation = (
                    near_static
                    & ~parallel_or_escaping
                )

            else:
                static_violation = np.zeros(
                    len(linear),
                    dtype=bool
                )

            if np.any(~static_mask):
                dynamic_violation = np.any(
                    distances[:, ~static_mask]
                    < comfort[:, None],
                    axis=1
                )

            else:
                dynamic_violation = np.zeros(
                    len(linear),
                    dtype=bool
                )

            valid &= ~(
                physical_collision
                | static_violation
                | dynamic_violation
            )

            if not np.any(valid):
                break

        if not np.any(valid):
            self.control_state = 'STOP_NO_PATH'

            self.warn(
                f'DWA: no safe candidate at '
                f'nearest={nearest:.2f} m; '
                f'rechecking next scan'
            )

            self.dwa_ms = (
                1000
                * (time.perf_counter() - began)
            )

            return Twist(), False

        cost = (
            4.5 * np.abs(
                linear - nav_v
            )
            + 1.7 * np.abs(
                angular - nav_w
            )
        )

        cost += (
            0.7
            * np.maximum(
                0.0,
                -linear
            )
        )

        cost += (
            3.0
            * np.maximum(
                0.0,
                0.47 - closest
            ) ** 2
        )

        if threats:
            cost += (
                0.2
                * np.abs(
                    angular - nav_w
                )
            )

        cost[~valid] = np.inf

        best = int(
            np.argmin(cost)
        )

        selected = Twist()

        selected.linear.x = float(
            linear[best]
        )

        selected.angular.z = float(
            angular[best]
        )

        self.control_state = (
            'FOLLOW'
            if (
                abs(
                    selected.linear.x - nav_v
                ) < 0.04
                and abs(
                    selected.angular.z - nav_w
                ) < 0.20
            )
            else 'AVOID'
        )

        self.dwa_ms = (
            1000
            * (time.perf_counter() - began)
        )

        return selected, True

    @staticmethod
    def path_error(poses, x, y):
        if len(poses) < 2:
            return float('nan'), 0.0

        coords = np.array([
            [
                p.pose.position.x,
                p.pose.position.y
            ]
            for p in poses
        ])

        start = coords[:-1]
        segments = coords[1:] - start

        length2 = np.sum(
            segments * segments,
            axis=1
        )

        good = length2 > 1e-8

        if not np.any(good):
            return (
                float(
                    np.linalg.norm(
                        coords[0] - [x, y]
                    )
                ),
                0.0
            )

        start = start[good]
        segments = segments[good]
        length2 = length2[good]

        t = np.clip(
            np.sum(
                ([x, y] - start) * segments,
                axis=1
            ) / length2,
            0,
            1
        )

        closest = (
            start
            + t[:, None] * segments
        )

        distances = np.linalg.norm(
            closest - [x, y],
            axis=1
        )

        index = int(
            np.argmin(distances)
        )

        return (
            float(distances[index]),
            float(
                np.arctan2(
                    segments[index, 1],
                    segments[index, 0]
                )
            )
        )

    def fresh_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time()
            )

            stamp = (
                transform.header.stamp.sec
                + transform.header.stamp.nanosec * 1e-9
            )

            now = (
                self.get_clock().now().nanoseconds
                * 1e-9
            )

            if (
                now > 0.0
                and (
                    stamp <= 0.0
                    or abs(now - stamp) > 1.0
                )
            ):
                return None

            p = transform.transform.translation

            return np.array([
                p.x,
                p.y,
                self.quaternion_yaw(
                    transform.transform.rotation
                )
            ])

        except TransformException:
            return None

    def goal_distance(self, goal):
        pose = self.fresh_map_pose()

        if pose is None:
            return None

        return float(
            np.hypot(
                pose[0] - goal[0],
                pose[1] - goal[1]
            )
        )

    def telemetry(self):
        with self.lock:
            state = self.control_state
            waypoint = self.waypoint
            scan_rx = self.scan_rx
            scan_ok = self.scan_ok

            nav = self.target_nav.linear.x
            cmd = self.current_cmd.linear.x
            odom = self.odom_speed
            distance = self.distance_travelled

            path = list(self.plan)
            plan_updates = self.plan_updates
            retry = self.recovery_attempt
            finished = self.mission_finished

        if finished:
            return

        map_pose = self.fresh_map_pose()

        if map_pose is None:
            self.get_logger().info(
                f'[{state} WP_{waypoint}] '
                f'ScanRx={scan_rx} '
                f'ScanOK={scan_ok} | '
                f'Nav={nav:.2f} '
                f'Cmd={cmd:.2f} '
                f'Odom={odom:.2f} | '
                f'Dist={distance:.2f} m | '
                f'TF map pose unavailable'
            )
            return

        if len(path) > 1:
            xt, path_heading = self.path_error(
                path,
                map_pose[0],
                map_pose[1]
            )

            heading_error = np.degrees(
                (
                    path_heading
                    - map_pose[2]
                    + np.pi
                ) % (2 * np.pi)
                - np.pi
            )

            goal = path[-1].pose.position

            remaining = np.hypot(
                map_pose[0] - goal.x,
                map_pose[1] - goal.y
            )

            path_msg = (
                f'XT={xt * 100:.0f}cm '
                f'Heading={heading_error:.0f}deg '
                f'Goal={remaining:.2f}m'
            )

        else:
            path_msg = (
                'Waiting for new global path'
            )

        self.get_logger().info(
            f'[{state} WP_{waypoint} Retry={retry}] '
            f'{path_msg} | '
            f'Nav={nav:.2f} '
            f'Cmd={cmd:.2f} '
            f'Odom={odom:.2f} | '
            f'Travel={distance:.2f}m | '
            f'ScanRx={scan_rx} '
            f'ScanOK={scan_ok} | '
            f'Plan={plan_updates} '
            f'DWA={self.dwa_ms:.0f}ms'
        )

    def publish_obstacles(self, now):
        directions = self.obstacle_directions

        for index in range(4):
            duration = (
                14.5
                if index in (0, 2)
                else 14.5 * 0.82
            )

            if (
                now - self.obstacle_times[index]
                >= duration
            ):
                directions[index] *= -1.0
                self.obstacle_times[index] = now

            msg = Twist()

            if index in (0, 2):
                msg.linear.y = float(
                    0.20 * directions[index]
                )

            else:
                msg.linear.x = float(
                    0.20 * directions[index]
                )

            self.pub_obs[index].publish(msg)

    def control_loop(self):
        if not rclpy.ok():
            return

        now = time.monotonic()

        dt = float(
            np.clip(
                now - self.last_control,
                0.01,
                0.20
            )
        )

        self.last_control = now

        with self.lock:
            finished = self.mission_finished

        if finished:
            self.pub_robot_vel.publish(Twist())

            for pub in self.pub_obs:
                pub.publish(Twist())

            return

        self.publish_obstacles(now)

        target, allowed = self.predictive_dwa()

        if not allowed:
            self.current_cmd = Twist()

        else:
            old = self.current_cmd
            cmd = Twist()

            cmd.linear.x = float(
                old.linear.x
                + np.clip(
                    target.linear.x - old.linear.x,
                    -self.max_decel * dt,
                    self.max_accel * dt
                )
            )

            cmd.angular.z = float(
                old.angular.z
                + np.clip(
                    target.angular.z - old.angular.z,
                    -self.max_turn_accel * dt,
                    self.max_turn_accel * dt
                )
            )

            self.current_cmd = cmd

        self.pub_robot_vel.publish(
            self.current_cmd
        )

    def goal_pose(self, xy):
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = (
            self.navigator.get_clock().now().to_msg()
        )

        pose.pose.position.x = float(xy[0])
        pose.pose.position.y = float(xy[1])
        pose.pose.orientation.w = 1.0

        return pose

    def run_mission(self):
        try:
            while (
                rclpy.ok()
                and self.get_clock().now().nanoseconds == 0
            ):
                time.sleep(0.1)

            initial = PoseStamped()
            initial.header.frame_id = 'map'
            initial.header.stamp = (
                self.navigator.get_clock().now().to_msg()
            )
            initial.pose.orientation.w = 1.0

            self.navigator.setInitialPose(initial)
            self.navigator.waitUntilNav2Active()

            while rclpy.ok():
                with self.lock:
                    ready_scan = (
                        self.last_scan_time is not None
                        and time.monotonic()
                        - self.last_scan_time
                        < self.scan_timeout
                    )

                    ready_odom = (
                        self.odom_time is not None
                        and time.monotonic()
                        - self.odom_time
                        < 1.0
                    )

                if (
                    ready_scan
                    and ready_odom
                    and self.fresh_map_pose() is not None
                ):
                    break

                self.warn(
                    'Mission waiting for fresh LiDAR, '
                    'odometry and map TF'
                )

                time.sleep(0.2)

            if not rclpy.ok():
                return

            goals = [
                (3.34, 3.14),
                (3.19, -3.06),
                (-3.34, -2.96),
                (-3.41, 3.11),
                (-1.09, 1.08),
                (0.0, 0.0)
            ]

            for index, goal in enumerate(goals, 1):
                if not rclpy.ok():
                    return

                with self.lock:
                    self.waypoint = index
                    self.plan = []

                for attempt in range(
                    1,
                    self.retry_limit + 1
                ):
                    with self.lock:
                        self.recovery_attempt = attempt

                    remaining = self.goal_distance(goal)

                    if (
                        remaining is not None
                        and remaining <= 0.35
                    ):
                        self.get_logger().info(
                            f'Goal {index}/6 already reached '
                            f'({remaining:.2f} m)'
                        )
                        break

                    self.get_logger().info(
                        f'Navigating to goal {index}/6 | '
                        f'attempt {attempt}/{self.retry_limit}'
                    )

                    accepted = self.navigator.goToPose(
                        self.goal_pose(goal)
                    )

                    if accepted:
                        while (
                            rclpy.ok()
                            and not self.navigator.isTaskComplete()
                        ):
                            time.sleep(0.1)

                        if not rclpy.ok():
                            return

                        result = self.navigator.getResult()

                    else:
                        result = TaskResult.FAILED

                    remaining = self.goal_distance(goal)

                    if (
                        result == TaskResult.SUCCEEDED
                        and remaining is not None
                        and remaining <= 0.40
                    ):
                        self.get_logger().info(
                            f'Goal {index}/6 verified | '
                            f'remaining={remaining:.2f} m'
                        )
                        break

                    self.get_logger().warn(
                        f'Goal {index}/6 not verified | '
                        f'Nav2={result} '
                        f'distance={remaining} | '
                        f'retrying after clearance'
                    )

                    if attempt == self.retry_limit:
                        self.get_logger().error(
                            f'Mission stopped safely: '
                            f'goal {index} failed after retries'
                        )

                        with self.lock:
                            self.mission_finished = True
                            self.mission_success = False

                        return

                    wait_until = (
                        time.monotonic() + 3.0
                    )

                    while (
                        rclpy.ok()
                        and time.monotonic() < wait_until
                    ):
                        time.sleep(0.2)

                else:
                    return

            with self.lock:
                self.mission_finished = True
                self.mission_success = True
                travelled = self.distance_travelled

            self.get_logger().info(
                f'All 6 goals verified by position | '
                f'travelled {travelled:.2f} m'
            )

        except Exception as exc:
            self.get_logger().error(
                f'Mission error: {exc}'
            )

            with self.lock:
                self.mission_finished = True
                self.mission_success = False


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = SimulationMaster()

        executor = rclpy.executors.MultiThreadedExecutor(
            num_threads=4
        )

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
