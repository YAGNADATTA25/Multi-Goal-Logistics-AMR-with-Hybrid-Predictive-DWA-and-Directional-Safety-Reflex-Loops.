import sys
import math
import re
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


bag_path = sys.argv[1]

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    ),
    rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
)

topic_types = {
    topic.name: topic.type
    for topic in reader.get_all_topics_and_types()
}

wanted = {
    '/odom',
    '/scan',
    '/plan',
    '/rosout'
}

message_classes = {
    topic: get_message(topic_types[topic])
    for topic in wanted
    if topic in topic_types
}

previous_xy = None
travel_distance = 0.0
speeds = []
laser_distances = []
path_messages = 0
changed_paths = 0
previous_path = None

dwa_times = []
cross_track_errors = []
verified_goals = set()
collision_stops = 0
collision_slowdowns = 0

mission_start = None
mission_end = None

while reader.has_next():
    topic, raw_data, timestamp = reader.read_next()

    if topic not in message_classes:
        continue

    msg = deserialize_message(
        raw_data,
        message_classes[topic]
    )

    if topic == '/odom':
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if previous_xy is not None:
            distance = math.hypot(
                x - previous_xy[0],
                y - previous_xy[1]
            )

            if 0.001 < distance < 0.25:
                travel_distance += distance

        previous_xy = (x, y)
        speeds.append(abs(msg.twist.twist.linear.x))

    elif topic == '/scan':
        valid = [
            value
            for value in msg.ranges
            if (
                math.isfinite(value)
                and msg.range_min <= value <= msg.range_max
            )
        ]

        if valid:
            laser_distances.append(min(valid))

    elif topic == '/plan':
        path_messages += 1

        path = tuple(
            (
                round(p.pose.position.x, 2),
                round(p.pose.position.y, 2)
            )
            for p in msg.poses
        )

        if path != previous_path:
            changed_paths += 1
            previous_path = path

    elif topic == '/rosout':
        text = msg.msg

        if (
            mission_start is None
            and (
                'Navigating to goal 1/6' in text
                or 'Following 6 goals' in text
            )
        ):
            mission_start = timestamp

        if (
            'All 6 goals verified' in text
            or 'Six-waypoint action completed' in text
            or 'All six waypoints completed successfully' in text
        ):
            mission_end = timestamp

        match = re.search(
            r'Goal (\d+)/6 verified',
            text
        )

        if match:
            verified_goals.add(int(match.group(1)))

        match = re.search(
            r'DWA=([\d.]+)\s*ms',
            text
        )

        if match:
            dwa_times.append(float(match.group(1)))

        match = re.search(
            r'XT=([\d.]+)\s*cm',
            text
        )

        if match:
            cross_track_errors.append(float(match.group(1)))

        if 'Robot to stop due to PolygonStop' in text:
            collision_stops += 1

        if 'Robot to slowdown' in text:
            collision_slowdowns += 1


print('\nWAREHOUSE AMR RESULTS')
print('-----------------------------')
print(f'Total odometry distance: {travel_distance:.2f} m')

if speeds:
    moving_speeds = [v for v in speeds if v > 0.01]

    print(f'Maximum measured speed: {max(speeds):.3f} m/s')

    if moving_speeds:
        print(
            f'Mean speed while moving: '
            f'{statistics.mean(moving_speeds):.3f} m/s'
        )

if laser_distances:
    print(
        f'Minimum valid LiDAR range: '
        f'{min(laser_distances):.3f} m'
    )

print(f'Global path messages: {path_messages}')
print(f'Changed path geometries: {changed_paths}')

if dwa_times:
    print(
        f'Mean reported DWA time: '
        f'{statistics.mean(dwa_times):.2f} ms'
    )
    print(
        f'Maximum reported DWA time: '
        f'{max(dwa_times):.2f} ms'
    )

if cross_track_errors:
    print(
        f'Mean reported cross-track error: '
        f'{statistics.mean(cross_track_errors):.2f} cm'
    )

print(f'Individually verified goal logs: {len(verified_goals)}')
print(f'Collision Monitor stop events: {collision_stops}')
print(f'Collision Monitor slowdown events: {collision_slowdowns}')

if (
    mission_start is not None
    and mission_end is not None
    and mission_end >= mission_start
):
    print(
        f'Mission time: '
        f'{(mission_end - mission_start) / 1e9:.2f} seconds'
    )
else:
    print('Mission time: not identifiable from recorded logs')