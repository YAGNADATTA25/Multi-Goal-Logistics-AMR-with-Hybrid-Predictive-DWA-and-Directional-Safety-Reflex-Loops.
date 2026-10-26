#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # --- 1. SET INITIAL POSE ---
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0 
    initial_pose.pose.position.y = 0.0
    # FIX: Orientation W must be 1.0 for a valid '0' rotation
    initial_pose.pose.orientation.w = 1.0 
    
    print("Step 1: Setting initial pose...")
    navigator.setInitialPose(initial_pose)

    # --- 2. WAIT FOR ACTIVATION ---
    # We add a small timeout/retry logic to ensure the lifecycle manager 
    # has time to flip nodes to ACTIVE.
    print("Step 2: Waiting for Navigation to reach ACTIVE state...")
    
    # Wait for Nav2 to be ready
    navigator.waitUntilNav2Active()
    
    print("Nav2 is now ACTIVE! Commencing mission...")

    # --- 3. MISSION GOALS ---
    mission_goals = [
        [3.34, 3.14],   # Goal Point 1
        [3.19, -3.06],  # Goal Point 2
        [-3.34, -2.96], # Goal Point 3
        [-3.41, 3.11],  # Goal Point 4
        [-1.64, 2.54]   # Goal Point 5
    ]

    goal_poses = []
    for pt in mission_goals:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.orientation.w = 1.0
        goal_poses.append(pose)

    # --- 4. START MOVEMENT ---
    print(f"Step 3: Sending {len(goal_poses)} waypoints...")
    
    # We use followWaypoints for a continuous circuit
    navigator.followWaypoints(goal_poses)

    # Monitoring movement with cleaner feedback
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0: # Reduce print frequency to keep terminal clean
            print(f'Navigating to waypoint: {feedback.current_waypoint + 1}')

    # --- 5. FINAL RESULT ---
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('\nSUCCESS: Warehouse circuit complete! Returning to base...')
    elif result == TaskResult.CANCELED:
        print('\nREJECTED: The mission was canceled.')
    elif result == TaskResult.FAILED:
        print('\nERROR: The mission failed.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
