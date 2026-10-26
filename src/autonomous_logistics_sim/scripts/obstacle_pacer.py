#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ObstaclePacer(Node):
    def __init__(self):
        super().__init__('obstacle_pacer')
        # Publisher configured to talk directly to your orange pole's remapped control channel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_obstacle', 10)
        
        # Timer ticks every 4.0 seconds to alternate directions
        self.timer = self.create_timer(4.0, self.timer_callback)
        self.msg = Twist()
        self.direction = 1.0  # 1.0 means forward along X, -1.0 means backward
        self.get_logger().info('Industrial Obstacle Pacer Node Activated! The orange pole is ready.')

    def timer_callback(self):
        # Set speed: Alternates between 0.3 m/s and -0.3 m/s
        self.msg.linear.x = 0.3 * self.direction
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        
        # Publish the command to Gazebo
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Obstacle moving along X-axis with velocity: {self.msg.linear.x} m/s')
        
        # Invert direction multiplier for the next timer cycle
        self.direction *= -1.0

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePacer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on node shutdown: commands 0 velocity so the pole doesn't drift infinitely
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
