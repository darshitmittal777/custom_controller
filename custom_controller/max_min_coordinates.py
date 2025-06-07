#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class max_min_coordinates(Node):
    def __init__(self):
        super().__init__('odometry_tracker')
        
        # Initialize trackers with extreme values
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        
        # Create subscriber for odometry data
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Default odometry topic for TurtleBot3
            self.odom_callback,
            10)
        
        self.get_logger().info("Odometry tracker node initialized")

    def odom_callback(self, msg):
        # Extract current position from odometry message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Update min/max values
        self.update_extremes(current_x, current_y)
        
    def update_extremes(self, x, y):
        updated = False
        
        if x < self.min_x:
            self.min_x = x
            updated = True
        if x > self.max_x:
            self.max_x = x
            updated = True
        if y < self.min_y:
            self.min_y = y
            updated = True
        if y > self.max_y:
            self.max_y = y
            updated = True
            
        if updated:
            self.log_extremes()
            
    def log_extremes(self):
        self.get_logger().info(
            f"New extremes - Min X: {self.min_x:.2f}, Max X: {self.max_x:.2f}, "
            f"Min Y: {self.min_y:.2f}, Max Y: {self.max_y:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    tracker = max_min_coordinates()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
