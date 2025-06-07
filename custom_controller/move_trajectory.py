# import rclpy
# import sys
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class move_trajectory(Node):
#     def __init__(self):
#         super().__init__("move_trajectory")
#         self.get_logger().info("Node Started")

#         self.cmd_vel_pub_1 = self.create_publisher(Twist, "/cmd_vel", 10)

#         try:
#             self.dir1 = float(sys.argv[1])  # linear velocity
#             self.dir2 = float(sys.argv[2])  # angular velocity
#         except (IndexError, ValueError):
#             self.get_logger().error("Usage: ros2 run <package> <node> <dir1> <dir2> (e.g., 1 -1)")
#             rclpy.shutdown()
#             return


#         # Timer to continuously publish velocity commands
#         self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz


#     def control_loop(self):
#         twist1 = Twist()
#         twist1.linear.x = self.dir1
#         twist1.angular.z = self.dir2
#         self.cmd_vel_pub_1.publish(twist1)



# def main(args=None):
#     rclpy.init(args=args)
#     node = move_trajectory()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
from tf_transformations import euler_from_quaternion

class move_trajectory(Node):
    def __init__(self):
        super().__init__('move_trajectory')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Ellipse parameters (modify these for different shapes)
        self.a = float(sys.argv[1])  # Semi-major axis
        self.b = float(sys.argv[2])  # Semi-minor axis
        self.angular_speed = 0.  # radians/sec
        self.theta = 0.0

        # Control gains (may need tuning for ellipse)
        self.kp_linear = 0.8
        self.kp_angular = 3.5

        # Time tracking
        self.last_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def control_loop(self):
        # Update theta based on elapsed time
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.theta += self.angular_speed * dt

        # Calculate target position on ellipse
        x_target = self.a * math.cos(self.theta)
        y_target = self.b * math.sin(self.theta)

        # Compute errors
        error_x = x_target - self.x
        error_y = y_target - self.y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)
        angle_error = self.normalize_angle(angle_to_target - self.yaw)

        # Proportional controller
        cmd = Twist()
        cmd.linear.x = min(self.kp_linear * distance_error, 0.22)
        cmd.angular.z = max(min(self.kp_angular * angle_error, 2.84), -2.84)

        # Safety check for angular alignment
        if abs(angle_error) > math.pi / 2:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = move_trajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
