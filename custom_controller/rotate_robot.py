# import rclpy
# import sys
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class rotate_robot(Node):
#     def __init__(self):
#         super().__init__("rotate_robot")
#         self.get_logger().info("Node Started")

#         self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

#         self.dir = float(sys.argv[1])  # Get direction from command-line input

#         # Log direction once
#         if self.dir > 0:
#             self.get_logger().info("Rotating anti-clockwise")
#         elif self.dir == 0:
#             self.get_logger().info("Stopping")
#         else:
#             self.get_logger().info("Rotating clockwise")

#         self.timer = self.create_timer(0.1, self.control_loop)

#     def control_loop(self):
#         twist = Twist()
#         twist.linear.x = 0.0
#         if self.dir > 0:
#             twist.angular.z = 0.2
#         elif self.dir == 0:
#             twist.angular.z = 0.0
#         else:
#             twist.angular.z = -0.2
#         self.cmd_vel_pub.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = rotate_robot()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DualRotateRobot(Node):
    def __init__(self):
        super().__init__("dual_rotate_robot")
        self.get_logger().info("Node Started")

        # Publishers for two robots
        self.cmd_vel_pub_1 = self.create_publisher(Twist, "/tb3_1/cmd_vel", 10)
        self.cmd_vel_pub_2 = self.create_publisher(Twist, "/cmd_vel", 10)

        # Parse direction inputs
        try:
            self.dir1 = float(sys.argv[1])  # Direction for /tb3_1/cmd_vel
            self.dir2 = float(sys.argv[2])  # Direction for /cmd_vel
        except (IndexError, ValueError):
            self.get_logger().error("Usage: ros2 run <package> <node> <dir1> <dir2> (e.g., 1 -1)")
            rclpy.shutdown()
            return

        # Direction logging
        self.describe_direction(self.dir1, "tb3_1")
        self.describe_direction(self.dir2, "base robot")

        # Timer to continuously publish velocity commands
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def describe_direction(self, direction, name):
        if direction > 0:
            self.get_logger().info(f"{name} rotating anti-clockwise")
        elif direction < 0:
            self.get_logger().info(f"{name} rotating clockwise")
        else:
            self.get_logger().info(f"{name} not rotating (angular velocity = 0)")

    def control_loop(self):
        # Twist for /tb3_1/cmd_vel
        twist1 = Twist()
        twist1.linear.x = 0.0
        if self.dir1 > 0:
            twist1.angular.z = 0.2
        elif self.dir1 < 0:
            twist1.angular.z = -0.2
        else:
            twist1.angular.z = 0.0
        self.cmd_vel_pub_1.publish(twist1)

        # Twist for /cmd_vel
        twist2 = Twist()
        twist2.linear.x = 0.0
        if self.dir2 > 0:
            twist2.angular.z = 0.2
        elif self.dir2 < 0:
            twist2.angular.z = -0.2
        else:
            twist2.angular.z = 0.0
        self.cmd_vel_pub_2.publish(twist2)


def main(args=None):
    rclpy.init(args=args)
    node = DualRotateRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()