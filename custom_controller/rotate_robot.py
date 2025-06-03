import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist

class rotate_robot(Node):
    def __init__(self):
        super().__init__("rotate_robot")
        self.get_logger().info("Node Started")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.dir = float(sys.argv[1]) #get direction of angular velocity
        
        twist = Twist()
        twist.linear.x = 0.0
        if(self.dir>0):
            self.get_logger().info("Rotating anti-clockwise")
            twist.angular.z = 0.2
        else:
            self.get_logger().info("Rotating clockwise")
            twist.angular.z = -0.2

        self.cmd_vel_pub.publish(twist) #publish velocity continuously


def main(args=None):
    rclpy.init(args=args)
    node = rotate_robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()