import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class MoveToNode(Node):
    def __init__(self):
        super().__init__("take_input_2")
        self.get_logger().info("Node Started")
        self.cmd_vel_pub = self.create_publisher(Twist, "/TB_01/cmd_vel", 10)#get target x coordinate from input
        qos_profile = QoSProfile(depth =10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT #required for Qos
        self.odom_sub = self.create_subscription(PoseStamped, "/vrpn_mocap/Rigid_body_002/pose", self.vrpn_mocap_pose_callback, qos_profile)#create subscriber to get position via motive tracker

        self.x_target = float(sys.argv[1]) #get target x coordinate from input
        self.y_target = float(sys.argv[2]) #get target y coordinate from input
        self.final_yaw = math.radians(float(sys.argv[3]))  # get final orientation from input

        self.state = 'ROTATE_TO_Y'  # Start by rotating to +Y direction
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0} #store coordinates and pose

        self.yaw_tolerance = 0.05  # radians (~5.7 degrees)
        self.pos_tolerance = 0.05  # meters

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("MoveToNode started")

    def vrpn_mocap_pose_callback(self, msg: PoseStamped): #update self.pose
        self.pose['x'] = msg.pose.position.x
        self.pose['y'] = msg.pose.position.y

        q = msg.pose.orientation # convert quaternion to get roll pitch and yaw
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w]) # store yaw (in 2D)
        self.pose['yaw'] = yaw

    def normalize_angle(self, angle):#transform angles from -180 to 180 degrees
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle <= -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        twist = Twist()
        current_yaw = self.normalize_angle(self.pose['yaw'])

        if self.state == 'ROTATE_TO_Y': #rotate towards y axis
            target_yaw = math.pi / 2  # 90 degrees
            error = self.normalize_angle(target_yaw - current_yaw)
            if abs(error) > self.yaw_tolerance:
                twist.angular.z = 0.3 if error > 0 else -0.3
            else:
                self.get_logger().info("Rotated to +Y axis")
                self.state = 'MOVE_Y'

        elif self.state == 'MOVE_Y': #move along y axis
            distance = self.y_target - self.pose['y']
            if abs(distance) > self.pos_tolerance:
                twist.linear.x = 0.2 if distance > 0 else -0.2
            else:
                self.get_logger().info(f"Reached Y = {self.y_target:.2f}")
                self.state = 'ROTATE_TO_X'

        elif self.state == 'ROTATE_TO_X': #rotate towards x axis
            target_yaw = 0.0
            error = self.normalize_angle(target_yaw - current_yaw)
            if abs(error) > self.yaw_tolerance:
                twist.angular.z = 0.3 if error > 0 else -0.3
            else:
                self.get_logger().info("Rotated to +X axis")
                self.state = 'MOVE_X'

        elif self.state == 'MOVE_X': #move along x axis
            distance = self.x_target - self.pose['x']
            if abs(distance) > self.pos_tolerance:
                twist.linear.x = 0.2 if distance > 0 else -0.2
            else:
                self.get_logger().info(f"Reached X = {self.x_target:.2f}")
                self.state = 'FINAL_ROTATE'

        elif self.state == 'FINAL_ROTATE': #finally rotate towards final required orientation
            target_yaw = self.normalize_angle(self.final_yaw)
            error = self.normalize_angle(target_yaw - current_yaw)
            if abs(error) > self.yaw_tolerance:
                twist.angular.z = 0.3 if error > 0 else -0.3
            else:
                self.get_logger().info(f"Final orientation {math.degrees(self.final_yaw):.1f}° set")
                self.state = 'DONE'

        elif self.state == 'DONE':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist) #publish velocity continuously


def main(args=None):
    rclpy.init(args=args)
    node = MoveToNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()