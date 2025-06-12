#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf_transformations import euler_from_quaternion
import numpy as np
from scipy.optimize import fsolve
import math

class follow8(Node):
    def __init__(self):
        super().__init__('follow8')

        # === Parameters (EDIT THESE AS NEEDED) ===
        self.r1 = 2   # Circle 1 radius
        self.r2 = 2   # Circle 2 radius
        self.d = 3    # Distance between c1 and c2 centers
        self.r3 = 0.5  # Circle 3 radius (upper tangent)
        self.r4 = 0.5  # Circle 4 radius (lower tangent)

        self.MAX_LIN_VEL = 0.18
        self.MAX_ANG_VEL = 2.2

        # Rotation angle (90 degrees clockwise)
        angle = -np.pi / 2  # negative for clockwise rotation

        # Rotation helper
        def rotate_point(point, angle_rad=angle):
            c, s = np.cos(angle_rad), np.sin(angle_rad)
            R = np.array([[c, -s],
                          [s,  c]])
            return R.dot(point)

        # === Calculate circle centers BEFORE rotation ===
        c1_raw = np.array([self.r1, 0.0])
        c2_raw = np.array([self.r1 + self.d, 0.0])

        # === Calculate tangent circle centers BEFORE rotation ===
        c3_raw = self.find_tangent_circle(c1_raw, self.r1, c2_raw, self.r2, self.r3, [self.r1 + self.d/2, self.r1])
        c4_raw = self.find_tangent_circle(c1_raw, self.r1, c2_raw, self.r2, self.r4, [self.r1 + self.d/2, -self.r1])

        # === Rotate all centers ===
        self.c1 = rotate_point(c1_raw)
        self.c2 = rotate_point(c2_raw)
        self.c3 = rotate_point(c3_raw)
        self.c4 = rotate_point(c4_raw)

        # === Calculate intersection points ON ROTATED centers ===
        self.intersections = [
            self.closest_points_on_circle_pair(self.c1, self.r1, self.c3, self.r3)[0],  # c1->c3
            self.closest_points_on_circle_pair(self.c3, self.r3, self.c2, self.r2)[0],  # c3->c2
            self.closest_points_on_circle_pair(self.c2, self.r2, self.c4, self.r4)[0],  # c2->c4
            self.closest_points_on_circle_pair(self.c4, self.r4, self.c1, self.r1)[0],  # c4->c1
        ]
        self.print_intersection_points()

        # === Explicit parity: 1 = CCW, -1 = CW ===
        # c1 (CW), c3 (CCW), c2 (CW), c4 (CCW)
        self.path_sequence = [
            (self.c1, self.r1, self.intersections[0], -1),  # c1, CW
            (self.c3, self.r3, self.intersections[1],  1),  # c3, CCW
            (self.c2, self.r2, self.intersections[2], -1),  # c2, CW
            (self.c4, self.r4, self.intersections[3],  1),  # c4, CCW
        ]
        self.current_segment = 0

        # === Variables for distance monitoring ===
        self.min_distance = float('inf')
        self.distance_below_threshold = False

        # === ROS2 setup ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.current_position = Point()
        self.current_yaw = 0.0
        self.state = 'moving'  # start moving immediately assuming robot faces +X
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.update_velocities()

    def print_intersection_points(self):
        self.get_logger().info("\n=== COMPUTED INTERSECTION POINTS ===")
        for i, point in enumerate(self.intersections):
            self.get_logger().info(f"Intersection {i+1}: [{point[0]:.4f}, {point[1]:.4f}]")

    def find_tangent_circle(self, c1, r1, c2, r2, r_fixed, guess):
        def system(vars, c1, r1, c2, r2, r_fixed):
            x, y = vars
            eq1 = np.hypot(x - c1[0], y - c1[1]) - (r_fixed + r1)
            eq2 = np.hypot(x - c2[0], y - c2[1]) - (r_fixed + r2)
            return [eq1, eq2]
        sol = fsolve(system, guess, args=(c1, r1, c2, r2, r_fixed))
        return np.array(sol)

    def closest_points_on_circle_pair(self, c1, r1, c2, r2):
        v = c2 - c1
        d = np.linalg.norm(v)
        if d == 0:
            return [c1, c1]
        u = v / d
        p1 = c1 + u * r1
        p2 = c2 - u * r2
        return [p1, p2]

    def odom_cb(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_yaw = euler_from_quaternion(orientation_list)[2]

    def update_velocities(self):
        center, radius, _, parity = self.path_sequence[self.current_segment]
        radius = max(radius, 0.01)  # Prevent division by zero
        ang_vel = self.MAX_LIN_VEL / radius
        if abs(ang_vel) > self.MAX_ANG_VEL:
            ang_vel = math.copysign(self.MAX_ANG_VEL, ang_vel)
            lin_vel = ang_vel * radius
        else:
            lin_vel = self.MAX_LIN_VEL
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel * parity

        # Reset min distance tracking for new segment
        self.min_distance = float('inf')
        self.distance_below_threshold = False

    def control_loop(self):
        twist = Twist()
        if self.state == 'moving':
            twist.linear.x = self.lin_vel
            twist.angular.z = self.ang_vel
            self.cmd_vel_pub.publish(twist)

            _, _, intersection, _ = self.path_sequence[self.current_segment]
            dx = intersection[0] - self.current_position.x
            dy = intersection[1] - self.current_position.y
            distance = math.hypot(dx, dy)

            self.get_logger().info(
                f"Segment {self.current_segment+1} → Distance to target: {distance:.3f}m "
                f"(Target: [{intersection[0]:.3f}, {intersection[1]:.3f}], "
                f"Current: [{self.current_position.x:.3f}, {self.current_position.y:.3f}])"
            )

            threshold = 0.07  # 10cm threshold to start monitoring closeness
            increase_margin = (1/100)*0.1  # 2cm margin to detect meaningful increase

            if distance < threshold:
                self.distance_below_threshold = True

            # Update min_distance if current distance is smaller
            if distance < self.min_distance:
                self.min_distance = distance

            # If distance increases meaningfully above min_distance, switch segment
            if self.distance_below_threshold and (distance - self.min_distance) > increase_margin:
                self.get_logger().info(
                    f"Distance increased by more than {increase_margin:.2f}m from min distance, switching segment {self.current_segment+1}"
                )
                self.current_segment = (self.current_segment + 1) % len(self.path_sequence)
                self.update_velocities()

def main(args=None):
    rclpy.init(args=args)
    controller = follow8()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
