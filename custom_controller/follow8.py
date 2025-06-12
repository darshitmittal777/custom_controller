#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf_transformations import euler_from_quaternion
import numpy as np
from scipy.optimize import fsolve
import math
import threading
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class follow8(Node):
    def __init__(self):
        super().__init__('follow8')

        # === PARAMETERS ===
        self.r1 = 2
        self.r2 = 2
        self.d = 3
        self.r3 = 0.5
        self.r4 = 0.5
        self.MAX_LIN_VEL = 0.18
        self.MAX_ANG_VEL = 2.2

        # === PLOTTING SETUP ===
        self.x_data = []
        self.y_data = []
        self.plot_thread = threading.Thread(target=self.live_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()

        # === ROTATE CIRCLE CONFIGURATION ===
        angle = -np.pi / 2
        def rotate_point(point, angle_rad=angle):
            c, s = np.cos(angle_rad), np.sin(angle_rad)
            R = np.array([[c, -s], [s, c]])
            return R.dot(point)

        # Circle centers before rotation
        c1_raw = np.array([self.r1, 0.0])
        c2_raw = np.array([self.r1 + self.d, 0.0])
        c3_raw = self.find_tangent_circle(c1_raw, self.r1, c2_raw, self.r2, self.r3, [self.r1 + self.d/2, self.r1])
        c4_raw = self.find_tangent_circle(c1_raw, self.r1, c2_raw, self.r2, self.r4, [self.r1 + self.d/2, -self.r1])

        # Apply rotation to all centers
        self.c1 = rotate_point(c1_raw)
        self.c2 = rotate_point(c2_raw)
        self.c3 = rotate_point(c3_raw)
        self.c4 = rotate_point(c4_raw)

        # === INTERSECTION POINTS (ALONG PATH SEQUENCE) ===
        self.intersections = [
            self.closest_points_on_circle_pair(self.c1, self.r1, self.c3, self.r3)[0],
            self.closest_points_on_circle_pair(self.c3, self.r3, self.c2, self.r2)[0],
            self.closest_points_on_circle_pair(self.c2, self.r2, self.c4, self.r4)[0],
            self.closest_points_on_circle_pair(self.c4, self.r4, self.c1, self.r1)[0],
        ]
        self.print_intersection_points()

        # === PATH FOLLOWING SEQUENCE: (center, radius, target intersection point, direction) ===
        self.path_sequence = [
            (self.c1, self.r1, self.intersections[0], -1),
            (self.c3, self.r3, self.intersections[1],  1),
            (self.c2, self.r2, self.intersections[2], -1),
            (self.c4, self.r4, self.intersections[3],  1),
        ]
        self.current_segment = 0
        self.min_distance = float('inf')
        self.distance_below_threshold = False

        # === ROS2 SETUP ===
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.current_position = Point()
        self.current_yaw = 0.0
        self.state = 'moving'
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
        self.x_data.append(self.current_position.x)
        self.y_data.append(self.current_position.y)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_yaw = euler_from_quaternion(orientation_list)[2]

    def update_velocities(self):
        center, radius, _, parity = self.path_sequence[self.current_segment]
        radius = max(radius, 0.01)
        ang_vel = self.MAX_LIN_VEL / radius
        if abs(ang_vel) > self.MAX_ANG_VEL:
            ang_vel = math.copysign(self.MAX_ANG_VEL, ang_vel)
            lin_vel = ang_vel * radius
        else:
            lin_vel = self.MAX_LIN_VEL
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel * parity
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

            threshold = 0.07
            increase_margin = (1 / 100) * 0.1

            if distance < threshold:
                self.distance_below_threshold = True
            if distance < self.min_distance:
                self.min_distance = distance
            if self.distance_below_threshold and (distance - self.min_distance) > increase_margin:
                self.get_logger().info(f"Switching to segment {(self.current_segment + 1) % len(self.path_sequence)}")
                self.current_segment = (self.current_segment + 1) % len(self.path_sequence)
                self.update_velocities()

    def live_plot(self):
        plt.ion()
        fig, ax = plt.subplots()
        traj_line, = ax.plot([], [], 'b-', label='Robot Trajectory')

        # === Plot static reference circles ===
        circles = [
            patches.Circle(self.c1, self.r1, fill=False, color='r', linestyle='--', label='C1'),
            patches.Circle(self.c2, self.r2, fill=False, color='g', linestyle='--', label='C2'),
            patches.Circle(self.c3, self.r3, fill=False, color='m', linestyle='--', label='C3'),
            patches.Circle(self.c4, self.r4, fill=False, color='orange', linestyle='--', label='C4'),
        ]
        for circle in circles:
            ax.add_patch(circle)

        # === Plot intersection points ===
        for pt in self.intersections:
            ax.plot(pt[0], pt[1], 'ko', markersize=5, label='Intersection' if pt is self.intersections[0] else "")

        ax.set_xlim(-6, 6)
        ax.set_ylim(-9, 1)
        ax.set_title("TurtleBot Figure-8 Path Tracking")
        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.grid(True)
        ax.legend()

        while True:
            if len(self.x_data) > 1:
                traj_line.set_data(self.x_data, self.y_data)
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    controller = follow8()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
