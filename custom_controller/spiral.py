# # import rclpy
# # from rclpy.node import Node
# # from geometry_msgs.msg import Twist
# # from nav_msgs.msg import Odometry

# # class spiral(Node):
# #     def __init__(self):
# #         super().__init__("spiral")
# #         self.get_logger().info("Node Started")
# #         self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)  #create publisher to give velocity commands
# #         self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10) #create subscriber to get position via odometry

# #         self.timer = self.create_timer(0.1, self.control_loop)
# #         self.get_logger().info("MoveToNode started")
# #         self.pose = {'x': 0.0, 'y': 0.0}

# #     def odom_callback(self, msg: Odometry): #update self.pose
# #         self.pose['x'] = msg.pose.pose.position.x
# #         self.pose['y'] = msg.pose.pose.position.y


# #     def control_loop(self):
# #         twist = Twist()
# #         twist.linear.x = 0.1
# #         twist.angular.z = 0.2*((self.pose['x']-0.0)**2 + (self.pose['y']-0.0)**2)**0.5
# #         self.cmd_vel_pub.publish(twist)


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = spiral()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()


# # if __name__ == "__main__":
# #     main()
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import math
# import matplotlib.pyplot as plt

# class Spiral(Node):
#     def __init__(self):
#         super().__init__("spiral")
#         self.get_logger().info("Spiral node started")

#         # Actual pose and trajectory
#         self.pose = {'x': 0.0, 'y': 0.0}
#         self.trajectory_x = []
#         self.trajectory_y = []

#         # Theoretical pose and trajectory
#         self.t_pose = {'x': 0.0, 'y': 0.0}
#         self.theoretical_angle = 0.0
#         self.theoretical_x = []
#         self.theoretical_y = []

#         # ROS setup
#         self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
#         self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
#         self.create_timer(0.1, self.control_loop)
#         self.create_timer(0.1, self.update_theory)
#         self.create_timer(0.2, self.plot_trajectory)  # Plot update timer

#         # Setup plot
#         plt.ion()
#         self.fig, self.ax = plt.subplots()
#         self.ax.set_title("Real-Time Robot vs Theoretical Trajectory")
#         self.ax.set_xlabel("X")
#         self.ax.set_ylabel("Y")
#         self.ax.grid(True)
#         self.ax.set_xlim(-5, 5)
#         self.ax.set_ylim(-5, 5)

#         self.actual_line, = self.ax.plot([], [], 'b-', label='Actual')
#         self.theory_line, = self.ax.plot([], [], 'r--', label='Theoretical')
#         self.ax.legend()

#     def odom_callback(self, msg: Odometry):
#         self.pose['x'] = msg.pose.pose.position.x
#         self.pose['y'] = msg.pose.pose.position.y
#         self.trajectory_x.append(self.pose['x'])
#         self.trajectory_y.append(self.pose['y'])

#     def control_loop(self):
#         twist = Twist()
#         twist.linear.x = 0.1
#         radius = ((self.pose['x']-0.0)**2 + (self.pose['y']-0.55)**2)**0.5
#         twist.angular.z = -0.2 * radius
#         self.cmd_vel_pub.publish(twist)

#     def update_theory(self):
#         dt = 0.1
#         radius = ((self.t_pose['x']-0.0)**2 + (self.t_pose['y']-0.55)**2)**0.5
#         self.t_pose['x'] += 0.1 * math.cos(self.theoretical_angle) * dt
#         self.t_pose['y'] += 0.1 * math.sin(self.theoretical_angle) * dt
#         self.theoretical_angle += -0.2 * radius * dt

#         self.theoretical_x.append(self.t_pose['x'])
#         self.theoretical_y.append(self.t_pose['y'])

#     def plot_trajectory(self):
#         if len(self.trajectory_x) > 1:
#             self.actual_line.set_xdata(self.trajectory_x)
#             self.actual_line.set_ydata(self.trajectory_y)
#             self.theory_line.set_xdata(self.theoretical_x)
#             self.theory_line.set_ydata(self.theoretical_y)

#             self.ax.relim()
#             self.ax.autoscale_view()
#             self.fig.canvas.draw()
#             self.fig.canvas.flush_events()

# def main(args=None):
#     rclpy.init(args=args)
#     node = Spiral()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class Spiral(Node):
    def __init__(self):
        super().__init__('spiral')

        # ROS subscribers and publishers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Motion parameters
        self.r1 = 0.35 #0.55
        self.d = 0.50 #0.75
        self.v = 0.11
        self.omega = 0.32 #0.2

        # State tracking
        self.current_state = 'MOVING'
        self.current_circle = 1

        # Trajectories
        self.theoretical_x = []
        self.theoretical_y = []
        self.theoretical_angle = 0.0
        self.theoretical_x_cur = 0.0
        self.theoretical_y_cur = 0.0
        self.theory_circle = 1

        self.current_pose = None

        # Real trajectory
        self.x_vals = []
        self.y_vals = []

        # Plot Setup
        plt.ion()
        self.fig, (self.ax_traj, self.ax_yaw_time) = plt.subplots(2, 1, figsize=(8, 10))

        self.line, = self.ax_traj.plot([], [], 'b-', label='Trajectory')
        self.theory_line, = self.ax_traj.plot([], [], 'r--', label='Theoretical Trajectory')
        self.ax_traj.set_title('Live Robot Trajectory')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True)
        self.ax_traj.set_aspect('equal', adjustable='datalim')
        self.ax_traj.legend()

        self.time_elapsed = 0.0
        self.time_list = []
        self.real_yaw_list = []
        self.theory_yaw_list = []

        self.line_real_yaw_time, = self.ax_yaw_time.plot([], [], 'g-', label='Real Yaw (°)')
        self.line_theory_yaw_time, = self.ax_yaw_time.plot([], [], 'r--', label='Theoretical Yaw (°)')
        self.ax_yaw_time.set_title('Yaw vs Time')
        self.ax_yaw_time.set_xlabel('Time (s)')
        self.ax_yaw_time.set_ylabel('Angle (°)')
        self.ax_yaw_time.grid(True)
        self.ax_yaw_time.legend()

        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.1, self.update_theoretical_position)

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        self.x_vals.append(x)
        self.y_vals.append(y)
        self.update_plot()

    def update_plot(self):
        self.line.set_data(self.x_vals, self.y_vals)
        self.theory_line.set_data(self.theoretical_x, self.theoretical_y)
        self.ax_traj.relim()
        self.ax_traj.autoscale_view()

        self.line_real_yaw_time.set_data(self.time_list, self.real_yaw_list)
        self.line_theory_yaw_time.set_data(self.time_list, self.theory_yaw_list)
        self.ax_yaw_time.relim()
        self.ax_yaw_time.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def calculate_distance(self, center_y):
        if not self.current_pose:
            return float('inf')
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        return math.sqrt(x**2 + (y - center_y)**2)

    def get_target_center(self):
        return (0.0, self.r1) if self.current_circle == 1 else (0.0, self.r1 + self.d)

    def control_loop(self):
        if not self.current_pose:
            return

        target_center_y = self.get_target_center()[1]
        current_dist = self.calculate_distance(target_center_y)

        if self.current_state == 'MOVING':
            twist = Twist()
            twist.linear.x = self.v
            twist.angular.z = self.omega * current_dist
            self.cmd_vel_pub.publish(twist)

            if self.current_circle == 1:
                dist_r1 = self.calculate_distance(self.r1)
                dist_r1_d = self.calculate_distance(self.r1 + self.d)
            else:
                dist_r1 = self.calculate_distance(self.r1)
                dist_r1_d = self.calculate_distance(self.r1 + self.d)

            self.get_logger().info(
                f"[MOVING] Circle: {self.current_circle}, "
                f"Distance to r1: {dist_r1:.3f}, "
                f"Distance to r1+d: {dist_r1_d:.3f}"
            )

            # Switch condition: other is closer
            if (self.current_circle == 1 and dist_r1_d < dist_r1) or \
            (self.current_circle == 2 and dist_r1 < dist_r1_d):
                self.current_state = 'STOPPING'


        elif self.current_state == 'STOPPING':
            self.cmd_vel_pub.publish(Twist())
            self.target_orientation = self.calculate_new_orientation()
            self.current_state = 'ROTATING'
            self.get_logger().info(
                f"[STOPPING ➝ ROTATING] Real Yaw: {math.degrees(self.get_yaw_from_pose(self.current_pose)):.2f}°, "
                f"Target Yaw: {math.degrees(self.target_orientation):.2f}°"
            )

        elif self.current_state == 'ROTATING':
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            if abs(current_yaw - self.target_orientation) < 0.05:
                self.current_circle = 2 if self.current_circle == 1 else 1
                self.current_state = 'MOVING'
            else:
                twist = Twist()
                twist.angular.z = -0.25
                self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # def calculate_new_orientation(self):
    #     dist1 = self.calculate_distance(self.r1)
    #     dist2 = self.calculate_distance(self.r1 + self.d)
    #     nearest_center = (0.0, self.r1) if dist1 < dist2 else (0.0, self.r1 + self.d)
    #     farther_center = (0.0, self.r1 + self.d) if dist1 >= dist2 else (0.0, self.r1)

    #     dx = self.current_pose.position.x - nearest_center[0]
    #     dy = self.current_pose.position.y - nearest_center[1]
    #     dx_old = self.current_pose.position.x - farther_center[0]
    #     dy_old = self.current_pose.position.y - farther_center[1]

    #     old_angle = math.atan2(dy_old, dx_old)
    #     current_angle = self.get_yaw_from_pose(self.current_pose)
    #     raw_tangent_angle = math.atan2(dy, dx) + (current_angle - old_angle)

    #     return self.normalize_angle(raw_tangent_angle)

    def calculate_new_orientation(self):
        # Determine distances to both centers
        dist_r1 = self.calculate_distance(self.r1)
        dist_r2 = self.calculate_distance(self.r1 + self.d)

        # Identify which is current and which was previous
        if dist_r1 < dist_r2:
            new_center = (0.0, self.r1)
            old_center = (0.0, self.r1 + self.d)
        else:
            new_center = (0.0, self.r1 + self.d)
            old_center = (0.0, self.r1)

        # Vector from center to robot (old and new)
        dx_new = self.current_pose.position.x - new_center[0]
        dy_new = self.current_pose.position.y - new_center[1]

        dx_old = self.current_pose.position.x - old_center[0]
        dy_old = self.current_pose.position.y - old_center[1]

        # Angle of radius from old center to robot (i.e., direction robot used to be moving along)
        old_radial_angle = math.atan2(dy_old, dx_old)

        # Robot's current yaw
        current_yaw = self.get_yaw_from_pose(self.current_pose)

        # Relative difference between actual yaw and old radial angle
        # This tells us how the robot was moving tangentially
        relative_tangent_angle = self.normalize_angle(current_yaw - old_radial_angle)

        # Now, compute new radial angle
        new_radial_angle = math.atan2(dy_new, dx_new)

        # Apply the same relative tangent offset to the new radial
        tangent_angle = new_radial_angle + relative_tangent_angle

        return self.normalize_angle(tangent_angle)



    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    def update_theoretical_position(self):
        if not hasattr(self, 'theoretical_state'):
            self.theoretical_state = 'MOVING'
            self.theoretical_circle = 1
            self.theoretical_x_cur = 0.0
            self.theoretical_y_cur = 0.0
            self.target_theory_angle = 0.0

        cx, cy = (0.0, self.r1) if self.theoretical_circle == 1 else (0.0, self.r1 + self.d)

        if self.theoretical_state == 'MOVING':
            dx = self.v * 0.1 * math.cos(self.theoretical_angle)
            dy = self.v * 0.1 * math.sin(self.theoretical_angle)

            self.theoretical_x_cur += dx
            self.theoretical_y_cur += dy
            self.theoretical_angle += self.omega* ((self.theoretical_x_cur - cx)**2+(self.theoretical_y_cur - cy)**2)**0.5 * 0.1
            self.theoretical_angle = self.normalize_angle(self.theoretical_angle)

            self.theoretical_x.append(self.theoretical_x_cur)
            self.theoretical_y.append(self.theoretical_y_cur)

            other_cy = self.r1 + self.d if self.theoretical_circle == 1 else self.r1
            dist_cur = math.sqrt(self.theoretical_x_cur**2 + (self.theoretical_y_cur - cy)**2)
            dist_other = math.sqrt(self.theoretical_x_cur**2 + (self.theoretical_y_cur - other_cy)**2)

            if dist_other < dist_cur: 
                self.theoretical_state = 'STOPPING'

        elif self.theoretical_state == 'STOPPING':
            self.theoretical_state = 'ROTATING'

            # Find both centers
            center1 = (0.0, self.r1)
            center2 = (0.0, self.r1 + self.d)

            # Distance to both centers
            dist1 = math.sqrt(self.theoretical_x_cur**2 + (self.theoretical_y_cur - center1[1])**2)
            dist2 = math.sqrt(self.theoretical_x_cur**2 + (self.theoretical_y_cur - center2[1])**2)

            # Identify new and old centers
            if dist1 < dist2:
                new_center = center1
                old_center = center2
            else:
                new_center = center2
                old_center = center1

            # Compute radial vectors
            dx_old = self.theoretical_x_cur - old_center[0]
            dy_old = self.theoretical_y_cur - old_center[1]
            dx_new = self.theoretical_x_cur - new_center[0]
            dy_new = self.theoretical_y_cur - new_center[1]

            # Old radial angle
            old_radial_angle = math.atan2(dy_old, dx_old)

            # Relative tangent direction from previous circle
            relative_tangent = self.normalize_angle(self.theoretical_angle - old_radial_angle)

            # New radial angle
            new_radial_angle = math.atan2(dy_new, dx_new)

            # Apply the relative tangent to the new radial
            self.target_theory_angle = self.normalize_angle(new_radial_angle + relative_tangent)

            self.get_logger().info(
                f"[STOPPING ➝ ROTATING] Theory Angle: {math.degrees(self.theoretical_angle):.2f}°, "
                f"Target: {math.degrees(self.target_theory_angle):.2f}°"
            )


        elif self.theoretical_state == 'ROTATING':
            self.theoretical_angle -= 0.25 * 0.1
            self.theoretical_angle = self.normalize_angle(self.theoretical_angle)

            if abs(self.theoretical_angle - self.target_theory_angle) < 0.05:
                self.theoretical_circle = 2 if self.theoretical_circle == 1 else 1
                self.theoretical_state = 'MOVING'

        self.time_elapsed += 0.1
        self.time_list.append(self.time_elapsed)
        self.theory_yaw_list.append(math.degrees(self.theoretical_angle))

        if self.current_pose:
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            self.real_yaw_list.append(math.degrees(current_yaw))
        else:
            self.real_yaw_list.append(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = Spiral()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

