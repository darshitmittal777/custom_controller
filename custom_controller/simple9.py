# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# import math
# import matplotlib.pyplot as plt

# class SimpleLivePlot(Node):
#     def __init__(self):
#         super().__init__('simple_live_plot')
#         self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Parameters
#         self.r1 = 1.0
#         self.d = 1.5
#         self.v = 0.18
#         self.omega = self.v / self.r1


#         self.STATES = ['MOVING', 'STOPPING', 'ROTATING']
#         self.current_state = 'MOVING'
#         self.current_circle = 1
        

#         self.current_pose = None
#         self.switch_threshold = 0.001

#         self.x_vals = []
#         self.y_vals = []

#         # Live plot setup
#         plt.ion()
#         self.fig, self.ax = plt.subplots()
#         self.line, = self.ax.plot([], [], 'b-', label='Trajectory')
#         self.ax.set_title('Live Robot Trajectory')
#         self.ax.set_xlabel('X (m)')
#         self.ax.set_ylabel('Y (m)')
#         self.ax.grid(True)
#         self.ax.set_aspect('equal', adjustable='datalim')
#         self.ax.legend()

#         self.create_timer(0.1, self.control_loop)

#     def odom_cb(self, msg):
#         self.current_pose = msg.pose.pose
#         x = self.current_pose.position.x
#         y = self.current_pose.position.y
#         self.x_vals.append(x)
#         self.y_vals.append(y)
#         self.update_plot()

#     def update_plot(self):
#         self.line.set_data(self.x_vals, self.y_vals)
#         self.ax.relim()
#         self.ax.autoscale_view()
#         self.fig.canvas.draw()
#         self.fig.canvas.flush_events()

#     def calculate_distance(self, center_y):
#         if not self.current_pose:
#             return float('inf')
#         x = self.current_pose.position.x
#         y = self.current_pose.position.y
#         return math.sqrt(x**2 + (y - center_y)**2)

#     def get_target_center(self):
#         return (0.0, self.r1) if self.current_circle == 1 else (0.0, self.r1 + self.d)

#     def control_loop(self):
#         if not self.current_pose:
#             return

#         target_center = self.get_target_center()[1]
#         current_dist = self.calculate_distance(target_center)

        
#         if self.current_state == 'MOVING':
#             twist = Twist()
#             twist.linear.x = self.v
#             twist.angular.z = self.omega
#             self.cmd_vel_pub.publish(twist)

#             other_center = self.r1 + self.d if self.current_circle == 1 else self.r1
#             other_dist = self.calculate_distance(other_center)

#             if other_dist < current_dist:
#                 self.current_state = 'STOPPING'

#         elif self.current_state == 'STOPPING':
#             self.cmd_vel_pub.publish(Twist())
#             self.target_orientation = self.calculate_new_orientation()
#             self.current_state = 'ROTATING'

#         elif self.current_state == 'ROTATING':
#             current_yaw = self.get_yaw_from_pose(self.current_pose)
#             if abs(current_yaw - self.target_orientation) < 0.05:
#                 self.current_circle = 2 if self.current_circle == 1 else 1
#                 self.current_state = 'MOVING'
#             else:
#                 twist = Twist()
#                 twist.angular.z = -0.25
#                 self.cmd_vel_pub.publish(twist)

#     def normalize_angle(self, angle):
#         return math.atan2(math.sin(angle), math.cos(angle))

#     def calculate_new_orientation(self):
#         dist_to_center1 = self.calculate_distance(self.r1)
#         dist_to_center2 = self.calculate_distance(self.r1 + self.d)

#         nearest_center = (0.0, self.r1) if dist_to_center1 < dist_to_center2 else (0.0, self.r1 + self.d)
#         dx = self.current_pose.position.x - nearest_center[0]
#         dy = self.current_pose.position.y - nearest_center[1]

#         tangent_angle = math.atan2(dy, dx) + math.pi / 2
#         return self.normalize_angle(tangent_angle)

#     def get_yaw_from_pose(self, pose):
#         q = pose.orientation
#         return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

# def main(args=None):
#     rclpy.init(args=args)
#     node = SimpleLivePlot()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cmd_vel_pub.publish(Twist())  # Stop robot
#         plt.ioff()
#         plt.show()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt
import numpy as np


class simple9(Node):
    def __init__(self):
        super().__init__('simple9')

        # ROS subscribers and publishers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Motion parameters
        self.r1 = 1.0          # Radius of both circles
        self.d = 1.5           # Distance between circle centers
        self.v = 0.18          # Linear velocity
        self.omega = self.v / self.r1  # Angular velocity for circular motion

        # State tracking for robot's real motion
        self.STATES = ['MOVING', 'STOPPING', 'ROTATING']
        self.current_state = 'MOVING'
        self.current_circle = 1  # 1 or 2 to switch between circles

        # Lists to store theoretical and real trajectory data
        self.theoretical_x = []
        self.theoretical_y = []
        self.theoretical_angle = 0.0

        self.theoretical_x_cur = 0.0
        self.theoretical_y_cur = 0.0
        self.theoretical_angle_cur = 0.0
        self.theory_circle = 1

        self.current_pose = None
        self.switch_threshold = 0.001

        # Real trajectory path
        self.x_vals = []
        self.y_vals = []

        # ------------------ Plot Setup ------------------
        theta = np.linspace(0, 2 * math.pi, 200)

        # Circle 1: Center (0,1), radius 1
        x1 = np.cos(theta)
        y1 = np.sin(theta) + 1

        # Circle 2: Center (0,2.5), radius 1
        x2 = np.cos(theta)
        y2 = np.sin(theta) + 2.5

        # Setup Matplotlib live plotting
        plt.ion()
        self.fig, (self.ax_traj, self.ax_yaw_time) = plt.subplots(2, 1, figsize=(8, 10))

        # Plot the reference circles
        self.ax_traj.plot(x1, y1, 'k--', linewidth=1, label='Circle 1')
        self.ax_traj.plot(x2, y2, 'k-.', linewidth=1, label='Circle 2')

        # Plot objects for real and theoretical trajectories
        self.line, = self.ax_traj.plot([], [], 'b-', label='Trajectory')
        self.theory_line, = self.ax_traj.plot([], [], 'r--', label='Theoretical Trajectory')

        self.ax_traj.set_title('Live Robot Trajectory')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True)
        self.ax_traj.set_aspect('equal', adjustable='datalim')
        self.ax_traj.legend()

        # --- Yaw vs Time plot setup ---
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

        # Timers for control loop and theoretical simulation (every 0.1s)
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.1, self.update_theoretical_position)

    # Callback to process odometry data and update real trajectory
    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        self.x_vals.append(x)
        self.y_vals.append(y)
        self.update_plot()

    # Function to update both trajectory and yaw vs time plots
    def update_plot(self):
        # Update trajectory subplot
        self.line.set_data(self.x_vals, self.y_vals)
        self.theory_line.set_data(self.theoretical_x, self.theoretical_y)
        self.ax_traj.relim()
        self.ax_traj.autoscale_view()

        # Update yaw vs time subplot
        self.line_real_yaw_time.set_data(self.time_list, self.real_yaw_list)
        self.line_theory_yaw_time.set_data(self.time_list, self.theory_yaw_list)
        self.ax_yaw_time.relim()
        self.ax_yaw_time.autoscale_view()

        # Render plots
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # Compute distance from current position to a given circle center
    def calculate_distance(self, center_y):
        if not self.current_pose:
            return float('inf')
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        return math.sqrt(x**2 + (y - center_y)**2)

    # Return current circle's center (used in motion logic)
    def get_target_center(self):
        return (0.0, self.r1) if self.current_circle == 1 else (0.0, self.r1 + self.d)

    # Main robot control loop with state machine
    def control_loop(self):
        if not self.current_pose:
            return

        # Determine which center we are currently tracking
        target_center_y = self.get_target_center()[1]
        current_dist = self.calculate_distance(target_center_y)

        # --- Moving state: Drive along circular trajectory ---
        if self.current_state == 'MOVING':
            twist = Twist()
            twist.linear.x = self.v
            twist.angular.z = self.omega
            self.cmd_vel_pub.publish(twist)

            # Check if it's time to switch circles
            other_center = self.r1 + self.d if self.current_circle == 1 else self.r1
            other_dist = self.calculate_distance(other_center)

            if other_dist < current_dist:
                self.current_state = 'STOPPING'

        # --- Stop and compute target rotation ---
        elif self.current_state == 'STOPPING':
            self.cmd_vel_pub.publish(Twist())  # stop robot
            self.target_orientation = self.calculate_new_orientation()
            self.current_state = 'ROTATING'
            self.get_logger().info(
                f"[STOPPING ➝ ROTATING] Real Yaw: {math.degrees(self.get_yaw_from_pose(self.current_pose)):.2f}°, "
                f"Target Yaw: {math.degrees(self.target_orientation):.2f}°"
            )

        # --- Rotate until aligned with tangent to next circle ---
        elif self.current_state == 'ROTATING':
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            if abs(current_yaw - self.target_orientation) < 0.05:
                self.current_circle = 2 if self.current_circle == 1 else 1
                self.current_state = 'MOVING'
            else:
                twist = Twist()
                twist.angular.z = -0.25
                self.cmd_vel_pub.publish(twist)

    # Normalize angle to [-π, π]
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # Compute the desired new heading after reaching intersection
    def calculate_new_orientation(self):
        dist1 = self.calculate_distance(self.r1)
        dist2 = self.calculate_distance(self.r1 + self.d)
        nearest_center = (0.0, self.r1) if dist1 < dist2 else (0.0, self.r1 + self.d)
        dx = self.current_pose.position.x - nearest_center[0]
        dy = self.current_pose.position.y - nearest_center[1]
        tangent_angle = math.atan2(dy, dx) + math.pi / 2
        return self.normalize_angle(tangent_angle)

    # Extract yaw (heading) from quaternion orientation
    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

    # Simulate theoretical motion on circular paths
    def update_theoretical_position(self):
        # Initialize if not yet defined
        if not hasattr(self, 'theoretical_state'):
            self.theoretical_state = 'MOVING'
            self.theoretical_circle = 1
            self.theoretical_x_cur = 0.0
            self.theoretical_y_cur = 0.0
            self.target_theory_angle = 0.0

        # Get center for current theoretical circle
        cx, cy = (0.0, self.r1) if self.theoretical_circle == 1 else (0.0, self.r1 + self.d)

        if self.theoretical_state == 'MOVING':
            # Move tangentially around circle
            angle = math.atan2(self.theoretical_x_cur - cx, -(self.theoretical_y_cur - cy))
            dx = self.v * 0.1 * math.cos(angle)
            dy = self.v * 0.1 * math.sin(angle)

            self.theoretical_x_cur += dx
            self.theoretical_y_cur += dy
            self.theoretical_angle += self.omega * 0.1
            self.theoretical_angle = self.normalize_angle(self.theoretical_angle)

            self.theoretical_x.append(self.theoretical_x_cur)
            self.theoretical_y.append(self.theoretical_y_cur)

            # Check if we’re closer to the other circle now
            other_cy = self.r1 + self.d if self.theoretical_circle == 1 else self.r1
            dist_cur = math.sqrt((self.theoretical_x_cur)**2 + (self.theoretical_y_cur - cy)**2)
            dist_other = math.sqrt((self.theoretical_x_cur)**2 + (self.theoretical_y_cur - other_cy)**2)

            if dist_other < dist_cur:
                self.theoretical_state = 'STOPPING'

        elif self.theoretical_state == 'STOPPING':
            # Set new tangent direction
            self.theoretical_state = 'ROTATING'
            dist1 = math.sqrt((self.theoretical_x_cur)**2 + (self.theoretical_y_cur - self.r1)**2)
            dist2 = math.sqrt((self.theoretical_x_cur)**2 + (self.theoretical_y_cur - (self.r1 + self.d))**2)
            nearest_center = (0.0, self.r1) if dist1 < dist2 else (0.0, self.r1 + self.d)
            dx = self.theoretical_x_cur - nearest_center[0]
            dy = self.theoretical_y_cur - nearest_center[1]
            tangent_angle = math.atan2(dy, dx) + math.pi / 2
            self.target_theory_angle = self.normalize_angle(tangent_angle)
            self.get_logger().info(
                f"[STOPPING ➝ ROTATING] Theory Angle: {math.degrees(self.theoretical_angle):.2f}°, "
                f"Target: {math.degrees(self.target_theory_angle):.2f}°"
            )

        elif self.theoretical_state == 'ROTATING':
            # Simulate slow rotation
            self.theoretical_angle -= 0.25 * 0.1
            self.theoretical_angle = self.normalize_angle(self.theoretical_angle)

            if abs(self.theoretical_angle - self.target_theory_angle) < 0.05:
                self.theoretical_circle = 2 if self.theoretical_circle == 1 else 1
                self.theoretical_state = 'MOVING'

        # Append yaw vs time data
        self.time_elapsed += 0.1
        self.time_list.append(self.time_elapsed)
        self.theory_yaw_list.append(math.degrees(self.theoretical_angle))

        if self.current_pose:
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            self.real_yaw_list.append(math.degrees(current_yaw))
        else:
            self.real_yaw_list.append(0.0)


# -------------------- Main Entry ---------------------
def main(args=None):
    rclpy.init(args=args)
    node = simple9()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())  # Stop the robot
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
