# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import math
# import matplotlib.pyplot as plt


# class Robot:
#     def __init__(self, node, robot_id, pose_topic, odom_topic, cmd_topic, init_x, init_y, color, ax, targets, cc, r_min, r_max):
#         self.node = node
#         self.robot_id = robot_id
#         self.pose_topic = pose_topic
#         self.odom_topic = odom_topic
#         self.cmd_topic = cmd_topic
#         self.init_x = init_x
#         self.init_y = init_y
#         self.color = color
#         self.ax = ax

#         self.sim_dt = 0.001
#         self.control_dt = 0.02

#         self.v_nominal = 0.11
#         self.v_min = 0.04
#         self.v_max = 0.15
#         self.w_max = 2.0

#         self.lookahead_dist = 0.30
#         self.heading_gain_max = 3.3
#         self.heading_gain_steepness = 0.9
#         self.heading_decay_gain = 2.0

#         self.r_min = r_min
#         self.r_max = r_max

#         self.cc = cc
#         self.targets = targets
#         self.theoritical_target = 0 if self.robot_id == "RB1" else 1
#         self.current_target = self.theoritical_target

#         self.last_switched = -1000
#         self.count = 0

#         self.current_x = init_x
#         self.current_y = init_y
#         self.current_yaw = math.pi / 2 if self.robot_id == "RB1" else -math.pi / 2

#         self.current_vel = 0.0

#         self.initialized = False
#         self.last_lookahead_index = 0

#         self.theory_x = []
#         self.theory_y = []
#         self.real_x = []
#         self.real_y = []

#         self.lookahead_x = 0.0
#         self.lookahead_y = 0.0

#         self.theoretical_x = init_x
#         self.theoretical_y = init_y
#         self.theoretical_theta = math.pi / 2 if self.robot_id == "RB1" else -math.pi / 2
#         self.theoretical_omega = 0.0

#         qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.cmd_pub = node.create_publisher(Twist, self.cmd_topic, 10)
#         node.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, qos)
#         node.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
#         node.create_timer(self.control_dt, self.control_loop)

#         self.real_path_plot, = ax.plot([], [], f'{color}-', label=f'{robot_id} Real')
#         self.lookahead_plot, = ax.plot([], [], f'{color}o', label=f'{robot_id} Lookahead')
#         self.theory_path_plot, = ax.plot([], [], f'{color}--', label=f'{robot_id} Theory')

#     def pose_callback(self, msg):
#         self.current_x = msg.pose.position.x
#         self.current_y = msg.pose.position.y
#         q = msg.pose.orientation
#         self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
#         self.real_x.append(self.current_x)
#         self.real_y.append(self.current_y)
#         if not self.initialized:
#             self.generate_theoretical_path()

#     def odom_callback(self, msg):
#         self.current_vel = msg.twist.twist.linear.x

#     def generate_theoretical_path(self):
#         for _ in range(200000):
#             self.update_theoretical_state()
#             self.theory_x.append(self.theoretical_x)
#             self.theory_y.append(self.theoretical_y)

#         self.theory_path_plot.set_data(self.theory_x, self.theory_y)
#         self.ax.relim()
#         self.ax.autoscale_view()
#         plt.draw()
#         plt.pause(0.001)
#         self.initialized = True

#     def update_theoretical_state(self):
#         v = self.v_nominal
#         r = math.hypot(self.theoretical_x - self.targets[self.theoritical_target][0],
#                        self.theoretical_y - self.targets[self.theoritical_target][1])
#         r = max(r, 0.05)
#         theoretical_pos = complex(self.theoretical_x, self.theoretical_y)
#         distances = [abs(complex(x, y) - theoretical_pos) for (x, y) in self.targets]
#         self.count += 1

#         min_distance = float('inf')
#         second_min_distance = float('inf')
#         min_index = -1
#         second_min_index = -1

#         for i, d in enumerate(distances):
#             if d < min_distance:
#                 second_min_distance = min_distance
#                 second_min_index = min_index
#                 min_distance = d
#                 min_index = i
#             elif d < second_min_distance:
#                 second_min_distance = d
#                 second_min_index = i

#         if second_min_distance - min_distance < 0.001 and self.count - self.last_switched > 10000:
#             self.last_switched = self.count
#             old_target = self.targets[min_index]
#             new_target = self.targets[second_min_index]

#             yaw_old = math.atan2(self.theoretical_y - old_target[1], self.theoretical_x - old_target[0])
#             yaw_new = math.atan2(self.theoretical_y - new_target[1], self.theoretical_x - new_target[0])
#             delta_theta = self.normalize_angle(yaw_new - yaw_old)
#             self.theoretical_theta = self.normalize_angle(self.theoretical_theta + delta_theta)
#             self.theoretical_omega = delta_theta / self.sim_dt
#             self.theoretical_x += v * math.cos(self.theoretical_theta) * self.sim_dt
#             self.theoretical_y += v * math.sin(self.theoretical_theta) * self.sim_dt
#             self.theoritical_target = second_min_index
#         else:
#             kk = self.cc[self.theoritical_target]
#             A = (2 * kk) / (self.r_min * self.r_max)
#             B = self.v_nominal - (kk * (self.r_min + self.r_max)) / (self.r_min * self.r_max)
#             w = A + B / r
#             self.theoretical_x += v * math.cos(self.theoretical_theta) * self.sim_dt
#             self.theoretical_y += v * math.sin(self.theoretical_theta) * self.sim_dt
#             self.theoretical_theta = self.normalize_angle(self.theoretical_theta + w * self.sim_dt)
#             self.theoretical_omega = w

#     def control_loop(self):
#         if not self.initialized:
#             return

#         robot_pos = complex(self.current_x, self.current_y)
#         path = [complex(x, y) for x, y in zip(self.theory_x, self.theory_y)]

#         search_window_size = 481
#         search_start = max(self.last_lookahead_index - 10, 0)
#         search_end = min(len(path), search_start + search_window_size)
#         search_path = path[search_start:search_end]

#         lookahead_index_offset = self.find_closest_ahead_index(robot_pos, self.current_yaw, search_path, 0, self.lookahead_dist)
#         lookahead_index = search_start + lookahead_index_offset
#         self.last_lookahead_index = lookahead_index

#         lookahead_target = path[lookahead_index]
#         self.lookahead_x = lookahead_target.real
#         self.lookahead_y = lookahead_target.imag

#         angle_to_target = math.atan2(self.lookahead_y - self.current_y, self.lookahead_x - self.current_x)
#         heading_error = self.normalize_angle(angle_to_target - self.current_yaw)

#         heading_gain = self.heading_gain_max / (1 + math.exp(self.heading_gain_steepness * abs(heading_error)))
#         omega = heading_gain * heading_error
#         v_err = self.v_max * math.exp(-self.heading_decay_gain * abs(heading_error))
#         v_cmd = max(min(self.v_nominal + v_err, self.v_max), self.v_min)

#         cmd = Twist()
#         cmd.linear.x = v_cmd
#         cmd.angular.z = max(min(omega, self.w_max), -self.w_max)
#         self.cmd_pub.publish(cmd)

#         self.real_path_plot.set_data(self.real_x, self.real_y)
#         self.lookahead_plot.set_data([self.lookahead_x], [self.lookahead_y])
#         self.ax.relim()
#         self.ax.autoscale_view()
#         plt.pause(0.001)

#     @staticmethod
#     def normalize_angle(angle):
#         return math.atan2(math.sin(angle), math.cos(angle))

#     def find_closest_ahead_index(self, robot_pos, robot_yaw, path, start_index, min_dist):
#         min_distance = float('inf')
#         candidate_index = len(path) - 1

#         for i in range(start_index, len(path)):
#             pt = path[i]
#             dx = pt.real - robot_pos.real
#             dy = pt.imag - robot_pos.imag
#             angle_to_point = math.atan2(dy, dx)
#             heading_error = abs(self.normalize_angle(angle_to_point - robot_yaw))
#             dist = abs(pt - robot_pos)
#             if heading_error < math.radians(90) and dist >= min_dist and dist < min_distance:
#                 min_distance = dist
#                 candidate_index = i

#         return candidate_index


# class PurePursuitRealMulti(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_real_dual')

#         self.targets = [(0.5, 0), (-0.5, 0)]
#         self.cc = [0.3, 0.25]
#         self.r_min = 0.7
#         self.r_max = 0.9

#         plt.ion()
#         self.fig, self.ax = plt.subplots()
#         self.ax.set_title("Pure Pursuit Dual Robot")
#         self.ax.set_xlabel("X")
#         self.ax.set_ylabel("Y")
#         self.ax.axis('equal')
#         self.ax.grid(True)

#         for (tx, ty) in self.targets:
#             min_circle = plt.Circle((tx, ty), self.r_min, color='blue', fill=False, linestyle='-', linewidth=1.0)
#             max_circle = plt.Circle((tx, ty), self.r_max, color='red', fill=False, linestyle='-', linewidth=1.0)
#             self.ax.add_patch(min_circle)
#             self.ax.add_patch(max_circle)

#         self.robot1 = Robot(self, "RB1", "/vrpn_mocap/Rigid_body_001/pose", "/odom", "/cmd_vel",
#                             1.2, 0.0, 'b', self.ax, self.targets, self.cc, self.r_min, self.r_max)

#         self.robot2 = Robot(self, "RB2", "/vrpn_mocap/Rigid_body_002/pose", "/TB_01/odom", "/TB_01/cmd_vel",
#                             -1.2, 0.0, 'm', self.ax, self.targets, self.cc, self.r_min, self.r_max)

#         self.ax.legend()


# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuitRealMulti()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import math
# import numpy as np
# import random
# import threading
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# class Robot:
#     def __init__(self, parent_node, name, pose_topic, odom_topic, cmd_vel_topic, centers, start_idx, c_list):
#         self.parent_node = parent_node
#         self.name = name

#         self.r_min = 0.8
#         self.r_max = 1.0
#         self.v_nominal = 0.11
#         self.v_max = 0.15
#         self.v_min = 0.04
#         self.w_max = 2.0
#         self.sim_dt = 0.001

#         self.num_of_targets = len(centers)
#         self.centers = centers
#         self.c_list = c_list
#         self.current_center_index = start_idx
#         self.next_center_index = (start_idx + 1) % self.num_of_targets

#         self.random_idx = random.randint(0, len(self.c_list)-1)
#         self.c = self.c_list[self.random_idx]

#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
#         self.current_vel = 0.0
#         self.initialized = False
#         self.theoretical_path_computed = False
#         self.shift_target = False

#         self.theory_x = []
#         self.theory_y = []
#         self.real_path_x = []
#         self.real_path_y = []
#         self.lookahead_path_x = []
#         self.lookahead_path_y = []

#         self.last_lookahead_index = 0
#         self.lookahead_x = 0.0
#         self.lookahead_y = 0.0

#         qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )
#         self.cmd_pub = parent_node.create_publisher(Twist, cmd_vel_topic, 10)
#         self.pose_sub = parent_node.create_subscription(
#             PoseStamped, pose_topic, self.pose_callback, qos)
#         self.odom_sub = parent_node.create_subscription(
#             Odometry, odom_topic, self.odom_callback, 10)

#     def pose_callback(self, msg):
#         self.current_x = msg.pose.position.x
#         self.current_y = msg.pose.position.y
#         q = msg.pose.orientation
#         self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
#                                       1 - 2 * (q.y ** 2 + q.z ** 2))
#         self.real_path_x.append(self.current_x)
#         self.real_path_y.append(self.current_y)

#         if not self.initialized:
#             self.initialize_theoretical_state()

#     def odom_callback(self, msg):
#         self.current_vel = msg.twist.twist.linear.x

#     def initialize_theoretical_state(self):
#         self.theoretical_x = self.current_x
#         self.theoretical_y = self.current_y
#         self.theoretical_theta = self.current_yaw
#         self.initialized = True
#         self.parent_node.get_logger().info(f'Theoretical state initialized for {self.name}')
#         threading.Thread(target=self.compute_theoretical_path).start()

#     def compute_theoretical_path(self):
#         random.seed(hash(self.name) % 10000)
#         rot = 2.0
#         max_points = math.floor(rot * 100000.0 / 1.75)
#         switch_threshold = 0.001

#         for _ in range(max_points):
#             current_center = self.centers[self.current_center_index]
#             next_center = self.centers[self.next_center_index]

#             dx_next = self.theoretical_x - next_center[0]
#             dy_next = self.theoretical_y - next_center[1]
#             dist_from_curr = math.hypot(self.theoretical_x - current_center[0],
#                                         self.theoretical_y - current_center[1])
#             dist_to_next = math.hypot(dx_next, dy_next)
#             dist_diff = abs(dist_from_curr - dist_to_next)

#             if dist_diff < switch_threshold and not self.shift_target:
#                 dx = self.theoretical_x - current_center[0]
#                 dy = self.theoretical_y - current_center[1]
#                 tangent_angle = math.atan2(dx, -dy)

#                 a_ = math.hypot(next_center[0] - current_center[0], next_center[1] - current_center[1])
#                 b_ = math.hypot(self.theoretical_x - next_center[0], self.theoretical_y - next_center[1])
#                 c_ = math.hypot(self.theoretical_x - current_center[0], self.theoretical_y - current_center[1])

#                 try:
#                     angle_diff = math.acos((-a_ ** 2 + c_ ** 2 + b_ ** 2) / (2 * c_ * b_))
#                 except ValueError:
#                     angle_diff = math.pi / 6

#                 self.theoretical_theta = self.normalize_angle(tangent_angle - angle_diff)
#                 random_shift = random.uniform(-math.pi/36, math.pi/36)
#                 self.theoretical_theta = self.normalize_angle(self.theoretical_theta + random_shift)

#                 self.current_center_index, self.next_center_index = self.next_center_index, (self.next_center_index + 1) % self.num_of_targets
#                 self.c = self.c_list[self.random_idx]
#                 self.shift_target = True

#             if dist_diff > switch_threshold:
#                 self.shift_target = False

#             dx = self.theoretical_x - self.centers[self.current_center_index][0]
#             dy = self.theoretical_y - self.centers[self.current_center_index][1]
#             r = max(math.hypot(dx, dy), 0.05)

#             w = (2 * self.c) / (self.r_min * self.r_max) + (self.v_nominal - (self.c * (self.r_min + self.r_max)) / (self.r_min * self.r_max)) / r
#             self.theoretical_x += self.v_nominal * math.cos(self.theoretical_theta) * self.sim_dt
#             self.theoretical_y += self.v_nominal * math.sin(self.theoretical_theta) * self.sim_dt
#             self.theoretical_theta = self.normalize_angle(self.theoretical_theta + w * self.sim_dt)

#             self.theory_x.append(self.theoretical_x)
#             self.theory_y.append(self.theoretical_y)

#         self.theoretical_path_computed = True
#         self.parent_node.get_logger().info(f'Theoretical path computed for {self.name}')

#     def control_loop(self):
#         if not self.theoretical_path_computed:
#             return

#         robot_pos = complex(self.current_x, self.current_y)
#         path = [complex(x, y) for x, y in zip(self.theory_x, self.theory_y)]

#         search_window_size = 300
#         search_start = max(self.last_lookahead_index - 10, 0)
#         search_end = min(len(path), search_start + search_window_size)
#         lookahead_index = self.find_closest_ahead_index(
#             robot_pos, self.current_yaw, path[search_start:search_end], 0, 0.25
#         ) + search_start
#         self.last_lookahead_index = lookahead_index

#         lookahead_target = path[lookahead_index]
#         self.lookahead_x = lookahead_target.real
#         self.lookahead_y = lookahead_target.imag
#         angle_to_target = math.atan2(
#             self.lookahead_y - self.current_y,
#             self.lookahead_x - self.current_x
#         )
#         heading_error = self.normalize_angle(angle_to_target - self.current_yaw)

#         heading_gain = 3.0 / (1 + math.exp(2.0 * abs(heading_error)))
#         omega = heading_gain * heading_error
#         v_err = 0.15 * math.exp(-2.0 * abs(heading_error))
#         v_cmd = max(min(self.v_nominal + v_err, self.v_max), self.v_min)

#         cmd = Twist()
#         cmd.linear.x = v_cmd
#         cmd.angular.z = max(min(omega, 2.0), -2.0)
#         self.cmd_pub.publish(cmd)

#         self.lookahead_path_x.append(self.lookahead_x)
#         self.lookahead_path_y.append(self.lookahead_y)

#     def find_closest_ahead_index(self, robot_pos, robot_yaw, path, start_index, min_dist):
#         min_distance = float('inf')
#         candidate_index = len(path) - 1

#         for i in range(start_index, len(path)):
#             pt = path[i]
#             dx = pt.real - robot_pos.real
#             dy = pt.imag - robot_pos.imag
#             angle_to_point = math.atan2(dy, dx)
#             heading_error = abs(self.normalize_angle(angle_to_point - robot_yaw))
#             dist = abs(pt - robot_pos)

#             if heading_error < math.radians(90) and dist >= min_dist and dist < min_distance:
#                 min_distance = dist
#                 candidate_index = i

#         return candidate_index

#     @staticmethod
#     def normalize_angle(angle):
#         return math.atan2(math.sin(angle), math.cos(angle))

# class PurePursuitController(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_controller')
#         self.control_dt = 0.02
#         self.centers = [(0.15, 0.0), (-0.15, 0.0)]
#         self.c_list = [0.20, 0.25, 0.30, 0.35]

#         self.robots = [
#             Robot(self, "RB1", "/vrpn_mocap/Rigid_body_001/pose", "/odom", "/cmd_vel", self.centers, 0, self.c_list),
#             Robot(self, "RB2", "/vrpn_mocap/Rigid_body_002/pose", "/TB_01/odom", "/TB_01/cmd_vel", self.centers, 1, self.c_list)
#         ]

#         self.control_timer = self.create_timer(self.control_dt, self.control_callback)
#         threading.Thread(target=self.start_plotting, daemon=True).start()

#     def control_callback(self):
#         for robot in self.robots:
#             robot.control_loop()

#     def start_plotting(self):
#         plt.ion()
#         fig, ax = plt.subplots(figsize=(8, 8))
#         ax.set_title("Real-time Robot Trajectories")
#         ax.set_xlabel("X")
#         ax.set_ylabel("Y")
#         ax.set_aspect('equal')

#         real_lines = [ax.plot([], [], label=f"{r.name} Real")[0] for r in self.robots]
#         theory_lines = [ax.plot([], [], '--', label=f"{r.name} Theory")[0] for r in self.robots]
#         lookahead_points = [ax.plot([], [], 'o', label=f"{r.name} Lookahead")[0] for r in self.robots]

#         for center in self.centers:
#             ax.add_patch(plt.Circle(center, self.robots[0].r_min, color='gray', fill=False, linestyle='--'))
#             ax.add_patch(plt.Circle(center, self.robots[0].r_max, color='black', fill=False, linestyle='-'))

#         ax.legend()

#         def update(frame):
#             for i, robot in enumerate(self.robots):
#                 real_lines[i].set_data(robot.real_path_x, robot.real_path_y)
#                 theory_lines[i].set_data(robot.theory_x, robot.theory_y)
#                 lookahead_points[i].set_data(robot.lookahead_x, robot.lookahead_y)
#             ax.relim()
#             ax.autoscale_view()
#             return real_lines + theory_lines + lookahead_points

#         ani = FuncAnimation(fig, update, interval=200)
#         plt.show(block=True)

# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuitController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import math
# import numpy as np
# import csv
# import random
# import threading

# class Robot:
#     def __init__(self, parent_node, name, pose_topic, odom_topic, cmd_vel_topic, centers, start_idx, c_list):
#         self.parent_node = parent_node
#         self.name = name

#         self.r_min = 0.8
#         self.r_max = 1.0
#         self.v_nominal = 0.11
#         self.v_max = 0.15
#         self.v_min = 0.04
#         self.w_max = 2.0
#         self.sim_dt = 0.001

#         self.num_of_targets = len(centers)

#         # Robot-specific parameters
#         self.centers = centers
#         self.c_list = c_list
#         self.current_center_index = start_idx
#         self.next_center_index = (start_idx + 1) % self.num_of_targets

#         self.random_idx = random.randint(0, len(self.c_list)-1)
#         self.c = self.c_list[self.random_idx]

#         # State management
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_yaw = 0.0
#         self.current_vel = 0.0
#         self.initialized = False
#         self.theoretical_path_computed = False
#         self.shift_target = False

#         # Path tracking
#         self.theory_x = []
#         self.theory_y = []
#         self.last_lookahead_index = 0
#         self.lookahead_x = 0.0
#         self.lookahead_y = 0.0
#         self.r_max -= 0.02
#         # Publishers and subscribers
#         qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )
#         self.cmd_pub = parent_node.create_publisher(Twist, cmd_vel_topic, 10)
#         self.pose_sub = parent_node.create_subscription(
#             PoseStamped, pose_topic, self.pose_callback, qos)
#         self.odom_sub = parent_node.create_subscription(
#             Odometry, odom_topic, self.odom_callback, 10)

#         # Initialize logs
#         self.init_csv_logs()

#     def init_csv_logs(self):
#         with open(f'real_path_{self.name}.csv', 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(['real_x', 'real_y'])
#         with open(f'lookahead_path_{self.name}.csv', 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(['lookahead_x', 'lookahead_y'])

#     def pose_callback(self, msg):
#         self.current_x = msg.pose.position.x
#         self.current_y = msg.pose.position.y
#         q = msg.pose.orientation
#         self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
#                                       1 - 2 * (q.y ** 2 + q.z ** 2))

#         # Log real position
#         with open(f'real_path_{self.name}.csv', 'a', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow([self.current_x, self.current_y])

#         if not self.initialized:
#             self.initialize_theoretical_state()

#     def odom_callback(self, msg):
#         self.current_vel = msg.twist.twist.linear.x

#     def initialize_theoretical_state(self):
#         self.theoretical_x = self.current_x
#         self.theoretical_y = self.current_y
#         self.theoretical_theta = self.current_yaw
#         self.initialized = True
#         self.parent_node.get_logger().info(f'Theoretical state initialized for {self.name}')

#         # Start theoretical path computation in separate thread
#         threading.Thread(target=self.compute_theoretical_path).start()

#     def compute_theoretical_path(self):
#         random.seed(hash(self.name)%10000)
#         rot = 2.0  # Number of rotations
#         max_points = math.floor(rot * 100000.0 / 1.75)
#         switch_threshold = 0.001

#         for _ in range(max_points):
#             current_center = self.centers[self.current_center_index]
#             next_center = self.centers[self.next_center_index]

#             # Distance calculations
#             dx_next = self.theoretical_x - next_center[0]
#             dy_next = self.theoretical_y - next_center[1]
#             dist_from_curr = math.hypot(self.theoretical_x - current_center[0],
#                                        self.theoretical_y - current_center[1])
#             dist_to_next = math.hypot(dx_next, dy_next)
#             dist_diff = abs(dist_from_curr - dist_to_next)

#             # Center switching logic
#             if dist_diff < switch_threshold and not self.shift_target:
#                 # Calculate tangent angle with random perturbation
#                 dx = self.theoretical_x - current_center[0]
#                 dy = self.theoretical_y - current_center[1]
#                 tangent_angle = math.atan2(dx, -dy)

#                 a_ = math.hypot(next_center[0] - current_center[0], next_center[1] - current_center[1])
#                 b_ = math.hypot(self.theoretical_x - next_center[0], self.theoretical_y - next_center[1])
#                 c_ = math.hypot(self.theoretical_x - current_center[0], self.theoretical_y - current_center[1])

#                 try:
#                     angle_diff = math.acos((-a_ ** 2 + c_ ** 2 + b_ ** 2) / (2 * c_ * b_))
#                 except ValueError:
#                     angle_diff = math.pi / 6

#                 #print(angle_diff*57, " ", self.theoretical_theta*57)
#                 #print(tangent_angle*57)

#                 self.theoretical_theta = self.normalize_angle(tangent_angle - angle_diff)


#                 self.theoretical_theta = self.normalize_angle(self.theoretical_theta)

#                 #print(self.theoretical_theta*57)

#                 # Switch centers and update curvature
#                 self.current_center_index, self.next_center_index = self.next_center_index, (self.next_center_index + 1) % self.num_of_targets
#                 self.c = self.c_list[self.random_idx]
#                 self.shift_target = True

#             if dist_diff > switch_threshold:
#                 self.shift_target = False

#             # Update theoretical position
#             dx = self.theoretical_x - self.centers[self.current_center_index][0]
#             dy = self.theoretical_y - self.centers[self.current_center_index][1]
#             r = max(math.hypot(dx, dy), 0.05)

#             # Velocity calculation
#             w = (2 * self.c) / (self.r_min* self.r_max) + (self.v_nominal - (self.c * (self.r_min + self.r_max)) / (self.r_min * self.r_max)) / r

#             self.theoretical_x += self.v_nominal * math.cos(self.theoretical_theta) * self.sim_dt
#             self.theoretical_y += self.v_nominal * math.sin(self.theoretical_theta) * self.sim_dt
#             self.theoretical_theta = self.normalize_angle(self.theoretical_theta + w * self.sim_dt)

#             self.theory_x.append(self.theoretical_x)
#             self.theory_y.append(self.theoretical_y)

#         # Save theoretical path
#         with open(f'theory_path_{self.name}.csv', 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(['theory_x', 'theory_y'])
#             for x, y in zip(self.theory_x, self.theory_y):
#                 writer.writerow([x, y])

#         self.theoretical_path_computed = True
#         self.parent_node.get_logger().info(f'Theoretical path computed for {self.name}')

#     def control_loop(self):
#         if not self.theoretical_path_computed:
#             return

#         robot_pos = complex(self.current_x, self.current_y)
#         path = [complex(x, y) for x, y in zip(self.theory_x, self.theory_y)]

#         # Find lookahead point
#         search_window_size = 300
#         search_start = max(self.last_lookahead_index - 10, 0)
#         search_end = min(len(path), search_start + search_window_size)
#         lookahead_index = self.find_closest_ahead_index(
#             robot_pos, self.current_yaw, path[search_start:search_end], 0, 0.25
#         ) + search_start
#         self.last_lookahead_index = lookahead_index

#         # Calculate heading error
#         lookahead_target = path[lookahead_index]
#         self.lookahead_x = lookahead_target.real
#         self.lookahead_y = lookahead_target.imag
#         angle_to_target = math.atan2(
#             self.lookahead_y - self.current_y,
#             self.lookahead_x - self.current_x
#         )
#         heading_error = self.normalize_angle(angle_to_target - self.current_yaw)

#         # Control calculations
#         heading_gain = 3.0 / (1 + math.exp(2.0 * abs(heading_error)))
#         omega = heading_gain * heading_error
#         v_err = 0.15 * math.exp(-2.0 * abs(heading_error))
#         v_cmd = max(min(self.v_nominal + v_err, self.v_max), self.v_min)

#         # Publish command
#         cmd = Twist()
#         cmd.linear.x = v_cmd
#         cmd.angular.z = max(min(omega, 2.0), -2.0)
#         self.cmd_pub.publish(cmd)

#         # Log lookahead point
#         with open(f'lookahead_path_{self.name}.csv', 'a', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow([self.lookahead_x, self.lookahead_y])

#     def find_closest_ahead_index(self, robot_pos, robot_yaw, path, start_index, min_dist):
#         min_distance = float('inf')
#         candidate_index = len(path) - 1

#         for i in range(start_index, len(path)):
#             pt = path[i]
#             dx = pt.real - robot_pos.real
#             dy = pt.imag - robot_pos.imag
#             angle_to_point = math.atan2(dy, dx)
#             heading_error = abs(self.normalize_angle(angle_to_point - robot_yaw))
#             dist = abs(pt - robot_pos)

#             if (heading_error < math.radians(90) and
#                 dist >= min_dist and
#                 dist < min_distance):
#                 min_distance = dist
#                 candidate_index = i

#         return candidate_index

#     @staticmethod
#     def normalize_angle(angle):
#         return math.atan2(math.sin(angle), math.cos(angle))

# class PurePursuitController(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit_controller')

#         # Control parameters
#         self.control_dt = 0.02

#         self.centers =  [(0.15, 0.0), (-0.15, 0.0)]
#         self.c_list = [0.40, 0.45, 0.30, 0.35]

#         # Create robots
#         self.robots = [
#             Robot(
#                 self, "RB1",
#                 pose_topic="/vrpn_mocap/Rigid_body_001/pose",
#                 odom_topic="/odom",
#                 cmd_vel_topic="/cmd_vel",
#                 centers=self.centers,
#                 start_idx = 0,
#                 c_list=self.c_list
#             ),
#             Robot(
#                 self, "RB2",
#                 pose_topic="/vrpn_mocap/Rigid_body_002/pose",
#                 odom_topic="/TB_01/odom",
#                 cmd_vel_topic="/TB_01/cmd_vel",
#                 centers=self.centers,
#                 start_idx = 1,
#                 c_list=self.c_list
#             )
#         ]

#         # Control timer
#         self.control_timer = self.create_timer(self.control_dt, self.control_callback)

#     def control_callback(self):
#         # Run control loop for all robots
#         for robot in self.robots:
#             robot.control_loop()

# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuitController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np
import csv
import random
import threading

class Robot:
    def __init__(self, parent_node, name, pose_topic, odom_topic, cmd_vel_topic, centers, c_list):
        self.parent_node = parent_node
        self.name = name

        self.r_min = 0.8                                                                                                                                                                      
        self.r_max = 1.0                                                                                                                                                                      
        self.v_nominal = 0.11
        self.v_max = 0.15
        self.v_min = 0.04
        self.w_max = 2.0
        self.sim_dt = 0.001

        self.centers = centers
        self.c_list = c_list

        self.circle_count = 0
        self.c = self.c_list[self.circle_count % len(self.c_list)]

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_vel = 0.0
        self.initialized = False
        self.theoretical_path_computed = False
        self.theory_x = []
        self.theory_y = []
        self.last_lookahead_index = 0
        self.lookahead_x = 0.0
        self.lookahead_y = 0.0
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cmd_pub = parent_node.create_publisher(Twist, cmd_vel_topic, 10)
        self.pose_sub = parent_node.create_subscription(PoseStamped, pose_topic, self.pose_callback, qos)
        self.odom_sub = parent_node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.init_csv_logs()

    def init_csv_logs(self):
        with open(f'real_path_{self.name}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['real_x', 'real_y'])
        with open(f'lookahead_path_{self.name}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['lookahead_x', 'lookahead_y'])

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y ** 2 + q.z ** 2))

        with open(f'real_path_{self.name}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.current_x, self.current_y])

        if not self.initialized:
            self.initialize_theoretical_state()

    def odom_callback(self, msg):
        self.current_vel = msg.twist.twist.linear.x

    def initialize_theoretical_state(self):
        self.theoretical_x = self.current_x
        self.theoretical_y = self.current_y
        self.theoretical_theta = self.current_yaw
        self.initialized = True

        distances = [math.hypot(self.theoretical_x - cx, self.theoretical_y - cy) for (cx, cy) in self.centers]
        self.theoritical_target = int(np.argmin(distances))
        #add shift
        self.theoretical_x = self.theoretical_x + 0.00*((self.centers[self.theoritical_target][0]-self.theoretical_x)/abs(self.centers[self.theoritical_target][0]-self.theoretical_x))
        self.theoretical_y = self.theoretical_y + 0.00*((self.centers[self.theoritical_target][1]-self.theoretical_y)/abs(self.centers[self.theoritical_target][1]-self.theoretical_y))
        self.parent_node.get_logger().info(f'Theoretical state initialized for {self.name} at center index {self.theoritical_target}')
        threading.Thread(target=self.compute_theoretical_path).start()

    def compute_theoretical_path(self):
        self.theory_x = []
        self.theory_y = []
        self.last_switched = -1000
        self.count = 0

        for _ in range(200000):
            v = self.v_nominal
            r = math.hypot(self.theoretical_x - self.centers[self.theoritical_target][0],
                           self.theoretical_y - self.centers[self.theoritical_target][1])
            r = max(r, 0.05)
            theoretical_pos = complex(self.theoretical_x, self.theoretical_y)
            distances = [abs(complex(x, y) - theoretical_pos) for (x, y) in self.centers]

            self.count += 1
            min_distance = float('inf')
            second_min_distance = float('inf')
            min_index = -1
            second_min_index = -1

            for i, d in enumerate(distances):
                if d < min_distance:
                    second_min_distance = min_distance
                    second_min_index = min_index
                    min_distance = d
                    min_index = i
                elif d < second_min_distance:
                    second_min_distance = d
                    second_min_index = i

            if second_min_distance - min_distance < 0.001 and self.count - self.last_switched > 10000:
                self.last_switched = self.count
                old_target = self.centers[min_index]
                new_target = self.centers[second_min_index]
                self.circle_count = np.random.randint(0, len(self.c_list))
                self.c = self.c_list[self.circle_count % len(self.c_list)]

                yaw_old = math.atan2(self.theoretical_y - old_target[1], self.theoretical_x - old_target[0])
                yaw_new = math.atan2(self.theoretical_y - new_target[1], self.theoretical_x - new_target[0])
                delta_theta = self.normalize_angle(yaw_new - yaw_old)
                self.theoretical_theta = self.normalize_angle(self.theoretical_theta + delta_theta)
                self.theoretical_x += v * math.cos(self.theoretical_theta) * self.sim_dt
                self.theoretical_y += v * math.sin(self.theoretical_theta) * self.sim_dt
                self.theoritical_target = second_min_index
            else:
                A = (2 * self.c) / (self.r_min * self.r_max)
                B = self.v_nominal - (self.c * (self.r_min + self.r_max)) / (self.r_min * self.r_max)
                w = A + B / r
                self.theoretical_x += v * math.cos(self.theoretical_theta) * self.sim_dt
                self.theoretical_y += v * math.sin(self.theoretical_theta) * self.sim_dt
                self.theoretical_theta = self.normalize_angle(self.theoretical_theta + w * self.sim_dt)

            self.theory_x.append(self.theoretical_x)
            self.theory_y.append(self.theoretical_y)

        with open(f'theory_path_{self.name}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['theory_x', 'theory_y'])
            for x, y in zip(self.theory_x, self.theory_y):
                writer.writerow([x, y])

        self.theoretical_path_computed = True
        self.parent_node.get_logger().info(f'Theoretical path computed for {self.name}')

    def control_loop(self):
        if not self.theoretical_path_computed:
            return

        robot_pos = complex(self.current_x, self.current_y)
        path = [complex(x, y) for x, y in zip(self.theory_x, self.theory_y)]

        search_window_size = 250
        search_start = max(self.last_lookahead_index - 10, 0)
        search_end = min(len(path), search_start + search_window_size)
        lookahead_index = self.find_closest_ahead_index(
            robot_pos, self.current_yaw, path[search_start:search_end], 0, 0.25
        ) + search_start
        self.last_lookahead_index = lookahead_index

        lookahead_target = path[lookahead_index]
        self.lookahead_x = lookahead_target.real
        self.lookahead_y = lookahead_target.imag
        angle_to_target = math.atan2(
            self.lookahead_y - self.current_y,
            self.lookahead_x - self.current_x
        )
        heading_error = self.normalize_angle(angle_to_target - self.current_yaw)

        heading_gain = 3.5 / (1 + math.exp(1.2 * abs(heading_error)))
        omega = heading_gain * heading_error
        v_err = 0.15 * math.exp(-2.0 * abs(heading_error))
        v_cmd = max(min(self.v_nominal + v_err, self.v_max), self.v_min)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = max(min(omega, 2.0), -2.0)
        self.cmd_pub.publish(cmd)

        with open(f'lookahead_path_{self.name}.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.lookahead_x, self.lookahead_y])

    def find_closest_ahead_index(self, robot_pos, robot_yaw, path, start_index, min_dist):
        min_distance = float('inf')
        candidate_index = len(path) - 1

        for i in range(start_index, len(path)):
            pt = path[i]
            dx = pt.real - robot_pos.real
            dy = pt.imag - robot_pos.imag
            angle_to_point = math.atan2(dy, dx)
            heading_error = abs(self.normalize_angle(angle_to_point - robot_yaw))
            dist = abs(pt - robot_pos)

            if heading_error < math.radians(90) and dist >= min_dist and dist < min_distance:
                min_distance = dist
                candidate_index = i

        return candidate_index

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.control_dt = 0.02
        self.centers = [(0.15, 0.0), (-0.15, 0.0)]
        self.c_list = [0.25, 0.28, 0.30, 0.35]

        self.robots = [
            Robot(self, "RB1", "/vrpn_mocap/Rigid_body_001/pose", "/odom", "/cmd_vel", self.centers, self.c_list),
            Robot(self, "RB2", "/vrpn_mocap/Rigid_body_002/pose", "/TB_01/odom", "/TB_01/cmd_vel", self.centers, self.c_list)
        ]

        self.control_timer = self.create_timer(self.control_dt, self.control_callback)

    def control_callback(self):
        for robot in self.robots:
            robot.control_loop()

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
