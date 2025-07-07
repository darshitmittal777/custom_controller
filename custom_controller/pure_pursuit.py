# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# import math
# import numpy as np
# import matplotlib.pyplot as plt

# class pure_pursuit(Node):
#     def __init__(self):
#         super().__init__('pure_pursuit')

#         self.sim_dt = 0.001
#         self.control_dt = 0.02

#         self.v_nominal = 0.11
#         self.v_min = 0.04
#         self.v_low = 0.05
#         self.v_max = 0.15
#         self.w_max = 2.0

#         self.kappa_lookahead_gain = 6.0
#         self.lookahead_dist_min = 0.15
#         self.lookahead_dist_max = 0.35
#         self.lookahead_dist = 0.30

#         self.heading_gain_max = 3.0
#         self.heading_gain_steepness = 1.2
#         self.heading_decay_gain = 2.0

#         self.r_min = 0.7
#         self.r_max = 0.9
#         # self.c = 0.35

#         # self.r_min_theory = 1.0
#         # self.r_max_theory = 0.0
#         # self.r_max_over_time = []
#         # self.r_min_over_time = []
#         # self.radius_time = []


#         self.theoretical_x = self.r_min
#         self.theoretical_y = 0.0
#         self.theoretical_theta = math.pi / 2
#         self.theory_v = self.v_nominal
#         self.theoretical_omega = 0.0
#         self.theoritical_target = 0
#         self.last_switched = -1000
#         self.count = 0

#         self.targets = [(0.5,0),(-0.5,0)]
#         # self.targets = [(0.5,0)]
#         self.cc = [0.3,0.25]
#         self.current_target = 0

#         self.current_x = self.r_min
#         self.current_y = 0.0
#         self.current_yaw = math.pi / 2
#         self.current_vel = 0.0

#         self.initialized = False
#         self.last_lookahead_index = 0

#         self.theory_x = []
#         self.theory_y = []
#         self.real_x = []
#         self.real_y = []

#         self.lookahead_x = 0.0
#         self.lookahead_y = 0.0

#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

#         self.control_timer = self.create_timer(self.control_dt, self.control_loop)

#         # === LIVE PLOTTING SETUP ===
#         plt.ion()
#         self.fig, self.ax = plt.subplots()
#         self.real_path_plot, = self.ax.plot([], [], 'b-', label='Real Path')
#         self.lookahead_plot, = self.ax.plot([], [], 'ro', label='Lookahead Point')
#         self.theory_path_plot, = self.ax.plot([], [], 'g--', label='Theoretical Path')
#         self.ax.set_title("Pure Pursuit Live")
#         self.ax.set_xlabel("X")
#         self.ax.set_ylabel("Y")
#         self.ax.legend()
#         self.ax.axis('equal')
#         self.ax.grid(True)
#         # ===========================
#                 # === PLOT STATIC CIRCLES AROUND TARGETS ===
#         for (tx, ty) in self.targets:
#             min_circle = plt.Circle((tx, ty), self.r_min, color='blue', fill=False, linestyle='-', linewidth=1.0)
#             max_circle = plt.Circle((tx, ty), self.r_max, color='red', fill=False, linestyle='-', linewidth=1.0)
#             self.ax.add_patch(min_circle)
#             self.ax.add_patch(max_circle)




#     def odom_callback(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y

#         q = msg.pose.pose.orientation
#         self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
#                                       1 - 2 * (q.y ** 2 + q.z ** 2))

#         self.current_vel = msg.twist.twist.linear.x

#         self.real_x.append(self.current_x)
#         self.real_y.append(self.current_y)

#         if not self.initialized:
#             self.initialize_theoretical_state()

#     def initialize_theoretical_state(self):
#         self.theoretical_x = 1.2
#         self.theoretical_y = 0
#         self.theoretical_theta = math.pi/2
#         self.initialized = True
#         self.get_logger().info('Theoretical state initialized')

#         for _ in range(200000):
#             self.update_theoretical_state()
#             self.theory_x.append(self.theoretical_x)
#             self.theory_y.append(self.theoretical_y)

#         self.get_logger().info('Theoretical Calculation Completed')

#         # === PLOT THEORETICAL PATH ON SAME GRAPH ===
#         self.theory_path_plot.set_data(self.theory_x, self.theory_y)
#         self.ax.relim()
#         self.ax.autoscale_view()
#         plt.draw()
#         plt.pause(0.001)
#         # # ============================================
#         #         # === PLOT RADIUS VS TIME ===
#         # plt.figure()
#         # plt.plot(self.radius_time, self.r_max_over_time, 'r-', label='r_max_theory')
#         # plt.plot(self.radius_time, self.r_min_over_time, 'b-', label='r_min_theory')
#         # plt.title("Theoretical Radius Over Time")
#         # plt.xlabel("Time (s)")
#         # plt.ylabel("Radius (m)")
#         # plt.legend()
#         # plt.grid(True)
#         # plt.show()
#         # # ============================


#     def control_loop(self):
#         if not self.initialized:
#             return

#         robot_pos = complex(self.current_x, self.current_y)
#         path = [complex(x, y) for x, y in zip(self.theory_x, self.theory_y)]

#         search_window_size = 519  
#         search_start = max(self.last_lookahead_index - 10, 0)
#         search_end = min(len(path), search_start + search_window_size)
#         search_path = path[search_start:search_end]

#         lookahead_index_offset = self.find_closest_ahead_index(
#             robot_pos, self.current_yaw, search_path, 0, self.lookahead_dist
#         )
#         lookahead_index = search_start + lookahead_index_offset
#         self.last_lookahead_index = lookahead_index

#         lookahead_target = path[lookahead_index]
#         self.lookahead_x = lookahead_target.real
#         self.lookahead_y = lookahead_target.imag

#         angle_to_target = math.atan2(
#             self.lookahead_y - self.current_y,
#             self.lookahead_x - self.current_x
#         )
#         heading_error = self.normalize_angle(angle_to_target - self.current_yaw)

#         heading_gain = self.heading_gain_max / (1 + math.exp(
#             self.heading_gain_steepness * abs(heading_error)))
#         omega = heading_gain * heading_error

#         v_err = self.v_max * math.exp(-self.heading_decay_gain * abs(heading_error))
#         v_cmd = max(min(self.v_nominal + v_err, self.v_max), self.v_min)

#         cmd = Twist()
#         cmd.linear.x = v_cmd
#         cmd.angular.z = max(min(omega, self.w_max), -self.w_max)
#         self.cmd_pub.publish(cmd)

#         # === UPDATE LIVE PLOT ===
#         self.real_path_plot.set_data(self.real_x, self.real_y)
#         self.lookahead_plot.set_data([self.lookahead_x], [self.lookahead_y])
#         self.ax.relim()
#         self.ax.autoscale_view()
#         plt.pause(0.001)
#         # ========================


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

#     def update_theoretical_state(self):
#         v = self.theory_v
#         r = math.hypot(self.theoretical_x - self.targets[self.theoritical_target][0], self.theoretical_y - self.targets[self.theoritical_target][1])
#         # self.r_max_theory = max(self.r_max_theory,r)
#         # self.r_min_theory = min(self.r_min_theory,r)
#         # self.r_max_over_time.append(self.r_max_theory)
#         # self.r_min_over_time.append(self.r_min_theory)
#         # self.radius_time.append(len(self.radius_time) * self.sim_dt)

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

#         if second_min_distance-min_distance<0.001 and self.count-self.last_switched>10000:
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

#     @staticmethod
#     def normalize_angle(angle):
#         return math.atan2(math.sin(angle), math.cos(angle))

# def main(args=None):
#     rclpy.init(args=args)
#     node = pure_pursuit()
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt

class pure_pursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.sim_dt = 0.001
        self.control_dt = 0.02

        self.v_nominal = 0.11
        self.v_min = 0.04
        self.v_low = 0.05
        self.v_max = 0.15
        self.w_max = 2.0

        self.kappa_lookahead_gain = 6.0
        self.lookahead_dist_min = 0.15
        self.lookahead_dist_max = 0.35
        self.lookahead_dist = 0.30

        self.heading_gain_max = 3.0
        self.heading_gain_steepness = 1.2
        self.heading_decay_gain = 2.0

        self.r_min = 0.7
        self.r_max = 0.9
        # self.c = 0.35

        # self.r_min_theory = 1.0
        # self.r_max_theory = 0.0
        # self.r_max_over_time = []
        # self.r_min_over_time = []
        # self.radius_time = []


        self.theoretical_x = self.r_min
        self.theoretical_y = 0.0
        self.theoretical_theta = math.pi / 2
        self.theory_v = self.v_nominal
        self.theoretical_omega = 0.0
        self.theoritical_target = 0
        self.last_switched = -1000
        self.count = 0

        self.targets = [(0.5,0),(-0.5,0)]
        # self.targets = [(0.5,0)]
        self.cc = [0.3,0.25]
        self.current_target = 0

        self.current_x = self.r_min
        self.current_y = 0.0
        self.current_yaw = math.pi / 2
        self.current_vel = 0.0

        self.initialized = False
        self.last_lookahead_index = 0

        self.theory_x = []
        self.theory_y = []
        self.real_x = []
        self.real_y = []

        self.lookahead_x = 0.0
        self.lookahead_y = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        # === LIVE PLOTTING SETUP ===
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.real_path_plot, = self.ax.plot([], [], 'b-', label='Real Path')
        self.lookahead_plot, = self.ax.plot([], [], 'ro', label='Lookahead Point')
        self.theory_path_plot, = self.ax.plot([], [], 'g--', label='Theoretical Path')
        self.ax.set_title("Pure Pursuit Live")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.axis('equal')
        self.ax.grid(True)
        # ===========================
                # === PLOT STATIC CIRCLES AROUND TARGETS ===
        for (tx, ty) in self.targets:
            min_circle = plt.Circle((tx, ty), self.r_min, color='blue', fill=False, linestyle='-', linewidth=1.0)
            max_circle = plt.Circle((tx, ty), self.r_max, color='red', fill=False, linestyle='-', linewidth=1.0)
            self.ax.add_patch(min_circle)
            self.ax.add_patch(max_circle)




    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y ** 2 + q.z ** 2))

        self.current_vel = msg.twist.twist.linear.x

        self.real_x.append(self.current_x)
        self.real_y.append(self.current_y)

        if not self.initialized:
            self.initialize_theoretical_state()

    def initialize_theoretical_state(self):
        self.theoretical_x = 1.2
        self.theoretical_y = 0
        self.theoretical_theta = math.pi/2
        self.initialized = True
        self.get_logger().info('Theoretical state initialized')

        for _ in range(200000):
            self.update_theoretical_state()
            self.theory_x.append(self.theoretical_x)
            self.theory_y.append(self.theoretical_y)

        self.get_logger().info('Theoretical Calculation Completed')

        # === PLOT THEORETICAL PATH ON SAME GRAPH ===
        self.theory_path_plot.set_data(self.theory_x, self.theory_y)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.001)
        # # ============================================
        #         # === PLOT RADIUS VS TIME ===
        # plt.figure()
        # plt.plot(self.radius_time, self.r_max_over_time, 'r-', label='r_max_theory')
        # plt.plot(self.radius_time, self.r_min_over_time, 'b-', label='r_min_theory')
        # plt.title("Theoretical Radius Over Time")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Radius (m)")
        # plt.legend()
        # plt.grid(True)
        # plt.show()
        # # ============================


    def control_loop(self):
        if not self.initialized:
            return

        robot_pos = complex(self.current_x, self.current_y)
        path = [complex(x, y) for x, y in zip(self.theory_x, self.theory_y)]

        search_window_size = 519  
        search_start = max(self.last_lookahead_index - 10, 0)
        search_end = min(len(path), search_start + search_window_size)
        search_path = path[search_start:search_end]

        lookahead_index_offset = self.find_closest_ahead_index(
            robot_pos, self.current_yaw, search_path, 0, self.lookahead_dist
        )
        lookahead_index = search_start + lookahead_index_offset
        self.last_lookahead_index = lookahead_index

        lookahead_target = path[lookahead_index]
        self.lookahead_x = lookahead_target.real
        self.lookahead_y = lookahead_target.imag

        angle_to_target = math.atan2(
            self.lookahead_y - self.current_y,
            self.lookahead_x - self.current_x
        )
        heading_error = self.normalize_angle(angle_to_target - self.current_yaw)

        heading_gain = self.heading_gain_max / (1 + math.exp(
            self.heading_gain_steepness * abs(heading_error)))
        omega = heading_gain * heading_error

        v_err = self.v_max * math.exp(-self.heading_decay_gain * abs(heading_error))
        v_cmd = max(min(self.v_nominal + v_err, self.v_max), self.v_min)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = max(min(omega, self.w_max), -self.w_max)
        self.cmd_pub.publish(cmd)

        # === UPDATE LIVE PLOT ===
        self.real_path_plot.set_data(self.real_x, self.real_y)
        self.lookahead_plot.set_data([self.lookahead_x], [self.lookahead_y])
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)
        # ========================


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

    def update_theoretical_state(self):
        v = self.theory_v
        r = math.hypot(self.theoretical_x - self.targets[self.theoritical_target][0], self.theoretical_y - self.targets[self.theoritical_target][1])
        # self.r_max_theory = max(self.r_max_theory,r)
        # self.r_min_theory = min(self.r_min_theory,r)
        # self.r_max_over_time.append(self.r_max_theory)
        # self.r_min_over_time.append(self.r_min_theory)
        # self.radius_time.append(len(self.radius_time) * self.sim_dt)

        r = max(r, 0.05)
        theoretical_pos = complex(self.theoretical_x, self.theoretical_y)
        distances = [abs(complex(x, y) - theoretical_pos) for (x, y) in self.targets]
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

        if second_min_distance-min_distance<0.001 and self.count-self.last_switched>10000:
            self.last_switched = self.count
            old_target = self.targets[min_index]
            new_target = self.targets[second_min_index]

            yaw_old = math.atan2(self.theoretical_y - old_target[1], self.theoretical_x - old_target[0])
            yaw_new = math.atan2(self.theoretical_y - new_target[1], self.theoretical_x - new_target[0])
            delta_theta = self.normalize_angle(yaw_new - yaw_old)
            self.theoretical_theta = self.normalize_angle(self.theoretical_theta + delta_theta)
            self.theoretical_omega = delta_theta / self.sim_dt
            self.theoretical_x += v * math.cos(self.theoretical_theta) * self.sim_dt
            self.theoretical_y += v * math.sin(self.theoretical_theta) * self.sim_dt
            self.theoritical_target = second_min_index
        else:
            kk = self.cc[self.theoritical_target]
            A = (2 * kk) / (self.r_min * self.r_max)
            B = self.v_nominal - (kk * (self.r_min + self.r_max)) / (self.r_min * self.r_max)
            # w = A + B / r

            #w =  0.2976*r + -0.0815/r#quadratic
            #w =np.exp(7.053255 * r**2 + -10.028635 * r + 1) * (2 * 7.053255 + -10.028635 / r) #exp_quad
            # w = -(0.275625) / r**3 + (0.547500) / r #inverse add
            # w = (-0.032570 * 12) / r * math.cos(12 * r + -1.375222) #sin(linear) + offset
            w = (1 / r) * math.cosh(0.872663 * r**2 + -1.286687 * r + 0.55) * (2 * 0.872663 * r + -1.286687) #sinh(quadratic)
            self.theoretical_x += v * math.cos(self.theoretical_theta) * self.sim_dt
            self.theoretical_y += v * math.sin(self.theoretical_theta) * self.sim_dt
            self.theoretical_theta = self.normalize_angle(self.theoretical_theta + w * self.sim_dt)
            self.theoretical_omega = w 

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = pure_pursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
