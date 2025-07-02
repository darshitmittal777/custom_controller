import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np
import csv

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

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

        self.heading_gain_max = 2.5
        self.heading_gain_steepness = 1.5
        self.heading_decay_gain = 2.0

        self.r_min = 0.8
        self.r_max = 1.0
        self.c = 0.35

        self.theoretical_x = self.r_min
        self.theoretical_y = 0.0
        self.theoretical_theta = math.pi / 2
        self.theory_v = self.v_nominal
        self.theoretical_omega = 0.0

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

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/vrpn_mocap/Rigid_body_001/pose', self.pose_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        self.init_csv_logs()

    def init_csv_logs(self):
        with open('real_path.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['real_x', 'real_y'])
        with open('lookahead_path.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['lookahead_x', 'lookahead_y'])

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        q = msg.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y ** 2 + q.z ** 2))

        self.real_x.append(self.current_x)
        self.real_y.append(self.current_y)

        with open('real_path.csv', 'a', newline='') as f:
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
        self.get_logger().info('Theoretical state initialized')

        for _ in range(100000):
            self.update_theoretical_state()
            self.theory_x.append(self.theoretical_x)
            self.theory_y.append(self.theoretical_y)

        with open('theory_path.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['theory_x', 'theory_y'])
            for x, y in zip(self.theory_x, self.theory_y):
                writer.writerow([x, y])

        self.get_logger().info('Theoretical Calculation Completed, starting robot')

    def control_loop(self):
        if not self.initialized:
            return

        robot_pos = complex(self.current_x, self.current_y)
        path = [complex(x, y) for x, y in zip(self.theory_x, self.theory_y)]

        # Use a forward window of points to limit search
        search_window_size = 300
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

        with open('lookahead_path.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.lookahead_x, self.lookahead_y])

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

    def find_closest_ahead_index(self, robot_pos, robot_yaw, path, start_index, min_dist):
        min_distance = float('inf')
        candidate_index = len(path) - 1  # Default to last point

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
        r = math.hypot(self.theoretical_x, self.theoretical_y)
        r = max(r, 0.05)  # Clamp to avoid division by 0

        A = (2 * self.c) / (self.r_min * self.r_max)
        B = self.v_nominal - (self.c * (self.r_min + self.r_max)) / (self.r_min * self.r_max)
        w = A + B / r

        self.theoretical_x += v * math.cos(self.theoretical_theta) * self.sim_dt
        self.theoretical_y += v * math.sin(self.theoretical_theta) * self.sim_dt
        self.theoretical_theta = self.normalize_angle(self.theoretical_theta + w * self.sim_dt)
        self.theoretical_omega = w

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

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