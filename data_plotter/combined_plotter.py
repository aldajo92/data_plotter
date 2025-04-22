import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import threading
import time


class CombinedPlotter(Node):
    def __init__(self):
        super().__init__('combined_plotter_node')

        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.twist_sub = self.create_subscription(Twist, '/vel_from_i2c', self.twist_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # State variables
        self.latest_imu_accel_x = 0.0
        self.latest_twist_linear_x = 0.0
        self.latest_odom_linear_x = 0.0
        self.latest_yaw = 0.0

        # Data buffers
        self.timestamps = []
        self.imu_accel_x_data = []
        self.twist_linear_x_data = []
        self.odom_linear_x_data = []

        self.start_time = time.time()
        self.lock = threading.Lock()
        self.window_size = 20.0

        # Plot setup
        plt.ion()
        self.fig, (self.ax_vel, self.ax_yaw) = plt.subplots(2, 1, figsize=(8, 8))

        # Top: velocity chart
        self.line_imu, = self.ax_vel.plot([], [], label='IMU Accel X', color='blue')
        self.line_twist, = self.ax_vel.plot([], [], label='Twist Linear X', color='red')
        self.line_odom, = self.ax_vel.plot([], [], label='Odometry Linear X', color='green')
        self.ax_vel.set_xlabel('Time (s)')
        self.ax_vel.set_ylabel('Value')
        self.ax_vel.set_title('Velocity Data')
        self.ax_vel.legend()

        # Bottom: polar plot for yaw
        self.ax_yaw = self.fig.add_subplot(212, polar=True)
        self.arrow, = self.ax_yaw.plot([0, 0], [0, 1], linewidth=2, color='blue')
        self.ax_yaw.set_ylim(0, 1)
        self.ax_yaw.set_theta_zero_location("N")
        self.ax_yaw.set_theta_direction(-1)

        # Start timers
        self.timer = self.create_timer(0.05, self.update_data)
        self.plot_thread = threading.Thread(target=self.plot_loop)
        self.plot_thread.start()

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        yaw = self.compute_yaw_from_quaternion(q.x, q.y, q.z, q.w)
        with self.lock:
            self.latest_yaw = -yaw
            self.latest_imu_accel_x = msg.linear_acceleration.x

    def twist_callback(self, msg: Twist):
        with self.lock:
            self.latest_twist_linear_x = msg.linear.x

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.latest_odom_linear_x = msg.twist.twist.linear.x

    def compute_yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def update_data(self):
        now = time.time() - self.start_time
        with self.lock:
            self.timestamps.append(now)
            self.imu_accel_x_data.append(self.latest_imu_accel_x)
            self.twist_linear_x_data.append(self.latest_twist_linear_x)
            self.odom_linear_x_data.append(self.latest_odom_linear_x)

            while self.timestamps and (now - self.timestamps[0] > self.window_size):
                self.timestamps.pop(0)
                self.imu_accel_x_data.pop(0)
                self.twist_linear_x_data.pop(0)
                self.odom_linear_x_data.pop(0)

    def plot_loop(self):
        while rclpy.ok():
            with self.lock:
                self.line_imu.set_xdata(self.timestamps)
                self.line_imu.set_ydata(self.imu_accel_x_data)

                self.line_twist.set_xdata(self.timestamps)
                self.line_twist.set_ydata(self.twist_linear_x_data)

                self.line_odom.set_xdata(self.timestamps)
                self.line_odom.set_ydata(self.odom_linear_x_data)

                self.ax_vel.relim()
                self.ax_vel.autoscale_view()

                # Update yaw arrow
                self.arrow.set_data([0, self.latest_yaw], [0, 1])
                yaw_deg = np.degrees(self.latest_yaw)
                self.ax_yaw.set_title(f"IMU Yaw Angle (Polar View) {yaw_deg:.1f}°")

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
