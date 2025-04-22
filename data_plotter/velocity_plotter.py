import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading
import time


class VelocityPlotter(Node):
    def __init__(self):
        super().__init__('velocity_plotter_node')

        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.twist_sub = self.create_subscription(Twist, '/vel_from_i2c', self.twist_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Shared latest values
        self.latest_imu_accel_x = 0.0
        self.latest_twist_linear_x = 0.0
        self.latest_odom_linear_x = 0.0

        # Data buffers
        self.timestamps = []
        self.imu_accel_x_data = []
        self.twist_linear_x_data = []
        self.odom_linear_x_data = []

        self.start_time = time.time()
        self.lock = threading.Lock()

        # Plot window duration
        self.window_size = 20.0  # seconds

        # Timer to update chart
        self.timer = self.create_timer(0.05, self.update_chart)

        # Start plotting thread
        self.plot_thread = threading.Thread(target=self.plot_data)
        self.plot_thread.start()

    def imu_callback(self, msg: Imu):
        with self.lock:
            self.latest_imu_accel_x = msg.linear_acceleration.x

    def twist_callback(self, msg: Twist):
        with self.lock:
            self.latest_twist_linear_x = msg.linear.x

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.latest_odom_linear_x = msg.twist.twist.linear.x

    def update_chart(self):
        now = time.time() - self.start_time

        with self.lock:
            self.timestamps.append(now)
            self.imu_accel_x_data.append(self.latest_imu_accel_x)
            self.twist_linear_x_data.append(self.latest_twist_linear_x)
            self.odom_linear_x_data.append(self.latest_odom_linear_x)

            # Keep only data within the time window
            while self.timestamps and (now - self.timestamps[0] > self.window_size):
                self.timestamps.pop(0)
                self.imu_accel_x_data.pop(0)
                self.twist_linear_x_data.pop(0)
                self.odom_linear_x_data.pop(0)

    def plot_data(self):
        plt.ion()
        fig, ax = plt.subplots()
        line_imu, = ax.plot([], [], label='IMU Accel X', color='blue')
        line_twist, = ax.plot([], [], label='Twist Linear X', color='red')
        line_odom, = ax.plot([], [], label='Odometry Linear X', color='green')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Value')
        ax.set_title('IMU Accel X, Twist Linear X, and Odometry Linear X')
        ax.legend()

        while rclpy.ok():
            with self.lock:
                line_imu.set_xdata(self.timestamps)
                line_imu.set_ydata(self.imu_accel_x_data)

                line_twist.set_xdata(self.timestamps)
                line_twist.set_ydata(self.twist_linear_x_data)

                line_odom.set_xdata(self.timestamps)
                line_odom.set_ydata(self.odom_linear_x_data)

                ax.relim()
                ax.autoscale_view()

            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
