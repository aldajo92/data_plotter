import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import threading
import time


class DataPlotter(Node):
    def __init__(self):
        super().__init__('imu_and_twist_plotter_timer')

        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.twist_sub = self.create_subscription(Twist, '/vel_from_i2c', self.twist_callback, 10)

        # Shared state
        self.latest_imu_accel_x = 0.0
        self.latest_twist_linear_x = 0.0

        self.timestamps = []
        self.imu_accel_x_data = []
        self.twist_linear_x_data = []

        self.start_time = time.time()
        self.lock = threading.Lock()

        # Buffer time window (in seconds)
        self.window_size = 10.0

        # Timer for data update
        self.timer = self.create_timer(0.1, self.update_chart)  # 10 Hz

        # Plotting thread
        self.plot_thread = threading.Thread(target=self.plot_data)
        self.plot_thread.start()

    def imu_callback(self, msg: Imu):
        with self.lock:
            self.latest_imu_accel_x = msg.linear_acceleration.x

    def twist_callback(self, msg: Twist):
        with self.lock:
            self.latest_twist_linear_x = msg.linear.x

    def update_chart(self):
        now = time.time() - self.start_time

        with self.lock:
            # Append new values
            self.timestamps.append(now)
            self.imu_accel_x_data.append(self.latest_imu_accel_x)
            self.twist_linear_x_data.append(self.latest_twist_linear_x)

            # Remove data older than window size
            while self.timestamps and (now - self.timestamps[0] > self.window_size):
                self.timestamps.pop(0)
                self.imu_accel_x_data.pop(0)
                self.twist_linear_x_data.pop(0)

    def plot_data(self):
        plt.ion()
        fig, ax = plt.subplots()
        line_imu, = ax.plot([], [], label='IMU Accel X', color='blue')
        line_twist, = ax.plot([], [], label='Twist Linear X', color='red')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Value')
        ax.set_title('IMU Accel X and Twist Linear X')
        ax.legend()

        while rclpy.ok():
            with self.lock:
                line_imu.set_xdata(self.timestamps)
                line_imu.set_ydata(self.imu_accel_x_data)
                line_twist.set_xdata(self.timestamps)
                line_twist.set_ydata(self.twist_linear_x_data)
                ax.relim()
                ax.autoscale_view()

            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = DataPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
