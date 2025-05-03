import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np


class ImuPolarPlotter(Node):
    def __init__(self):
        super().__init__('imu_polar_plotter')

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.latest_yaw = 0.0  # Radians

        # Plot setup
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, polar=True)
        self.arrow, = self.ax.plot([0, 0], [0, 1], linewidth=2, color='blue')

        self.ax.set_ylim(0, 1)
        self.ax.set_theta_zero_location("N")  # North at top
        self.ax.set_theta_direction(-1)       # Clockwise (compass style)

        # Timer to update chart
        self.timer = self.create_timer(0.1, self.update_plot)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        yaw = self.compute_yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.latest_yaw = -yaw  # Invert yaw to match compass-style rotation

    def compute_yaw_from_quaternion(self, x, y, z, w):
        """
        Compute the yaw (Z-axis rotation) from a quaternion using only numpy.
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def update_plot(self):
        self.arrow.set_data([0, self.latest_yaw], [0, 1])
        yaw_deg = np.degrees(self.latest_yaw)
        self.ax.set_title(f"IMU Yaw Angle (Polar View) {yaw_deg:.1f}°")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = ImuPolarPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
