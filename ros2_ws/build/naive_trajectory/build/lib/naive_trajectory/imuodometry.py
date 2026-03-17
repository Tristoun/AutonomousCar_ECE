# imu_odometry_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

class ImuOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry')
        self.sub = self.create_subscription(Imu, '/imu_data', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom_imu', 10)
        self.vx = self.vy = 0.0
        self.x  = self.y  = self.yaw = 0.0
        self.last_t = None

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = now
            return
        dt = now - self.last_t
        self.last_t = now

        # Orientation quaternion → yaw
        q = msg.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.yaw = np.arctan2(siny, cosy)

        # Intégration acc (corps → monde)
        ax_w = msg.linear_acceleration.x * np.cos(self.yaw) \
             - msg.linear_acceleration.y * np.sin(self.yaw)
        ay_w = msg.linear_acceleration.x * np.sin(self.yaw) \
             + msg.linear_acceleration.y * np.cos(self.yaw)

        # Seuil anti-bruit (dead-band)
        THRESHOLD = 0.05  # m/s²
        if abs(ax_w) < THRESHOLD: ax_w = 0.0
        if abs(ay_w) < THRESHOLD: ay_w = 0.0

        self.vx += ax_w * dt
        self.vy += ay_w * dt
        self.x  += self.vx * dt
        self.y  += self.vy * dt

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.angular.z = msg.angular_velocity.z
        self.pub.publish(odom)

def main():
    rclpy.init()
    rclpy.spin(ImuOdometry())