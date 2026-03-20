#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class ImuOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry')

        self.vx = self.vy = 0.0
        self.x  = self.y  = self.yaw = 0.0
        self.last_t   = None
        self.last_tf_t = 0.0
        self.TF_RATE  = 20.0

        self.bias_ax = self.bias_ay = 0.0
        self.bias_samples = []
        self.bias_ready   = False
        self.BIAS_N = 100

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ← topic corrigé
        self.sub = self.create_subscription(Imu, '/imu_data', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom_imu', 10)

        self.get_logger().info('imu_odometry démarré sur /imu_data — calibration...')

    def quat_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny, cosy)

    def is_quat_valid(self, q):
        """Retourne False si le quaternion contient NaN ou est dénormalisé."""
        if any(math.isnan(v) for v in [q.x, q.y, q.z, q.w]):
            return False
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        return 0.99 < norm < 1.01

    def cb(self, msg: Imu):
        # Validation quaternion en priorité — on ignore le message si NaN
        if not self.is_quat_valid(msg.orientation):
            return

        # Calibration biais accéléromètre
        if not self.bias_ready:
            self.bias_samples.append((
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
            ))
            if len(self.bias_samples) >= self.BIAS_N:
                arr = np.array(self.bias_samples)
                self.bias_ax = float(np.mean(arr[:, 0]))
                self.bias_ay = float(np.mean(arr[:, 1]))
                self.bias_ready = True
                self.get_logger().info(
                    f'Biais : ax={self.bias_ax:.4f} ay={self.bias_ay:.4f}'
                )
            return

        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = now
            return
        dt = now - self.last_t
        if dt <= 0.0 or dt > 0.5:
            self.last_t = now
            return
        self.last_t = now

        self.yaw = self.quat_to_yaw(msg.orientation)

        ax_b = msg.linear_acceleration.x - self.bias_ax
        ay_b = msg.linear_acceleration.y - self.bias_ay
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        ax_w = ax_b * c - ay_b * s
        ay_w = ax_b * s + ay_b * c

        DEAD = 0.04
        if abs(ax_w) < DEAD: ax_w = 0.0
        if abs(ay_w) < DEAD: ay_w = 0.0

        self.vx += ax_w * dt
        self.vy += ay_w * dt
        self.vx *= 0.98
        self.vy *= 0.98
        self.x  += self.vx * dt
        self.y  += self.vy * dt

        stamp = msg.header.stamp

        # TF throttlée à 20 Hz
        if (now - self.last_tf_t) >= (1.0 / self.TF_RATE):
            self.last_tf_t = now
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = stamp
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id  = 'base_link'
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation      = msg.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        # Odométrie à 100 Hz
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation = msg.orientation
        odom.pose.covariance[0]    = 0.5
        odom.pose.covariance[7]    = 0.5
        odom.pose.covariance[35]   = 0.02
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.linear.y  = self.vy
        odom.twist.twist.angular.z = msg.angular_velocity.z
        odom.twist.covariance[0]   = 0.1
        odom.twist.covariance[7]   = 0.1
        odom.twist.covariance[35]  = 0.01
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()