#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class ImuPseudoOdomNode(Node):
    def __init__(self):
        super().__init__("imu_pseudo_odom")

        self.create_subscription(Int16MultiArray, '/motor_pwm', self.pwm_callback, 10)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.left_pwm, self.right_pwm = 0, 0
        self.imu_angular_vel_z = 0.0
        self.last_time = self.get_clock().now()

        # On garde une constante approximative, le SLAM corrigera l'erreur
        self.K_V = 0.003  
        
        self.create_timer(0.05, self.update_odom)
        self.get_logger().info("🧭 Odométrie IMU + Pseudo-Translation active")

    def pwm_callback(self, msg):
        if len(msg.data) >= 2:
            self.left_pwm, self.right_pwm = msg.data[0], msg.data[1]

    def imu_callback(self, msg):    
        self.imu_angular_vel_z = msg.angular_velocity.z

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 1. ROTATION : 100% IMU (Trés précis)
        w = self.imu_angular_vel_z

        # 2. TRANSLATION : Heuristique basique
        # Si les PWM sont très faibles, on force la vitesse à 0 pour éviter la dérive
        if abs(self.left_pwm) < 20 and abs(self.right_pwm) < 20:
            v = 0.0
        else:
            # Sinon on donne une estimation grossière, le Lidar corrigera
            v = -((self.left_pwm + self.right_pwm) / 2.0 * self.K_V)
        
        # 3. Intégration
        self.x += (v * math.cos(self.th)) * dt
        self.y += (v * math.sin(self.th)) * dt
        self.th += w * dt

        # 4. Envoi TF & Odom
        q_z = math.sin(self.th / 2.0)
        q_w = math.cos(self.th / 2.0)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = q_z, q_w
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q_z, q_w
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)
        self.odom_pub.publish(odom)

def main():
    rclpy.init(); rclpy.spin(ImuPseudoOdomNode()); rclpy.shutdown()

if __name__ == '__main__':
    main()