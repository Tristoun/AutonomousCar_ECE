#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
import numpy as np


class TeleopPWMControl(Node):

    def __init__(self):
        super().__init__("teleop_pwm_control")

        # ===================== PARAMETERS =====================
        self.MAX_PWM = 100
        self.MAX_LINEAR_PWM = 100
        self.MAX_TURN_PWM = 90
        self.DEADBAND = 0.05

        self.estop_active = False

        # ===================== ROS INTERFACE =====================
        self.teleop_sub = self.create_subscription(
            Twist,
            "/teleop",
            self.teleop_callback,
            10
        )

        self.estop_sub = self.create_subscription(
            Bool,
            "/emergency_stop",
            self.estop_callback,
            10
        )

        self.motor_pub = self.create_publisher(
            Int16MultiArray,
            "/motor_pwm",
            10
        )

        self.get_logger().info("🎮 Teleop PWM Control Ready with E-STOP")

    # ==========================================================
    # EMERGENCY STOP CALLBACK
    # ==========================================================
    def estop_callback(self, msg: Bool):
        self.estop_active = msg.data

        if self.estop_active:
            self.get_logger().warn("🛑 EMERGENCY STOP ACTIVATED")
            self.publish_pwm(0, 0)
            self.estop_active = False
        else:
            self.get_logger().info("✅ Emergency Stop Released")

    # ==========================================================
    # TELEOP CALLBACK
    # ==========================================================
    def teleop_callback(self, msg: Twist):

        # 🚨 If E-STOP active → ignore all movement
        if self.estop_active:
            self.publish_pwm(0, 0)
            return

        linear = msg.linear.x
        angular = msg.angular.z

        # Deadband
        if abs(linear) < self.DEADBAND:
            linear = 0.0
        if abs(angular) < self.DEADBAND:
            angular = 0.0

        # Scale to PWM
        linear_pwm = linear * self.MAX_LINEAR_PWM
        turn_pwm = angular * self.MAX_TURN_PWM

        # Differential drive mix
        left_pwm = linear_pwm - turn_pwm
        right_pwm = linear_pwm + turn_pwm

        # Clamp
        left_pwm = int(np.clip(left_pwm, -self.MAX_PWM, self.MAX_PWM))
        right_pwm = int(np.clip(right_pwm, -self.MAX_PWM, self.MAX_PWM))
        

        left_pwm = 130
        right_pwm = -120
        self.publish_pwm(left_pwm, right_pwm)

    # ==========================================================
    def publish_pwm(self, left, right):
        msg = Int16MultiArray()
        msg.data = [left, right]
        self.motor_pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopPWMControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Teleop stopped")
    finally:
        node.publish_pwm(0, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
