#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
import numpy as np


class TeleopPWMControl(Node):

    def __init__(self):
        super().__init__("teleop_pwm_control")

        # ===================== PARAMETERS =====================
        self.MAX_PWM = 110
        self.MAX_LINEAR_PWM = 100      # Forward max power
        self.MAX_TURN_PWM = 80         # Turning influence
        self.DEADBAND = 0.05           # Ignore tiny joystick noise

        # ===================== ROS INTERFACE =====================
        self.teleop_sub = self.create_subscription(
            Twist,
            "/teleop",      # Set this to your Foxglove topic
            self.teleop_callback,
            10
        )

        self.motor_pub = self.create_publisher(
            Int16MultiArray,
            "/motor_pwm",
            10
        )

        self.get_logger().info("🎮 Teleop PWM Control Ready")

    # ==========================================================
    # TELEOP CALLBACK
    # ==========================================================
    def teleop_callback(self, msg: Twist):

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

        # Clamp for safety
        left_pwm = int(np.clip(left_pwm, -self.MAX_PWM, self.MAX_PWM))
        right_pwm = int(np.clip(right_pwm, -self.MAX_PWM, self.MAX_PWM))

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
