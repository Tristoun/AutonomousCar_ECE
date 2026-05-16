#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray, Int32
import tf2_ros
import math
import time

class FastPurePursuit(Node):
    def __init__(self):
        super().__init__('fast_pure_pursuit')

        # --- PARAMÈTRES ---
        self.BASE_PWM  = 110
        self.PWM_TURN  = 115

        # Lookahead
        self.LOOKAHEAD = 0.8

        # Sécurité LiDAR
        self.FRONT_DANGER   = 0.7
        self.SIDE_THRESHOLD = 0.55

        # Machine à états (inspirée du naif)
        self.STATE_DRIVE = "DRIVE"
        self.STATE_WAIT  = "WAIT"
        self.STATE_NAIF  = "NAIF"   # Reprise du mode naif si pas de path / obstacle
        self.state = self.STATE_NAIF
        self.state_end_time = 0.0

        # Frames
        self.ROBOT_FRAME = 'laser_link'
        self.MAP_FRAME   = 'map'

        # --- ROS ---
        self.create_subscription(Path,      '/global_path', self.path_callback, 10)
        self.create_subscription(Int32,     '/lap_count',   self.lap_callback,  10)
        self.create_subscription(LaserScan, '/scan',        self.scan_callback, 10)
        self.motor_pub = self.create_publisher(Int16MultiArray, '/motor_pwm', 10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path         = None
        self.ranges       = []
        self.current_lap  = 1
        self.target_index = 0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🏎️ Pure Pursuit + fallback naif activé")

    # ------------------------------------------------------------------ #
    #  CALLBACKS                                                           #
    # ------------------------------------------------------------------ #

    def path_callback(self, msg):
        self.path = msg
        self.target_index = 0
        self.state = self.STATE_DRIVE
        self.get_logger().info(f"📍 Path reçu : {len(msg.poses)} waypoints")

    def lap_callback(self, msg):
        self.current_lap = msg.data

    def scan_callback(self, msg):
        self.ranges = msg.ranges

    # ------------------------------------------------------------------ #
    #  LIDAR HELPERS                                                       #
    # ------------------------------------------------------------------ #

    def get_dist(self, idx):
        if not self.ranges:
            return 10.0
        points = [self.ranges[i % len(self.ranges)] for i in range(idx - 5, idx + 5)]
        valid  = [p for p in points if 0.05 < p < 10.0]
        return sum(valid) / len(valid) if valid else 10.0

    def get_front_dist(self):
        return self.get_dist(0)

    # ------------------------------------------------------------------ #
    #  POSE ROBOT                                                          #
    # ------------------------------------------------------------------ #

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.MAP_FRAME, self.ROBOT_FRAME, rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return None

    # ------------------------------------------------------------------ #
    #  PURE PURSUIT                                                        #
    # ------------------------------------------------------------------ #

    def find_lookahead_waypoint(self, rx, ry):
        if not self.path:
            return None

        best_dist = float('inf')
        nearest_idx = self.target_index

        search_range = range(
            max(0, self.target_index - 10),
            min(len(self.path.poses), self.target_index + 60)
        )
        for i in search_range:
            p = self.path.poses[i].pose.position
            d = math.hypot(p.x - rx, p.y - ry)
            if d < best_dist:
                best_dist = d
                nearest_idx = i

        for i in range(nearest_idx, len(self.path.poses)):
            p = self.path.poses[i].pose.position
            if math.hypot(p.x - rx, p.y - ry) >= self.LOOKAHEAD:
                self.target_index = i
                return p

        # Fin du path → boucle
        self.target_index = 0
        return self.path.poses[-1].pose.position

    # ------------------------------------------------------------------ #
    #  MODE NAIF (fallback)                                                #
    # ------------------------------------------------------------------ #

    def run_naif(self, now):
        """Comportement naif identique au CorridorStepMaster."""
        if self.state == self.STATE_WAIT:
            if now < self.state_end_time:
                self.send_motors(0, 0)
                return
            else:
                self.state = self.STATE_NAIF

        d_front = self.get_front_dist()
        d_left  = self.get_dist(60)
        d_right = self.get_dist(408)

        if d_front < self.FRONT_DANGER or abs(d_right - d_left) > self.SIDE_THRESHOLD:
            if d_left > d_right:
                l_pwm, r_pwm = -self.PWM_TURN, self.PWM_TURN
            else:
                l_pwm, r_pwm = self.PWM_TURN, -self.PWM_TURN

            self.send_motors(l_pwm, r_pwm)
            time.sleep(0.2)
            self.send_motors(0, 0)
            self.state = self.STATE_WAIT
            self.state_end_time = now + 0.8
        else:
            self.send_motors(self.BASE_PWM, self.BASE_PWM)

    # ------------------------------------------------------------------ #
    #  BOUCLE PRINCIPALE                                                   #
    # ------------------------------------------------------------------ #

    def control_loop(self):
        if not self.ranges:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Lap 1 ou pas de path → mode naif
        if self.current_lap < 2 or self.path is None:
            self.run_naif(now)
            return

        # --- SÉCURITÉ LIDAR (priorité absolue) ---
        d_front = self.get_front_dist()
        d_left  = self.get_dist(60)
        d_right = self.get_dist(408)

        if d_front < self.FRONT_DANGER or abs(d_right - d_left) > self.SIDE_THRESHOLD:
            # Obstacle détecté pendant pure pursuit → bascule naif temporairement
            self.get_logger().warn("⚠️ Obstacle pendant pursuit → naif", throttle_duration_sec=1.0)
            self.run_naif(now)
            return

        # Attente post-pivot
        if self.state == self.STATE_WAIT:
            if now < self.state_end_time:
                self.send_motors(0, 0)
                return
            else:
                self.state = self.STATE_DRIVE

        # --- PURE PURSUIT ---
        pose = self.get_robot_pose()
        if not pose:
            # Pas de TF → naif
            self.run_naif(now)
            return

        rx, ry, ryaw = pose
        target = self.find_lookahead_waypoint(rx, ry)
        if not target:
            self.run_naif(now)
            return

        angle_to_target = math.atan2(target.y - ry, target.x - rx)
        error_angle = math.atan2(
            math.sin(angle_to_target - ryaw),
            math.cos(angle_to_target - ryaw)
        )

        # Vitesse adaptative selon l'angle
        turn_factor   = max(0.5, 1.0 - abs(error_angle) / math.pi)
        speed         = int(self.BASE_PWM * turn_factor)
        speed         = max(100, speed)

        steering      = int(50.0 * error_angle)
        l_pwm = max(100, min(150, speed - steering))
        r_pwm = max(100, min(150, speed + steering))

        self.get_logger().info(
            f"🎯 err={math.degrees(error_angle):.1f}° | "
            f"L={l_pwm} R={r_pwm} | wp={self.target_index}/{len(self.path.poses)}",
            throttle_duration_sec=0.3
        )

        self.send_motors(l_pwm, r_pwm)

    def send_motors(self, l, r):
        self.motor_pub.publish(Int16MultiArray(data=[int(l), int(r)]))


def main():
    rclpy.init()
    rclpy.spin(FastPurePursuit())
    rclpy.shutdown()

if __name__ == '__main__':
    main()