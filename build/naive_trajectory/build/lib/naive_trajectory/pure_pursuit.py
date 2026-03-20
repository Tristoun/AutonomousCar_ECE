#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray, Int32
from visualization_msgs.msg import Marker
import tf2_ros
import math
import time

class PurePursuitHybrid(Node):
    def __init__(self):
        super().__init__('pure_pursuit_hybrid')

        # --- CONFIG MOTEURS ---
        self.PWM_FORWARD = 100
        self.PWM_TURN    = 115

        # --- CONFIG PURE PURSUIT ---
        self.LOOKAHEAD_DIST  = 0.25   # adapté au path A* sous-échantillonné (~20 cm)
        self.ANGLE_THRESHOLD = 0.30   # ~17° : en dessous → tout droit, au-dessus → pivot

        # --- CONFIG RÉFLEXE LIDAR (sécurité obstacle) ---
        self.FRONT_DANGER = 0.5
        self.SCAN_WINDOW  = 40        # ±40° devant

        # --- MACHINE À ÉTATS ---
        # Identique au code naïf : une impulsion moteur puis une pause WAIT
        self.STATE_DRIVE = "DRIVE"
        self.STATE_WAIT  = "WAIT"
        self.state          = self.STATE_DRIVE
        self.state_end_time = 0.0

        # --- INDEX WAYPOINT ---
        self.waypoint_index    = 0
        self.index_initialized = False

        # --- ROS ---
        self.create_subscription(Path,      '/global_path', self.path_callback, 10)
        self.create_subscription(Int32,     '/lap_count',   self.lap_callback,  10)
        self.create_subscription(LaserScan, '/scan',        self.scan_callback, 10)
        self.motor_pub  = self.create_publisher(Int16MultiArray, '/motor_pwm',       10)
        self.marker_pub = self.create_publisher(Marker,          '/target_waypoint', 10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path        = None
        self.scan_ranges = []
        self.current_lap = 1

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("🎯 Pure Pursuit Hybride (step+wait) activé")

    # ------------------------------------------------------------------ callbacks
    def path_callback(self, msg):
        self.path = msg
        self.index_initialized = False
        self.get_logger().info(f"📍 Path reçu : {len(msg.poses)} waypoints")

    def lap_callback(self, msg):
        prev = self.current_lap
        self.current_lap = msg.data
        if prev < 2 and self.current_lap >= 2:
            self.index_initialized = False
            self.get_logger().info("🏁 Lap 2 — réinitialisation index waypoint")

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges

    # ------------------------------------------------------------------ helpers LiDAR
    def get_dist(self, idx):
        """Moyenne robuste sur ±5 points autour de l'index — identique au code naïf."""
        if not self.scan_ranges:
            return 10.0
        n      = len(self.scan_ranges)
        points = [self.scan_ranges[i % n] for i in range(idx - 5, idx + 5)]
        valid  = [p for p in points if 0.05 < p < 10.0]
        return sum(valid) / len(valid) if valid else 10.0

    def is_obstacle_ahead(self):
        """Vérifie la zone frontale ±SCAN_WINDOW degrés."""
        if not self.scan_ranges:
            return False
        n = len(self.scan_ranges)
        for i in range(-self.SCAN_WINDOW, self.SCAN_WINDOW):
            d = self.scan_ranges[i % n]
            if 0.05 < d < self.FRONT_DANGER:
                return True
        return False

    # ------------------------------------------------------------------ helpers TF / waypoint
    def get_robot_pose(self):
        try:
            t   = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except Exception:
            return None

    def reset_index_to_nearest(self, rx, ry):
        if self.path is None:
            return
        best_idx, best_dist = 0, float('inf')
        for i, p in enumerate(self.path.poses):
            d = math.hypot(p.pose.position.x - rx, p.pose.position.y - ry)
            if d < best_dist:
                best_dist, best_idx = d, i
        self.waypoint_index    = best_idx
        self.index_initialized = True
        self.get_logger().info(
            f"🔍 Index → waypoint {best_idx}/{len(self.path.poses)} (dist={best_dist:.2f} m)"
        )

    def find_target_waypoint(self, rx, ry):
        if self.path is None:
            return None
        poses = self.path.poses
        n     = len(poses)
        if n == 0:
            return None
        for _ in range(n):
            idx  = self.waypoint_index % n
            p    = poses[idx]
            dist = math.hypot(p.pose.position.x - rx, p.pose.position.y - ry)
            if dist >= self.LOOKAHEAD_DIST:
                return (p.pose.position.x, p.pose.position.y)
            self.waypoint_index += 1
        return None

    def send_motors(self, left, right):
        self.motor_pub.publish(Int16MultiArray(data=[int(left), int(right)]))

    # ------------------------------------------------------------------ boucle principale
    def control_loop(self):
        if self.current_lap < 2 or self.path is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # -- Init de l'index au premier cycle du lap 2 --
        if not self.index_initialized:
            pose = self.get_robot_pose()
            if pose is None:
                return
            self.reset_index_to_nearest(pose[0], pose[1])
            return

        # -- Gestion de la pause WAIT (identique au code naïf) --
        if self.state == self.STATE_WAIT:
            if now < self.state_end_time:
                self.send_motors(0, 0)
                return
            else:
                self.state = self.STATE_DRIVE

        # -- Lecture de la pose --
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # -- Priorité 1 : réflexe obstacle (logique naïve LiDAR) --
        if self.is_obstacle_ahead():
            self.get_logger().warn("⚠️ Obstacle — pivot réflexe")
            d_left  = self.get_dist(55)
            d_right = self.get_dist(405)
            if d_left > d_right:
                self.send_motors(-self.PWM_TURN, self.PWM_TURN)
            else:
                self.send_motors(self.PWM_TURN, -self.PWM_TURN)
            time.sleep(0.2)                        # impulsion courte — toléré comme dans le naïf
            self.send_motors(0, 0)
            self.state          = self.STATE_WAIT
            self.state_end_time = now + 0.8        # pause stabilisation SLAM
            return

        # -- Priorité 2 : calcul de l'angle vers le waypoint --
        target = self.find_target_waypoint(rx, ry)
        if target is None:
            self.get_logger().warn("Aucun waypoint trouvé.")
            self.send_motors(0, 0)
            return

        self._publish_marker(target[0], target[1])

        dx    = target[0] - rx
        dy    = target[1] - ry
        alpha = math.atan2(dy, dx) - ryaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))   # normalise [-π, π]

        # -- Décision : même mécanique step+wait que le naïf --
        if abs(alpha) > self.ANGLE_THRESHOLD:
            # Virage nécessaire → impulsion + pause (comme le naïf)
            if alpha > 0:
                self.send_motors(-self.PWM_TURN, self.PWM_TURN)
            else:
                self.send_motors(self.PWM_TURN, -self.PWM_TURN)
            time.sleep(0.2)
            self.send_motors(0, 0)
            self.state          = self.STATE_WAIT
            self.state_end_time = now + 0.8
        else:
            # Aligné → avance tout droit (même PWM que le naïf)
            self.send_motors(self.PWM_FORWARD, self.PWM_FORWARD)

    # ------------------------------------------------------------------ marqueur Rviz
    def _publish_marker(self, x, y):
        m = Marker()
        m.header.frame_id    = "map"
        m.header.stamp       = self.get_clock().now().to_msg()
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = x
        m.pose.position.y    = y
        m.pose.position.z    = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.a = 1.0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitHybrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_motors(0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()