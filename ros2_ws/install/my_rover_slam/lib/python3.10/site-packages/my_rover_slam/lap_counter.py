#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tf2_ros
import math
from slam_toolbox.srv import Pause

class LapManager(Node):
    def __init__(self):
        super().__init__('lap_manager')
        
        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Service SLAM
        self.pause_slam_client = self.create_client(Pause, '/slam_toolbox/pause_new_measurements')
        
        # Publisher & Timer
        self.lap_pub = self.create_publisher(Int32, '/lap_count', 10)
        self.create_timer(0.1, self.check_lap)
        
        # Variables d'état
        self.lap_count = 1
        self.has_left_start = False
        self.slam_paused = False
        
        # Position de départ
        self.start_x = None
        self.start_y = None
        
        # Configuration des seuils
        self.DIST_PARTIR = 1.0  # Distance min pour valider le départ (m)
        self.DIST_ARRIVER = 0.9 # Rayon de la ligne d'arrivée (m)
        
        self.get_logger().info("🏁 Lap Manager prêt. En attente du SLAM...")

    def trigger_slam_pause(self):
        """ Appelle le service pour stopper la mise à jour de la carte """
        if self.slam_paused:
            return
            
        if not self.pause_slam_client.service_is_ready():
            self.get_logger().warn("⏳ Service Pause SLAM indisponible...")
            return

        req = Pause.Request()
        future = self.pause_slam_client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            future.result()
            self.slam_paused = True
            self.get_logger().info("🛑 SLAM FIGÉ : Mode Course activé (Localisation seule).")
        except Exception as e:
            self.get_logger().error(f"❌ Échec de la mise en pause du SLAM : {e}")

    def check_lap(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'laser_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF indisponible : {e}", throttle_duration_sec=2.0)
            return

        # Init : on attend que le robot ait bougé un minimum avant de fixer le départ
        # Évite de fixer (0,0) avant que le SLAM ait convergé
        if self.start_x is None:
            # On ne fixe le départ que si la position semble stabilisée (non nulle)
            if abs(x) < 0.01 and abs(y) < 0.01:
                self.get_logger().info("⏳ En attente de position SLAM valide...", 
                                    throttle_duration_sec=1.0)
                return
            self.start_x, self.start_y = x, y
            self.get_logger().info(f"📍 Ligne de départ fixée : [{x:.2f}, {y:.2f}]")
            return

        dist = math.hypot(x - self.start_x, y - self.start_y)

        self.get_logger().info(
            f"dist={dist:.2f} | has_left={self.has_left_start} | lap={self.lap_count}",
            throttle_duration_sec=0.5
        )

        if not self.has_left_start and dist > self.DIST_PARTIR:
            self.has_left_start = True
            self.get_logger().info("🚀 Départ validé !")

        elif self.has_left_start and dist < self.DIST_ARRIVER:
            self.lap_count += 1
            self.has_left_start = False
            self.get_logger().info(f"🏆 PASSAGE AU TOUR {self.lap_count} !")
            if self.lap_count >= 2:
                self.trigger_slam_pause()

        self.lap_pub.publish(Int32(data=self.lap_count))

def main():
    rclpy.init()
    node = LapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()