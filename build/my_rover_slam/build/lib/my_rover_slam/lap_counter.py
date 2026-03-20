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
        self.DIST_PARTIR = 2.0  # Distance min pour valider le départ (m)
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
            # Récupération position robot
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
        except:
            return 

        # 1. Init du point de départ
        if self.start_x is None:
            self.start_x, self.start_y = x, y
            self.get_logger().info(f"📍 Ligne de départ fixée : [{x:.2f}, {y:.2f}]")
            return

        # 2. Calcul distance
        dist = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)

        # 3. Logique de franchissement
        # On quitte la zone
        if not self.has_left_start and dist > self.DIST_PARTIR:
            self.has_left_start = True
            self.get_logger().info("🚀 Départ validé !")

        # On revient dans la zone (Tour terminé)
        elif self.has_left_start and dist < self.DIST_ARRIVER:
            self.lap_count += 1
            self.has_left_start = False # Reset pour le prochain tour éventuel
            
            self.get_logger().info(f"🏆 PASSAGE AU TOUR {self.lap_count} !")
            
            # Si on passe au tour 2, on fige le SLAM
            if self.lap_count >= 2:
                self.trigger_slam_pause()
        
        # Publication du tour
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