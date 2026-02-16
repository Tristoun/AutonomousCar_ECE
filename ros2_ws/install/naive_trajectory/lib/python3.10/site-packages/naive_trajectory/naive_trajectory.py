#!/usr/bin/env python3
"""
Autonomous Corridor Follower with Enhanced Follow-the-Gap Logic
Version 4.0 - Optimized for smooth corridor navigation
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from collections import deque


class AdaptiveCorridorFollower(Node):
    def __init__(self):
        super().__init__("adaptive_corridor_follower")
        
        # ==================== CONFIGURATION ====================
        self.HARDWARE_OFFSET = 3.14159
        
        # Vitesses adaptatives
        self.BASE_SPEED = 100
        self.MIN_SPEED = 70
        self.MAX_SPEED = 110  # LIMITE STRICTE
        self.TURN_SPEED_REDUCTION = 0.7  # Facteur de réduction en virage
        
        # Gains du contrôleur
        self.KP_STEERING = 120.0  # Gain proportionnel pour le steering
        self.KD_STEERING = 15.0   # Gain dérivé pour amortir les oscillations
        
        # ==================== NAVIGATION ====================
        self.LOOKAHEAD_DIST = 0.65  # Distance de lookahead (augmentée)
        self.MIN_LOOKAHEAD = 0.40   # Distance minimale en virage serré
        self.MAX_LOOKAHEAD = 0.70   # Distance maximale en ligne droite
        
        self.SAFETY_RADIUS = 0.35   # Rayon de sécurité autour du point cible
        self.EMERGENCY_DIST = 0.5  # Distance d'urgence
        self.CORRIDOR_WIDTH_MIN = 0.5  # Largeur minimale de couloir acceptable
        
        # Scan parameters
        self.SCAN_ANGLE = 150  # Degrés (scan total)
        self.SCAN_RESOLUTION = 41  # Nombre de rayons
        self.SCAN_DISTANCES = [0.3, 0.5, 0.7]  # Distances de scan multiples
        
        # ==================== FOLLOW THE GAP ====================
        self.GAP_THRESHOLD = 0.35  # Distance minimale pour considérer un gap
        self.GAP_MIN_WIDTH = 2  # Réduit de 3 à 2 pour accepter plus de gaps
        self.CENTER_BIAS = 0.3  # Biais vers le centre (0-1)
        
        # ==================== FILTRAGE & STABILITÉ ====================
        self.STEERING_SMOOTH_FACTOR = 0.65  # Plus élevé = plus lisse
        self.SPEED_SMOOTH_FACTOR = 0.75
        self.heading_history = deque(maxlen=5)  # Historique des directions
        
        self.WATCHDOG_TIMEOUT = 1.5
        self.MAP_TIMEOUT = 1.0  # Timeout pour la carte (plus tolérant)
        self.STUCK_THRESHOLD = 0.05  # Vitesse linéaire minimale (m/s)
        self.STUCK_TIME_LIMIT = 1.5  # Augmenté à 5s pour laisser le temps d'explorer
        
        # ==================== ÉTAT ====================
        self.map = None
        self.pose = None
        self.velocity = 0.0
        self.last_pose_time = self.get_clock().now()
        self.last_map_time = self.get_clock().now()
        self.last_position = None
        self.stuck_start_time = None
        self.no_gap_start_time = None  # Nouveau: temps quand aucun gap trouvé
        self.observation_duration = 1.0  # Durée d'observation réduite (1 seconde)
        
        self.last_steering = 0.0
        self.last_steering_rate = 0.0
        self.current_speed = self.BASE_SPEED
        self.current_lookahead = self.LOOKAHEAD_DIST
        
        # Statistiques
        self.total_distance = 0.0
        self.max_gap_width = 0
        
        # ==================== ROS2 INTERFACE ====================
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.sub_pose = self.create_subscription(
            Odometry, '/odom_rf2o', self.pose_callback, 10
        )
        
        self.motor_pub = self.create_publisher(Int16MultiArray, '/motor_pwm', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/nav_debug_markers', 10)
        
        self.timer = self.create_timer(0.08, self.control_loop)  # 12.5 Hz
        
        self.get_logger().info("🚀 Enhanced Corridor Follower Ready (V4.0)")
        self.get_logger().info(f"   Lookahead: {self.LOOKAHEAD_DIST}m | Safety: {self.SAFETY_RADIUS}m")

    # ==================== CALLBACKS ====================
    
    def map_callback(self, msg):
        """Réception de la carte d'occupation"""
        self.map = msg
        self.last_map_time = self.get_clock().now()

    def pose_callback(self, msg):
        """Réception de l'odométrie et calcul de la pose"""
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        raw_yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        
        # Calcul de la vitesse
        if self.last_position is not None:
            dx = new_x - self.last_position[0]
            dy = new_y - self.last_position[1]
            dt = 0.1  # Approximation
            self.velocity = math.sqrt(dx*dx + dy*dy) / dt
            self.total_distance += math.sqrt(dx*dx + dy*dy)
        
        self.pose = (new_x, new_y, raw_yaw)
        self.last_position = (new_x, new_y)
        self.last_pose_time = self.get_clock().now()

    # ==================== UTILITAIRES CARTE ====================
    
    def is_localization_stale(self):
        """Vérifie si la localisation est périmée"""
        now = self.get_clock().now()
        return (now - self.last_pose_time).nanoseconds / 1e9 > self.WATCHDOG_TIMEOUT
    
    def is_map_stale(self):
        """Vérifie si la carte est périmée"""
        now = self.get_clock().now()
        return (now - self.last_map_time).nanoseconds / 1e9 > self.MAP_TIMEOUT

    def get_cell_value(self, x, y):
        """Récupère la valeur d'occupation d'une cellule"""
        if not self.map:
            return 100
        
        info = self.map.info
        px = int((x - info.origin.position.x) / info.resolution)
        py = int((y - info.origin.position.y) / info.resolution)
        
        if 0 <= px < info.width and 0 <= py < info.height:
            val = self.map.data[py * info.width + px]
            # Traite l'inconnu comme partiellement libre (favorise l'exploration)
            return val if val != -1 else 20
        return 100

    def check_path_clear(self, x1, y1, x2, y2, num_checks=8):
        """Vérifie si le chemin entre deux points est libre"""
        for i in range(num_checks):
            t = i / (num_checks - 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.get_cell_value(x, y) > 50:
                return False
        return True

    def is_point_safe(self, x, y, radius=None):
        """Vérifie si un point et son voisinage sont sûrs"""
        if radius is None:
            radius = self.SAFETY_RADIUS
        
        # Check du point central
        if self.get_cell_value(x, y) > 50:
            return False
        
        # Check circulaire autour du point
        num_checks = 8
        for i in range(num_checks):
            angle = 2 * math.pi * i / num_checks
            check_x = x + radius * math.cos(angle)
            check_y = y + radius * math.sin(angle)
            if self.get_cell_value(check_x, check_y) > 50:
                return False
        
        return True

    # ==================== FOLLOW THE GAP CORE ====================
    
    def find_best_gap(self, rx, ry, heading):
        """
        Implémentation améliorée de l'algorithme Follow the Gap
        CORRECTION: Scan inversé pour correspondre à l'orientation réelle de la map
        Retourne: (best_angle, gap_quality, gap_width)
        """
        scan_angle_rad = math.radians(self.SCAN_ANGLE)
        # INVERSION: On scanne de droite à gauche pour correspondre à la map
        angles = np.linspace(scan_angle_rad/2, -scan_angle_rad/2, self.SCAN_RESOLUTION)
        
        # Scan multi-distances pour robustesse
        gap_scores = np.zeros(len(angles))
        distance_scores = []
        
        for scan_dist in self.SCAN_DISTANCES:
            dist_gaps = []
            
            for idx, angle in enumerate(angles):
                target_yaw = heading + angle
                tx = rx + scan_dist * np.cos(target_yaw)
                ty = ry + scan_dist * np.sin(target_yaw)
                
                # Vérification complète de sécurité
                is_safe = self.is_point_safe(tx, ty)
                path_clear = self.check_path_clear(rx, ry, tx, ty)
                
                # CORRECTION: Pénalité forte pour les directions vers les obstacles
                cell_value = self.get_cell_value(tx, ty)
                if cell_value > 50:
                    dist_gaps.append(-1.0)  # Pénalité forte pour obstacle
                elif is_safe and path_clear:
                    dist_gaps.append(1.0)  # Récompense pour espace libre
                else:
                    dist_gaps.append(0.0)  # Neutre
            
            distance_scores.append(dist_gaps)
        
        # Fusion des scores (moyenne pondérée favorisant les distances proches)
        weights = [0.5, 0.3, 0.2]
        for i, weight in enumerate(weights):
            gap_scores += np.array(distance_scores[i]) * weight
        
        # Identification des gaps continus
        gaps = []
        in_gap = False
        gap_start = 0
        
        for i, score in enumerate(gap_scores):
            # Un gap valide peut avoir un score moins élevé pour éviter de paniquer
            if score > 0.4 and not in_gap:  # Réduit de 0.6 à 0.4
                in_gap = True
                gap_start = i
            elif score <= 0.4 and in_gap:
                gap_width = i - gap_start
                if gap_width >= self.GAP_MIN_WIDTH:
                    gap_center = (gap_start + i - 1) / 2
                    gaps.append({
                        'start': gap_start,
                        'end': i - 1,
                        'center': gap_center,
                        'width': gap_width,
                        'angle': angles[int(gap_center)]
                    })
                in_gap = False
        
        # Dernier gap si il va jusqu'au bout
        if in_gap and (len(gap_scores) - gap_start) >= self.GAP_MIN_WIDTH:
            gap_center = (gap_start + len(gap_scores) - 1) / 2
            gaps.append({
                'start': gap_start,
                'end': len(gap_scores) - 1,
                'center': gap_center,
                'width': len(gap_scores) - gap_start,
                'angle': angles[int(gap_center)]
            })
        
        if not gaps:
            return None, 0.0, 0
        
        # Sélection du meilleur gap avec critères multiples
        best_gap = None
        best_score = -1
        
        for gap in gaps:
            # Score basé sur:
            # 1. Largeur du gap (plus large = mieux)
            # 2. Proximité avec l'axe central (CENTER_BIAS)
            # 3. Continuité avec la direction précédente
            # 4. NOUVEAU: Bonus pour les directions proches de tout droit
            
            width_score = min(gap['width'] / 10.0, 1.0)
            center_score = 1.0 - abs(gap['angle']) / (scan_angle_rad/2)
            
            # Bonus si proche de la direction précédente
            continuity_score = 1.0
            if self.heading_history:
                avg_prev = np.mean(self.heading_history)
                continuity_score = math.exp(-abs(gap['angle'] - avg_prev) / 0.5)
            
            # NOUVEAU: Bonus pour avancer tout droit (évite les virages inutiles)
            forward_bias = math.exp(-abs(gap['angle']) / 0.8)  # Fort bonus pour angle proche de 0
            
            # Score final pondéré avec biais vers l'avant
            total_score = (
                width_score * 0.3 +
                center_score * 0.2 +
                continuity_score * 0.2 +
                forward_bias * 0.3  # 30% du score pour favoriser l'avant
            )
            
            if total_score > best_score:
                best_score = total_score
                best_gap = gap
        
        self.max_gap_width = max(self.max_gap_width, best_gap['width'])
        return best_gap['angle'], best_score, best_gap['width']

    # ==================== CONTRÔLEUR PRINCIPAL ====================
    
    def control_loop(self):
        """Boucle de contrôle principale"""
        
        # ========== VÉRIFICATIONS CRITIQUES ==========
        
        # Vérification 1: Map et Pose disponibles
        if self.map is None:
            self.get_logger().warn("⏸️  Attente de la carte... ARRÊT")
            self.stop_robot()
            return
            
        if self.pose is None:
            self.get_logger().warn("⏸️  Attente de la position... ARRÊT")
            self.stop_robot()
            return
        
        # Vérification 2: Localisation périmée
        if self.is_localization_stale():
            self.get_logger().error("🛑 Localisation perdue! ARRÊT COMPLET")
            self.stop_robot()
            return
        
        # Vérification 3: Carte périmée (SLAM peut avoir des problèmes)
        if self.is_map_stale():
            self.get_logger().warn("⚠️  Carte non mise à jour depuis >3s... ARRÊT PRÉVENTIF")
            self.stop_robot()
            return
        
        rx, ry, raw_yaw = self.pose
        real_heading = raw_yaw + self.HARDWARE_OFFSET
        
        # Détection de blocage
        if self.velocity < self.STUCK_THRESHOLD:
            if self.stuck_start_time is None:
                self.stuck_start_time = self.get_clock().now()
            elif (self.get_clock().now() - self.stuck_start_time).nanoseconds / 1e9 > self.STUCK_TIME_LIMIT:
                self.get_logger().warn("⚠️ Robot bloqué! Manœuvre d'urgence...")
                self.execute_unstuck_maneuver()
                return
        else:
            self.stuck_start_time = None
        
        # ========== FOLLOW THE GAP ==========
        best_angle, gap_quality, gap_width = self.find_best_gap(rx, ry, real_heading)
        
        if best_angle is None:
            # NOUVEAU COMPORTEMENT: Exploration prudente au lieu de paniquer
            if self.no_gap_start_time is None:
                self.no_gap_start_time = self.get_clock().now()
                self.get_logger().warn("⏸️  Vision limitée - Mode exploration...")
            
            observation_elapsed = (self.get_clock().now() - self.no_gap_start_time).nanoseconds / 1e9
            
            if observation_elapsed < 1.0:  # Réduit à 1 seconde
                # Arrêt court pour stabiliser la carte
                self.get_logger().warn(f"🔍 Stabilisation... {observation_elapsed:.1f}s")
                self.stop_robot()
                self.publish_markers(rx, ry, real_heading, 0.0, is_searching=True)
                return
            else:
                # MODE EXPLORATION: Avance tout droit LENTEMENT pour mapper
                self.get_logger().info("🐢 Mode exploration: avance prudemment pour mapper...")
                self.publish_pwm(60, 60)  # Avance très lentement
                self.publish_markers(rx, ry, real_heading, 0.0, gap_quality=0.2, gap_width=0)
                return
        
        # Si on trouve un gap, on reset le timer d'observation
        self.no_gap_start_time = None
        
        # Vérification de la qualité du gap
        if gap_quality < 0.2:
            self.get_logger().warn(
                f"⚠️  Qualité de passage très faible ({gap_quality:.2f}) - Mode prudent!"
            )
            # Très prudent mais on continue quand même
            speed_penalty = 0.6
        elif gap_quality < 0.4:
            # Qualité moyenne-basse, on ralentit modérément
            speed_penalty = 0.75
        elif gap_quality < 0.6:
            # Qualité correcte
            speed_penalty = 0.9
        else:
            # Bonne qualité, vitesse normale
            speed_penalty = 1.0
        
        # DEBUG: Log de la direction choisie
        if self.get_clock().now().nanoseconds % 500_000_000 < 100_000_000:
            direction = "DROITE" if best_angle > 0 else "GAUCHE" if best_angle < 0 else "TOUT DROIT"
            self.get_logger().info(
                f"🎯 Direction: {direction} ({math.degrees(best_angle):.1f}°) | "
                f"Quality: {gap_quality:.2f} | Width: {gap_width}"
            )
        
        # ========== ADAPTATION DYNAMIQUE ==========
        
        # Ajustement du lookahead basé sur l'angle
        angle_factor = math.cos(best_angle)
        self.current_lookahead = self.MIN_LOOKAHEAD + (self.MAX_LOOKAHEAD - self.MIN_LOOKAHEAD) * angle_factor
        
        # Calcul de la commande de steering avec dérivée
        steering_error = best_angle
        steering_rate = steering_error - self.last_steering
        
        steering_command = (
            self.KP_STEERING * steering_error +
            self.KD_STEERING * steering_rate
        )
        
        # Lissage du steering
        steering_smooth = (
            self.STEERING_SMOOTH_FACTOR * self.last_steering +
            (1 - self.STEERING_SMOOTH_FACTOR) * steering_error
        )
        
        self.last_steering = steering_smooth
        self.last_steering_rate = steering_rate
        self.heading_history.append(best_angle)
        
        # ========== ADAPTATION DE VITESSE ==========
        
        # Réduction en fonction de l'angle
        turn_factor = math.cos(abs(steering_smooth))
        speed_target = self.BASE_SPEED * (
            self.TURN_SPEED_REDUCTION + (1 - self.TURN_SPEED_REDUCTION) * turn_factor
        )
        
        # Application de la pénalité de qualité
        speed_target *= speed_penalty
        
        # Léger bonus si gap large et qualité haute (mais sans dépasser MAX_SPEED)
        if gap_quality > 0.8 and gap_width > 8:
            speed_target = min(speed_target * 1.05, self.MAX_SPEED)  # +5% max
        
        # SÉCURITÉ: Limite absolue
        speed_target = min(speed_target, self.MAX_SPEED)
        
        # Lissage de la vitesse
        self.current_speed = (
            self.SPEED_SMOOTH_FACTOR * self.current_speed +
            (1 - self.SPEED_SMOOTH_FACTOR) * speed_target
        )
        
        # Limitation minimale
        self.current_speed = max(self.MIN_SPEED, self.current_speed)
        
        # ========== COMMANDE MOTEUR ==========
        
        l_pwm = self.current_speed - steering_command
        r_pwm = self.current_speed + steering_command
        
        self.publish_pwm(l_pwm, r_pwm)
        self.publish_markers(rx, ry, real_heading, steering_smooth, gap_quality, gap_width)
        
        # Logging périodique
        if self.get_clock().now().nanoseconds % 2_000_000_000 < 100_000_000:
            self.get_logger().info(
                f"📊 Gap: {math.degrees(best_angle):.1f}° | "
                f"Quality: {gap_quality:.2f} ({int(speed_penalty*100)}%) | "
                f"Width: {gap_width} | "
                f"Speed: {self.current_speed:.0f}"
            )

    # ==================== COMPORTEMENTS D'URGENCE ====================
    
    def execute_search_behavior(self):
        """Comportement de recherche quand aucun gap n'est trouvé"""
        # Rotation lente pour scanner l'environnement (respecte la limite de 110)
        self.publish_pwm(-80, 80)
    
    def execute_unstuck_maneuver(self):
        """Manœuvre pour se dégager si bloqué"""
        self.get_logger().warn("🔄 Robot bloqué - Tentative d'exploration latérale...")
        # Essai de rotation douce pour trouver un passage
        # (on ne recule plus immédiatement, on explore d'abord)
        self.publish_pwm(-70, 70)
        self.stuck_start_time = None  # Reset

    # ==================== COMMANDES MOTEUR ====================
    
    def publish_pwm(self, left, right):
        """Publie les commandes PWM aux moteurs avec limite STRICTE à 110"""
        # SÉCURITÉ: Double vérification de la limite
        left_clamped = int(np.clip(left, -110, 110))
        right_clamped = int(np.clip(right, -110, 110))
        
        msg = Int16MultiArray(data=[left_clamped, right_clamped])
        self.motor_pub.publish(msg)

    def stop_robot(self):
        """Arrêt complet du robot"""
        self.publish_pwm(0, 0)

    # ==================== VISUALISATION ====================
    
    def publish_markers(self, x, y, heading, steer, gap_quality=0.0, gap_width=0, is_searching=False):
        """Publie des markers de debug pour RViz"""
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        
        # 1. Flèche de direction (Cyan/Bleu)
        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = now
        arrow.type = Marker.ARROW
        arrow.id = 0
        arrow.scale.x = self.current_lookahead
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.a = 1.0
        arrow.color.b = 1.0
        arrow.color.g = 0.5
        arrow.pose.position.x = float(x)
        arrow.pose.position.y = float(y)
        arrow.pose.position.z = 0.25
        
        target_yaw = heading + steer
        arrow.pose.orientation.w = math.cos(target_yaw / 2)
        arrow.pose.orientation.z = math.sin(target_yaw / 2)
        markers.markers.append(arrow)
        
        # 2. Point cible (Vert/Orange/Rouge selon qualité)
        sphere = Marker()
        sphere.header.frame_id = "map"
        sphere.header.stamp = now
        sphere.type = Marker.SPHERE
        sphere.id = 1
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.20
        sphere.color.a = 1.0
        
        if is_searching:
            sphere.color.r = 1.0  # Rouge
        elif gap_quality > 0.7:
            sphere.color.g = 1.0  # Vert
        elif gap_quality > 0.4:
            sphere.color.r = 1.0
            sphere.color.g = 0.5  # Orange
        else:
            sphere.color.r = 1.0
            sphere.color.g = 0.2  # Rouge-orange
        
        sphere.pose.position.x = x + self.current_lookahead * math.cos(target_yaw)
        sphere.pose.position.y = y + self.current_lookahead * math.sin(target_yaw)
        sphere.pose.position.z = 0.25
        markers.markers.append(sphere)
        
        # 3. Rayon de sécurité (Cercle)
        circle = Marker()
        circle.header.frame_id = "map"
        circle.header.stamp = now
        circle.type = Marker.CYLINDER
        circle.id = 2
        circle.scale.x = self.SAFETY_RADIUS * 2
        circle.scale.y = self.SAFETY_RADIUS * 2
        circle.scale.z = 0.02
        circle.color.a = 0.3
        circle.color.b = 1.0
        circle.pose.position.x = sphere.pose.position.x
        circle.pose.position.y = sphere.pose.position.y
        circle.pose.position.z = 0.01
        markers.markers.append(circle)
        
        # 4. Texte d'information
        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = now
        text.type = Marker.TEXT_VIEW_FACING
        text.id = 3
        text.scale.z = 0.15
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = 0.6
        text.text = f"Q:{gap_quality:.2f} W:{gap_width} S:{int(self.current_speed)}"
        markers.markers.append(text)
        
        self.marker_pub.publish(markers)


# ==================== MAIN ====================

def main():
    rclpy.init()
    node = AdaptiveCorridorFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Arrêt demandé par l'utilisateur")
        node.stop_robot()
    finally:
        node.get_logger().info(f"📊 Distance totale: {node.total_distance:.2f}m")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()