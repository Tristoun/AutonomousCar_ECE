#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import tf2_ros
import math
import numpy as np
from scipy.ndimage import binary_dilation
import heapq

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')

        self.create_subscription(OccupancyGrid, '/map',       self.map_callback, 10)
        self.create_subscription(Int32,         '/lap_count', self.lap_callback, 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_lap    = 1
        self.path_generated = False
        self.ros_path_cache = None   # FIX 1 : on garde le path pour le republier

        self.map_data = None
        self.map_info = None

        self.checkpoints     = []
        self.CHECKPOINT_DIST = 1.5

        # Inflation max souhaitée (25 cm à résolution 0.05 m)
        # En cas d'échec A*, on réduit automatiquement jusqu'à INFLATION_MIN
        self.INFLATION_MAX_PIXELS = 5
        self.INFLATION_MIN_PIXELS = 1   # plancher absolu : 1 pixel = 5 cm de marge

        # FIX 3 : sous-échantillonnage du path A* (1 point gardé sur N)
        # Avec résolution=0.05 m et step=4 → un waypoint tous les ~20 cm
        self.DOWNSAMPLE_STEP = 4

        self.create_timer(0.5, self.record_checkpoints)
        # FIX 1 : republier le path toutes les 2 s pour que le pure_pursuit
        # le reçoive même s'il démarre après la génération
        self.create_timer(2.0, self.republish_path)

        self.get_logger().info("🗺️ Global Planner prêt. Enregistrement Tour 1...")

    # ------------------------------------------------------------------ callbacks
    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def lap_callback(self, msg):
        self.current_lap = msg.data
        if self.current_lap >= 2 and not self.path_generated:
            self.generate_optimal_path()

    # ------------------------------------------------------------------ FIX 1 : republication
    def republish_path(self):
        """Republie le path calculé toutes les 2 s pour éviter les messages manqués."""
        if self.ros_path_cache is None:
            return
        self.ros_path_cache.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.ros_path_cache)

    # ------------------------------------------------------------------ enregistrement checkpoints
    def record_checkpoints(self):
        if self.current_lap != 1 or self.map_info is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x  = tf.transform.translation.x
            y  = tf.transform.translation.y
        except Exception:
            return

        if not self.checkpoints:
            self.checkpoints.append((x, y))
            return

        last_x, last_y = self.checkpoints[-1]
        if math.hypot(x - last_x, y - last_y) >= self.CHECKPOINT_DIST:
            self.checkpoints.append((x, y))
            self.get_logger().info(f"📍 Checkpoint {len(self.checkpoints)} enregistré à ({x:.2f}, {y:.2f})")

    # ------------------------------------------------------------------ utilitaires carte
    def world_to_grid(self, x, y):
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.map_info.resolution + self.map_info.origin.position.x
        y = gy * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    # ------------------------------------------------------------------ A*
    def a_star(self, start_grid, goal_grid, obstacle_map):
        neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        open_set  = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score   = {start_grid: 0}

        def heuristic(a, b):
            return math.hypot(a[0]-b[0], a[1]-b[1])

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_grid:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_grid)
                return path[::-1]

            for dx, dy in neighbors:
                neighbor = (current[0]+dx, current[1]+dy)
                h, w = obstacle_map.shape
                if 0 <= neighbor[1] < h and 0 <= neighbor[0] < w:
                    if obstacle_map[neighbor[1], neighbor[0]] != 0:
                        continue
                    cost = math.hypot(dx, dy)
                    tg   = g_score[current] + cost
                    if neighbor not in g_score or tg < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor]   = tg
                        f = tg + heuristic(neighbor, goal_grid)
                        heapq.heappush(open_set, (f, neighbor))
        return []

    # ------------------------------------------------------------------ FIX 3 : sous-échantillonnage
    def downsample(self, path_grid):
        """Garde 1 point sur DOWNSAMPLE_STEP pour alléger le path.
        Un waypoint tous les ~20 cm avec résolution=0.05 m et step=4.
        Le dernier point est toujours conservé pour boucler proprement."""
        if len(path_grid) <= 1:
            return path_grid
        sampled = path_grid[::self.DOWNSAMPLE_STEP]
        if sampled[-1] != path_grid[-1]:
            sampled.append(path_grid[-1])
        return sampled

    # ------------------------------------------------------------------ carte gonflée (mise en cache)
    def build_safe_map(self, radius_pixels):
        """Construit la carte avec un rayon d'inflation donné."""
        obstacles = (self.map_data == 100) | (self.map_data == -1)
        structure = np.ones((radius_pixels * 2 + 1, radius_pixels * 2 + 1))
        inflated  = binary_dilation(obstacles, structure=structure)
        return np.where(inflated, 100, 0)

    # ------------------------------------------------------------------ A* avec fallback d'inflation
    def a_star_with_fallback(self, start_g, goal_g, segment_idx):
        """Tente A* en réduisant l'inflation de INFLATION_MAX à INFLATION_MIN
        jusqu'à trouver un chemin. Retourne (segment, rayon_utilisé) ou ([], 0)."""
        for radius in range(self.INFLATION_MAX_PIXELS, self.INFLATION_MIN_PIXELS - 1, -1):
            safe_map = self.build_safe_map(radius)

            # Vérifier que start/goal ne sont pas dans un mur gonflé
            sy, sx = start_g[1], start_g[0]
            gy, gx = goal_g[1],  goal_g[0]
            h, w   = safe_map.shape
            start_blocked = not (0 <= sy < h and 0 <= sx < w) or safe_map[sy, sx] == 100
            goal_blocked  = not (0 <= gy < h and 0 <= gx < w) or safe_map[gy, gx] == 100

            if start_blocked or goal_blocked:
                self.get_logger().warn(
                    f"  Segment {segment_idx} : inflation={radius}px "
                    f"→ start_bloqué={start_blocked} goal_bloqué={goal_blocked}, on réduit..."
                )
                continue

            segment = self.a_star(start_g, goal_g, safe_map)
            if segment:
                if radius < self.INFLATION_MAX_PIXELS:
                    self.get_logger().warn(
                        f"  Segment {segment_idx} : trouvé avec inflation réduite à "
                        f"{radius}px ({radius * 0.05 * 100:.0f} cm de marge)"
                    )
                return segment, radius

        return [], 0

    # ------------------------------------------------------------------ génération du path global
    def generate_optimal_path(self):
        if not self.checkpoints or self.map_data is None:
            self.get_logger().warn("Données manquantes pour calculer le chemin.")
            return

        self.get_logger().info(
            f"⚙️ Calcul A* en cours "
            f"(inflation {self.INFLATION_MAX_PIXELS}→{self.INFLATION_MIN_PIXELS} px)..."
        )
        self.path_generated = True

        full_path_grid   = []
        checkpoints_loop = self.checkpoints + [self.checkpoints[0]]

        for i in range(len(checkpoints_loop) - 1):
            start_m = checkpoints_loop[i]
            goal_m  = checkpoints_loop[i + 1]
            start_g = self.world_to_grid(start_m[0], start_m[1])
            goal_g  = self.world_to_grid(goal_m[0],  goal_m[1])

            segment, radius_used = self.a_star_with_fallback(start_g, goal_g, i)

            if not segment:
                self.get_logger().error(
                    f"❌ Impossible de relier checkpoint {i} → {i+1} "
                    f"même avec inflation minimale ({self.INFLATION_MIN_PIXELS}px)"
                )
                self.path_generated = False   # autorise un nouvel essai si la carte évolue
                return

            self.get_logger().info(
                f"✔ Segment {i}→{i+1} : {len(segment)} pts, "
                f"inflation={radius_used}px ({radius_used * 0.05 * 100:.0f} cm)"
            )
            full_path_grid.extend(segment[:-1])

        # Sous-échantillonnage avant publication
        full_path_grid = self.downsample(full_path_grid)
        self.get_logger().info(f"📐 Path : {len(full_path_grid)} waypoints après sous-échantillonnage")

        # Conversion en message ROS Path
        ros_path              = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp    = self.get_clock().now().to_msg()

        for (gx, gy) in full_path_grid:
            x, y = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header             = ros_path.header
            pose.pose.position.x    = float(x)
            pose.pose.position.y    = float(y)
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.ros_path_cache = ros_path
        self.path_pub.publish(ros_path)
        self.get_logger().info("✅ Chemin A* publié sur /global_path (republication auto toutes les 2 s)")


def main():
    rclpy.init()
    rclpy.spin(GlobalPlanner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()