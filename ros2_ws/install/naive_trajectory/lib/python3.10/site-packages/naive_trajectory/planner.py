#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import tf2_ros
import math
import numpy as np
from scipy.ndimage import binary_dilation, binary_opening, binary_closing
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
        self.ros_path_cache = None

        self.map_data = None
        self.map_info = None

        self.checkpoints     = []
        self.CHECKPOINT_DIST = 0.8

        self.INFLATION_MAX_PIXELS = 6
        self.INFLATION_MIN_PIXELS = 0  # 0 = carte brute, dernier recours
        self.DOWNSAMPLE_STEP = 3

        self.ROBOT_FRAME = 'laser_link'
        self.MAP_FRAME   = 'map'

        self.create_timer(0.3, self.record_checkpoints)
        self.create_timer(2.0, self.republish_path)

        self.get_logger().info("🗺️ Global Planner prêt — frame: laser_link")

    # ------------------------------------------------------------------ #
    #  CALLBACKS                                                           #
    # ------------------------------------------------------------------ #

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        if self.path_generated and self.ros_path_cache is not None:
            self.get_logger().info("🔄 Carte mise à jour → recalcul A*")
            self.path_generated = False
            self.generate_optimal_path()

    def lap_callback(self, msg):
        self.current_lap = msg.data
        if self.current_lap >= 2 and not self.path_generated:
            self.generate_optimal_path()

    def republish_path(self):
        if self.ros_path_cache is None:
            return
        self.ros_path_cache.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.ros_path_cache)

    def record_checkpoints(self):
        if self.current_lap != 1 or self.map_info is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.MAP_FRAME, self.ROBOT_FRAME, rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
        except Exception:
            return

        if not self.checkpoints:
            self.checkpoints.append((x, y))
            return

        last_x, last_y = self.checkpoints[-1]
        if math.hypot(x - last_x, y - last_y) >= self.CHECKPOINT_DIST:
            self.checkpoints.append((x, y))
            self.get_logger().info(
                f"📍 Checkpoint {len(self.checkpoints)} à ({x:.2f}, {y:.2f})")

    # ------------------------------------------------------------------ #
    #  CARTE SÉCURISÉE                                                     #
    # ------------------------------------------------------------------ #

    def build_safe_map(self, radius_pixels):
        """
        Construit une carte navigable avec niveaux de lissage progressifs.
        Plus radius_pixels est bas, plus la carte est permissive.
        À radius=0 on retourne la carte la plus permissive possible.
        """
        # Obstacles certains uniquement (ignore les inconnues -1 pour max permissivité)
        if radius_pixels == 0:
            # Carte ultra-permissive : seulement les murs durs, pas d'inflation
            obstacles = (self.map_data == 100)
            # Juste un opening minimal pour enlever le bruit pixel isolé
            obstacles = binary_opening(obstacles, structure=np.ones((2, 2)))
            return np.where(obstacles, 100, 0)

        # Carte normale avec lissage
        obstacles = (self.map_data == 100) | (self.map_data == -1)

        # 1. Ferme les trous dans les murs (artéfacts SLAM)
        obstacles = binary_closing(obstacles, structure=np.ones((5, 5)))

        # 2. Supprime les pixels obstacles isolés
        obstacles = binary_opening(obstacles, structure=np.ones((3, 3)))

        # 3. Lissage final des contours
        obstacles = binary_closing(obstacles, structure=np.ones((3, 3)))

        # 4. Dilation des espaces libres : grignote les obstacles
        #    pour garantir un passage dans les zones serrées
        free = binary_dilation(~obstacles, structure=np.ones((3, 3)))
        obstacles = ~free

        # 5. Inflation de sécurité
        structure = np.ones((radius_pixels * 2 + 1, radius_pixels * 2 + 1))
        obstacles = binary_dilation(obstacles, structure=structure)

        return np.where(obstacles, 100, 0)

    # ------------------------------------------------------------------ #
    #  SNAP VERS CASE LIBRE                                                #
    # ------------------------------------------------------------------ #

    def snap_to_free(self, gx, gy, safe_map, search_radius=20):
        """Déplace un point bloqué vers la case libre la plus proche."""
        h, w = safe_map.shape
        if 0 <= gy < h and 0 <= gx < w and safe_map[gy, gx] == 0:
            return gx, gy

        best, best_dist = None, float('inf')
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= ny < h and 0 <= nx < w and safe_map[ny, nx] == 0:
                    d = math.hypot(dx, dy)
                    if d < best_dist:
                        best_dist = d
                        best = (nx, ny)
        return best if best else (gx, gy)

    # ------------------------------------------------------------------ #
    #  A*                                                                  #
    # ------------------------------------------------------------------ #

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
                    tg = g_score[current] + math.hypot(dx, dy)
                    if neighbor not in g_score or tg < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor]   = tg
                        heapq.heappush(open_set,
                            (tg + heuristic(neighbor, goal_grid), neighbor))
        return []

    # ------------------------------------------------------------------ #
    #  A* AVEC FALLBACK PROGRESSIF                                         #
    # ------------------------------------------------------------------ #

    def a_star_with_fallback(self, start_g, goal_g, segment_idx):
        """
        Essaie A* avec inflation décroissante.
        Niveaux : 6px → 5px → ... → 1px → 0px (carte brute permissive)
        À chaque niveau, snap start/goal vers case libre avant de tenter.
        Garantit un chemin sauf si les deux points sont physiquement
        dans le même mur solide.
        """
        for radius in range(self.INFLATION_MAX_PIXELS, self.INFLATION_MIN_PIXELS - 1, -1):
            safe_map = self.build_safe_map(radius)
            h, w = safe_map.shape

            sx, sy = self.snap_to_free(start_g[0], start_g[1], safe_map)
            gx, gy = self.snap_to_free(goal_g[0],  goal_g[1],  safe_map)

            if not (0 <= sy < h and 0 <= sx < w and
                    0 <= gy < h and 0 <= gx < w):
                continue
            if safe_map[sy, sx] == 100 or safe_map[gy, gx] == 100:
                continue

            segment = self.a_star((sx, sy), (gx, gy), safe_map)
            if segment:
                if radius < self.INFLATION_MAX_PIXELS:
                    self.get_logger().warn(
                        f"  Segment {segment_idx} : inflation={radius}px "
                        f"({'carte brute' if radius == 0 else f'{radius*5}cm'})")
                return segment, radius

        self.get_logger().error(
            f"❌ Segment {segment_idx} vraiment infranchissable — checkpoint dans un mur solide")
        return [], -1

    # ------------------------------------------------------------------ #
    #  GÉNÉRATION DU PATH                                                  #
    # ------------------------------------------------------------------ #

    def downsample(self, path_grid):
        if len(path_grid) <= 1:
            return path_grid
        sampled = path_grid[::self.DOWNSAMPLE_STEP]
        if sampled[-1] != path_grid[-1]:
            sampled.append(path_grid[-1])
        return sampled

    def world_to_grid(self, x, y):
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.map_info.resolution + self.map_info.origin.position.x
        y = gy * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def generate_optimal_path(self):
        if not self.checkpoints or self.map_data is None:
            self.get_logger().warn("Données manquantes pour le calcul.")
            return

        self.get_logger().info(
            f"⚙️ A* en cours ({len(self.checkpoints)} checkpoints)...")
        self.path_generated = True

        full_path_grid   = []
        checkpoints_loop = self.checkpoints + [self.checkpoints[0]]
        failed_segments  = []

        for i in range(len(checkpoints_loop) - 1):
            start_g = self.world_to_grid(*checkpoints_loop[i])
            goal_g  = self.world_to_grid(*checkpoints_loop[i + 1])

            segment, radius_used = self.a_star_with_fallback(start_g, goal_g, i)

            if not segment:
                # Segment raté : on relie en ligne droite pour ne pas bloquer
                self.get_logger().warn(
                    f"⚠️ Segment {i}→{i+1} : ligne droite de secours")
                segment = [start_g, goal_g]
                failed_segments.append(i)

            self.get_logger().info(
                f"✔ Segment {i}→{i+1} : {len(segment)} pts, "
                f"inflation={radius_used}px")
            full_path_grid.extend(segment[:-1])

        if failed_segments:
            self.get_logger().warn(
                f"⚠️ {len(failed_segments)} segment(s) en ligne droite : {failed_segments}")

        full_path_grid = self.downsample(full_path_grid)
        self.get_logger().info(
            f"📐 {len(full_path_grid)} waypoints après sous-échantillonnage")

        ros_path = Path()
        ros_path.header.frame_id = self.MAP_FRAME
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
        self.get_logger().info("✅ Path publié sur /global_path")


def main():
    rclpy.init()
    rclpy.spin(GlobalPlanner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()