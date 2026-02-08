import numpy as np
import matplotlib.pyplot as plt

# =====================================================
# 1. OccupancyGrid (structure ROS-like)
# =====================================================
class FakeOccupancyGrid:
    class Info:
        def __init__(self, resolution, width, height, origin):
            self.resolution = resolution #taille cellule
            self.width = width #nb cellule X
            self.height = height #Y
            self.origin = origin 

    class Origin:
        class Position:
            def __init__(self, x, y):
                self.x = x
                self.y = y
        def __init__(self, x, y):
            self.position = self.Position(x, y)

    def __init__(self, resolution, width, height, origin_x, origin_y):
        self.info = self.Info(
            resolution,
            width,
            height,
            self.Origin(origin_x, origin_y)
        )
        self.data = np.zeros(width * height, dtype=np.int8) #représente toute la grille

# =====================================================
# 2. Construction du corridor zigzag (dans la grille)
# =====================================================
def build_zigzag_corridor(grid, d=3.0, A=3.0, k=0.4):
    res = grid.info.resolution
    w = grid.info.width
    h = grid.info.height
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y

    for iy in range(h):
        for ix in range(w):
            wx = ox + (ix + 0.5) * res #milieu de la cellule
            wy = oy + (iy + 0.5) * res

            y_left =  d + A * np.sin(k * wx) 
            y_right = -d + A * np.sin(k * wx)

            if abs(wy - y_left) < res or abs(wy - y_right) < res:
                grid.data[iy * w + ix] = 100  # occupé

# =====================================================
# 3. OccupancyGrid -> pseudo point cloud
# =====================================================
def occupancy_grid_to_points(grid):
    res = grid.info.resolution
    w = grid.info.width
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y

    xs, ys = [], []

    for idx, val in enumerate(grid.data):
        if val < 50:
            continue

        y = idx // w
        x = idx % w

        wx = ox + (x + 0.5) * res
        wy = oy + (y + 0.5) * res

        xs.append(wx)
        ys.append(wy)

    return np.array(xs), np.array(ys)

# =====================================================
# 4. Follow-the-Gap (naïf, inchangé)
# =====================================================
def preprocess_lidar(x_points, y_points, car_pos, max_distance=4.0):
    dx = x_points - car_pos[0]
    dy = y_points - car_pos[1]
    distances = np.sqrt(dx**2 + dy**2)

    mask = distances <= max_distance
    dx, dy, distances = dx[mask], dy[mask], distances[mask]

    angles = np.arctan2(dy, dx)

    # champ de vision avant uniquement
    fov = np.deg2rad(180)
    mask = np.abs(angles) < fov / 2

    return angles[mask], distances[mask]

def create_bubble(angles, distances, bubble_radius=0.5):
    mask = distances > bubble_radius
    return angles[mask], distances[mask]

def find_max_gap(angles):
    angles = np.sort(angles)
    gaps = np.diff(angles)
    i = np.argmax(gaps)
    return angles[i], angles[i + 1]

def choose_best_point(start_angle, end_angle):
    return 0.5 * (start_angle + end_angle)

# =====================================================
# 5. Simulation
# =====================================================
# Grille
grid = FakeOccupancyGrid(
    resolution=0.2,
    width=300,
    height=200,
    origin_x=-5.0,
    origin_y=-10.0
)

build_zigzag_corridor(grid)

# Grille -> points
x_points, y_points = occupancy_grid_to_points(grid)

# Véhicule
car_pos = np.array([0.0, 0.0])
trajectory = [car_pos.copy()]
gaps_history = []
angles_chosen = []

dt = 1 #choisir le pas  (ici tous les 1m);
n_steps = 50

for _ in range(n_steps):
    angles, distances = preprocess_lidar(x_points, y_points, car_pos)
    angles, distances = create_bubble(angles, distances)

    if len(angles) < 2:
        break

    start_a, end_a = find_max_gap(angles)
    best_angle = choose_best_point(start_a, end_a)

    gaps_history.append((start_a, end_a))
    angles_chosen.append(best_angle)

    car_pos += dt * np.array([np.cos(best_angle), np.sin(best_angle)])
    trajectory.append(car_pos.copy())

trajectory = np.array(trajectory)

# =====================================================
# 6. Affichage
# =====================================================
plt.figure(figsize=(12, 5))
plt.scatter(x_points, y_points, s=5, c='blue', label="Obstacles (grille)")
plt.plot(trajectory[:,0], trajectory[:,1], 'k-', lw=2, label="Trajectoire")
plt.scatter(trajectory[0,0], trajectory[0,1], c='red', label="Départ")

# Affichage des gaps
for i, (a0, a1) in enumerate(gaps_history):
    x0, y0 = trajectory[i]
    L = 2.5
    plt.plot([x0, x0 + L*np.cos(a0)], [y0, y0 + L*np.sin(a0)], 'c--', alpha=0.4)
    plt.plot([x0, x0 + L*np.cos(a1)], [y0, y0 + L*np.sin(a1)], 'c--', alpha=0.4)

# Direction choisie
for i, a in enumerate(angles_chosen):
    plt.arrow(
        trajectory[i,0], trajectory[i,1],
        0.8*np.cos(a), 0.8*np.sin(a),
        head_width=0.15, fc='purple', ec='purple'
    )

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Algo traj (Follow-the-Gap) à partir d'une OccupancyGrid")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.show()
