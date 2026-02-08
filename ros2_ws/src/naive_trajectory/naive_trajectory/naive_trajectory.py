import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int16MultiArray
import numpy as np

class MinimalTrajectory(Node):

    def __init__(self):
        super().__init__("naive_trajectory")

        self.sub_mapping = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.motor_pub = self.create_publisher(
            Int16MultiArray,
            '/motor_pwm',
            10
        )

        self.map = None
        self.pose = None  

        # Timer for running algorithm at 10 Hz
        self.create_timer(0.1, self.control_loop)

    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info("Map received")

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = np.arctan2(siny, cosy)
        self.pose = (np.array([x, y]), yaw)

    def angle_to_wheels(self, angle, base_speed=180):
        # Convert desired angle to left/right motor PWM
        turn = np.clip(angle / np.pi, -1.0, 1.0)
        left = base_speed * (1.0 - turn)
        right = base_speed * (1.0 + turn)
        left = int(np.clip(left, -100, 100))
        right = int(np.clip(right, -100, 100))
        return left, right

    def occupancy_grid_to_points(self, grid):
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

    def preprocess_lidar(self, x_points, y_points, car_pos, max_distance=4.0):
        dx = x_points - car_pos[0]
        dy = y_points - car_pos[1]
        distances = np.sqrt(dx**2 + dy**2)
        mask = distances <= max_distance
        dx, dy, distances = dx[mask], dy[mask], distances[mask]
        angles = np.arctan2(dy, dx)
        fov = np.deg2rad(180)  # front only
        mask = np.abs(angles) < fov / 2
        return angles[mask], distances[mask]

    def create_bubble(self, angles, distances, bubble_radius=0.5):
        mask = distances > bubble_radius
        return angles[mask], distances[mask]

    def find_max_gap(self, angles):
        angles = np.sort(angles)
        gaps = np.diff(angles)
        if len(gaps) == 0:
            return 0.0, 0.0
        i = np.argmax(gaps)
        return angles[i], angles[i + 1]

    def choose_best_point(self, start_angle, end_angle):
        return 0.5 * (start_angle + end_angle)

    def publish_motor_pwm(self, left, right):
        msg = Int16MultiArray()
        msg.data = [left, right]
        self.motor_pub.publish(msg)

    def control_loop(self):
        if self.map is None or self.pose is None:
            return

        car_pos, yaw = self.pose
        x_points, y_points = self.occupancy_grid_to_points(self.map)

        # Follow-the-gap
        angles, distances = self.preprocess_lidar(x_points, y_points, car_pos)
        angles, distances = self.create_bubble(angles, distances)
        if len(angles) < 2:
            return

        start_a, end_a = self.find_max_gap(angles)
        best_angle = self.choose_best_point(start_a, end_a)

        # Convert to wheel commands
        left, right = self.angle_to_wheels(best_angle)
        self.get_logger().info("Try to send trajectory")

        self.publish_motor_pwm(left, right)

def main(args=None):
    rclpy.init(args=args)
    minimal_trajectory = MinimalTrajectory()
    rclpy.spin(minimal_trajectory)
    minimal_trajectory.destroy_node()
    rclpy.shutdown()
