import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import numpy as np
import math

# ---------------- Fake Occupancy Grid ----------------
class FakeOccupancyGrid:
    class Info:
        def __init__(self, resolution, width, height, origin):
            self.resolution = resolution
            self.width = width
            self.height = height
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
        self.data = np.zeros(width * height, dtype=np.int8)

# ---------------- Build zigzag corridor ----------------
def build_zigzag_corridor(grid, d=3.0, A=3.0, k=0.4):
    res = grid.info.resolution
    w = grid.info.width
    h = grid.info.height
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y

    for iy in range(h):
        for ix in range(w):
            wx = ox + (ix + 0.5) * res
            wy = oy + (iy + 0.5) * res

            y_left =  d + A * np.sin(k * wx) 
            y_right = -d + A * np.sin(k * wx)

            if abs(wy - y_left) < res or abs(wy - y_right) < res:
                grid.data[iy * w + ix] = 100  # occupied

# ---------------- ROS2 Node ----------------
class FakeMapPosePublisher(Node):
    def __init__(self):
        super().__init__("fake_map_pose_publisher")

        # Publisher for fake map
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        # Publisher for fake pose
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)

        # Timer to publish at 5 Hz
        self.create_timer(0.2, self.timer_callback)

        # Initialize fake occupancy grid
        self.grid = FakeOccupancyGrid(resolution=0.2, width=100, height=100, origin_x=-10, origin_y=-10)
        build_zigzag_corridor(self.grid)

        # Robot state
        self.robot_x = -8.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

    # ---------------- Convert to nav_msgs OccupancyGrid ----------------
    def occupancy_grid_to_msg(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = float(self.grid.info.resolution)
        msg.info.width = int(self.grid.info.width)
        msg.info.height = int(self.grid.info.height)
        msg.info.origin.position.x = float(self.grid.info.origin.position.x)
        msg.info.origin.position.y = float(self.grid.info.origin.position.y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.data.tolist()
        return msg

    # ---------------- Convert robot state to PoseWithCovarianceStamped ----------------
    def pose_to_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position = Point(x=self.robot_x, y=self.robot_y, z=0.0)
        qz = math.sin(self.robot_yaw / 2.0)
        qw = math.cos(self.robot_yaw / 2.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        return msg

    # ---------------- Timer callback ----------------
    def timer_callback(self):
        # Publish fake map
        map_msg = self.occupancy_grid_to_msg()
        self.map_pub.publish(map_msg)

        # Publish fake pose
        pose_msg = self.pose_to_msg()
        self.pose_pub.publish(pose_msg)

        # Move robot slowly forward along X
        self.robot_x += 0.05  # 5cm per tick
        self.robot_yaw = 0.0  # keep straight for now

def main(args=None):
    rclpy.init(args=args)
    node = FakeMapPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
