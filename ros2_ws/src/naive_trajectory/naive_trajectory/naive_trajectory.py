import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int16MultiArray
import numpy as np

class MinimalTrajectory(Node):

    def __init__(self):
        super().__init__("naive_trajectory")

        # Subscription to the Occupancy Grid (Map)
        self.sub_mapping = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Subscription to Odometry (Current Position)
        # We use /odom instead of /amcl_pose for real-time SLAM navigation
        self.sub_pose = self.create_subscription(
            Odometry,
            '/odom_rf2o',
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

        # Timer for running control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Trajectory Node Started - Waiting for Map and Odom...")

    def map_callback(self, msg):
        self.map = msg

    def pose_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract Yaw (Rotation) from Quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        self.pose = (np.array([x, y]), yaw)

    def angle_to_wheels(self, target_angle, base_speed=140):
        """
        target_angle: 0 is straight, positive is left, negative is right.
        """
        # Tuning parameter: how aggressively to turn
        steering_sensitivity = 150.0 
        
        turn_effort = target_angle * steering_sensitivity
        
        left_pwm = base_speed - turn_effort
        right_pwm = base_speed + turn_effort
        
        # Clip to hardware limits (0-255 for ESP32)
        left = int(np.clip(left_pwm, 0, 255))
        right = int(np.clip(right_pwm, 0, 255))
        
        return left, right

    def occupancy_grid_to_local_points(self, grid, car_pos, yaw):
        res = grid.info.resolution
        w = grid.info.width
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        # Only process points within a 4m box of the car to save CPU
        local_points = []
        for idx, val in enumerate(grid.data):
            if val < 60: # Threshold for "Occupied"
                continue
            
            # Map coordinates
            grid_y = idx // w
            grid_x = idx % w
            wx = ox + (grid_x + 0.5) * res
            wy = oy + (grid_y + 0.5) * res
            
            # 1. Translate to Car as Origin
            dx = wx - car_pos[0]
            dy = wy - car_pos[1]
            
            # 2. Rotate to Car Heading (Car Front = 0 rad)
            lx = dx * np.cos(-yaw) - dy * np.sin(-yaw)
            ly = dx * np.sin(-yaw) + dy * np.cos(-yaw)
            
            # Only keep points in front (lx > 0) and close range
            if 0.1 < lx < 3.5 and abs(ly) < 3.0:
                local_points.append([lx, ly])
                
        return np.array(local_points)

    def find_max_gap(self, angles):
        if len(angles) < 2:
            return -0.5, 0.5 # Default small forward corridor
            
        angles = np.sort(angles)
        gaps = np.diff(angles)
        idx = np.argmax(gaps)
        return angles[idx], angles[idx + 1]

    def control_loop(self):
        if self.map is None or self.pose is None:
            return

        car_pos, yaw = self.pose
        
        # Transform map data to car-local coordinates
        points = self.occupancy_grid_to_local_points(self.map, car_pos, yaw)

        if len(points) == 0:
            # Path is clear - drive straight
            self.publish_motor_pwm(150, 150)
            return

        # Calculate angles to all obstacles
        obs_angles = np.arctan2(points[:, 1], points[:, 0])
        
        # Safety: Add field of view boundaries so it doesn't try to turn 180 deg
        obs_angles = np.append(obs_angles, [-np.pi/2, np.pi/2])

        # Find the biggest gap between obstacles
        start_a, end_a = self.find_max_gap(obs_angles)
        best_angle = (start_a + end_a) / 2.0

        # Convert the chosen angle to motor speeds
        left, right = self.angle_to_wheels(best_angle)
        self.publish_motor_pwm(left, right)
        
        # Debugging
        self.get_logger().info(f"Gap found: {np.degrees(best_angle):.1f}° | PWM: L={left} R={right}")

    def publish_motor_pwm(self, left, right):
        msg = Int16MultiArray()
        msg.data = [left, right]
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_motor_pwm(0, 0) # Stop on exit
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()