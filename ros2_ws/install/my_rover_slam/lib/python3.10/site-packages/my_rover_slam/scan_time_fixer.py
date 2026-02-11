import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanTimeFixer(Node):
    def __init__(self):
        super().__init__('scan_time_fixer')
        
        # Match micro-ROS default Best Effort QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Try RELIABLE first
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)

        # Output to rf2o with the same QoS
        self.publisher = self.create_publisher(LaserScan, '/scan_fixed', qos_profile)
        self.get_logger().info('Scan Time Fixer is active and re-stamping scans.')

    def listener_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link' # Force this to match your static transform
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanTimeFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()