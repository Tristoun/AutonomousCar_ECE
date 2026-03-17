#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.br = StaticTransformBroadcaster(self)
        self.publish_static_tfs()
        self.get_logger().info('Static TFs publiées (timestamp=0)')

    def make_tf(self, parent, child, x, y, z, yaw):
        tf = TransformStamped()
        # timestamp 0 = valable pour TOUS les temps → plus jamais de drop
        tf.header.stamp.sec     = 0
        tf.header.stamp.nanosec = 0
        tf.header.frame_id      = parent
        tf.child_frame_id       = child
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z

        # yaw → quaternion
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        return tf

    def publish_static_tfs(self):
        tfs = [
            # base_link → laser_link (0.1m devant, 0.05m haut, retourné à 180°)
            self.make_tf('base_link', 'laser_link',
                         x=0.1, y=0.0, z=0.05, yaw=math.pi),
            # base_link → imu_link
            self.make_tf('base_link', 'imu_link',
                         x=0.0, y=0.0, z=0.1,  yaw=0.0),
        ]
        self.br.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()