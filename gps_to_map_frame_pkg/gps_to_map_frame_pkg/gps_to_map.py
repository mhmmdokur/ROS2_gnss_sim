#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix

class PointToGPSNode(Node):
    def __init__(self):
        super().__init__('point_to_gps_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointStamped,
            'clicked_point',
            self.point_callback,
            10
        )
        
        # Publisher
        self.publisher = self.create_publisher(NavSatFix, 'tf2_gps_pos', 10)

    def point_callback(self, msg):
        try:
            transformed_point = self.tf_buffer.transform(msg, 'tf2_gps_frame', timeout=rclpy.duration.Duration(seconds=3))
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'tf2_gps_frame'
            gps_msg.latitude = transformed_point.point.y
            gps_msg.longitude = transformed_point.point.x
            gps_msg.altitude = transformed_point.point.z  # Assuming the altitude might be useful
            self.publisher.publish(gps_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to transform PointStamped to NavSatFix: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointToGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
