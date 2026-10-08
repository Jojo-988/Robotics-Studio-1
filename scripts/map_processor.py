#!/usr/bin/env python3
""" Receive a SLAM map and report its metadata"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

class MapDataProcessor(Node):
    def __init__(self):
        super().__init__('map_data_processor')

        self.declare_parameter('map_topic','/husky1/map')
        topic = self.get_parameter('map_topic').value

        # match the SLAM map publisher and receive its retained map
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.subscription = self.create_subscription(
            OccupancyGrid, topic, self.map_callback, qos
        )
        self.get_logger().info(f'Waiting for map on {topic}')

    def map_callback(self, msg):
        info = msg.info

        if(
            info.width == 0
            or info.height == 0
            or info.resolution <= 0
            or len(msg.data) != info.width * info.height
        ):
            self.get_logger().warning(
                'Received invalid map metadata or size'
            )
            return

            
        self.get_logger().info(
            f'Map received: frame={msg.header.frame_id}, '
            f'size={info.width}x{info.height}, '
            f'resolution={info.resolution:.3f} m/cell, '
            f'origin=({info.origin.position.x:.2f}, '
            f'{info.origin.position.y:.2f})',
            throttle_duration_sec=2.0,
            )
        
def main(args =None):
    rclpy.init(args=args)
    node = MapDataProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if  rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
