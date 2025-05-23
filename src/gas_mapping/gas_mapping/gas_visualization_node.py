import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from gas_mapping_msgs.msg import GasReadingStamped
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class GasVisualizationNode(Node):
    def __init__(self):
        super().__init__('gas_visualization_node')

        self.subscription = self.create_subscription(
            GasReadingStamped,
            '/gas_reading',
            self.gas_callback,
            10
        )

        # Separate publishers for each gas type
        self.marker_pubs = {
            'mq2': self.create_publisher(MarkerArray, '/mq2_gas_markers', 10),
            'mq3': self.create_publisher(MarkerArray, '/mq3_gas_markers', 10),
            'mq9': self.create_publisher(MarkerArray, '/mq9_gas_markers', 10),
            'mq135': self.create_publisher(MarkerArray, '/mq135_gas_markers', 10),
        }

        self.pc_pubs = {
            'mq2': self.create_publisher(PointCloud2, '/mq2_gas_pointcloud', 10),
            'mq3': self.create_publisher(PointCloud2, '/mq3_gas_pointcloud', 10),
            'mq9': self.create_publisher(PointCloud2, '/mq9_gas_pointcloud', 10),
            'mq135': self.create_publisher(PointCloud2, '/mq135_gas_pointcloud', 10),
        }

        self.marker_id = 0

        # Buffers to accumulate gas readings per type
        self.gas_buffers = {
            'mq2': [],
            'mq3': [],
            'mq9': [],
            'mq135': [],
        }

    def gas_callback(self, msg: GasReadingStamped):
        gases = {
            'mq2': (msg.mq2_percent, ColorRGBA(r=0.2, g=0.2, b=0.2, a=0.0)),
            'mq3': (msg.mq3_percent, ColorRGBA(r=1.0, g=0.9568, b=0.2902, a=0.0)),
            'mq9': (msg.mq9_percent, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.0)),
            'mq135': (msg.mq135_percent, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.0)),
        }

        offset_map = {
            'mq2': (0.1, 0.1),
            'mq3': (-0.1, 0.1),
            'mq9': (-0.1, -0.1),
            'mq135': (0.1, -0.1),
        }

        for gas_type, (percent, color) in gases.items():
            markers = MarkerArray()

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = msg.header.stamp
            marker.ns = gas_type
            marker.id = self.marker_id
            self.marker_id += 1

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = msg.pose.position.x + offset_map[gas_type][0]
            marker.pose.position.y = msg.pose.position.y + offset_map[gas_type][1]
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            color.a = min(max(percent / 100.0, 0.05), 1.0)
            marker.color = color

            marker.lifetime.sec = 30
            markers.markers.append(marker)

            self.marker_pubs[gas_type].publish(markers)

            rgba = struct.unpack('I', struct.pack('BBBB', int(color.r*255), int(color.g*255), int(color.b*255), int(color.a*255)))[0]
            self.gas_buffers[gas_type].append([
                msg.pose.position.x + offset_map[gas_type][0],
                msg.pose.position.y + offset_map[gas_type][1],
                0.2,
                rgba
            ])

            self.publish_pointcloud(gas_type)

    def publish_pointcloud(self, gas_type):
        buffer = self.gas_buffers[gas_type]
        if not buffer:
            return

        cloud_msg = PointCloud2()
        cloud_msg.header = Header()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = "map"

        cloud_msg.height = 1
        cloud_msg.width = len(buffer)
        cloud_msg.is_dense = False
        cloud_msg.is_bigendian = False
        cloud_msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * len(buffer)
        cloud_msg.data = np.array(buffer, dtype=np.float32).tobytes()

        self.pc_pubs[gas_type].publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GasVisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
