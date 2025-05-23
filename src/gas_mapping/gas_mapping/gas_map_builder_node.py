import rclpy
from rclpy.node import Node
from gas_mapping_msgs.msg import GasReadingStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import numpy as np
import struct
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration

class GasMapBuilder(Node):
    def __init__(self):
        super().__init__('gas_map_builder')

        self.subscription = self.create_subscription(
            GasReadingStamped,
            '/gas_reading',
            self.gas_callback,
            10
        )

        self.pc_pubs = {
            'mq2': self.create_publisher(PointCloud2, '/mq2_gas_heatmap', 10),
            'mq3': self.create_publisher(PointCloud2, '/mq3_gas_heatmap', 10),
            'mq9': self.create_publisher(PointCloud2, '/mq9_gas_heatmap', 10),
            'mq135': self.create_publisher(PointCloud2, '/mq135_gas_heatmap', 10),
        }

        self.color_map = {
            'mq2': (128, 128, 128),
            'mq3': (255, 255, 0),
            'mq9': (255, 0, 0),
            'mq135': (0, 255, 0),
        }

        self.grid_resolution = 0.1  # meters
        self.gas_map = {
            'mq2': {},
            'mq3': {},
            'mq9': {},
            'mq135': {},
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0, self.publish_heatmaps)

    def quantize(self, x, y):
        qx = round(x / self.grid_resolution) * self.grid_resolution
        qy = round(y / self.grid_resolution) * self.grid_resolution
        return (qx, qy)

    def gas_callback(self, msg: GasReadingStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.5)
            )
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose
            transformed_pose = do_transform_pose(pose, transform)
            px = transformed_pose.pose.position.x
            py = transformed_pose.pose.position.y
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        gases = {
            'mq2': msg.mq2_percent,
            'mq3': msg.mq3_percent,
            'mq9': msg.mq9_percent,
            'mq135': msg.mq135_percent,
        }

        for gas, percent in gases.items():
            key = self.quantize(px, py)
            current = self.gas_map[gas].get(key, 0.0)
            if abs(percent - current) > 1.0:
                self.gas_map[gas][key] = percent

    def publish_heatmaps(self):
        for gas, readings in self.gas_map.items():
            points = []
            r, g, b = self.color_map[gas]
            for (x, y), percent in readings.items():
                alpha = min(max(percent / 100.0, 0.05), 1.0)
                rgba = struct.unpack('I', struct.pack('BBBB', r, g, b, int(alpha * 255)))[0]
                points.append([x, y, 0.0, rgba])

            if not points:
                continue

            cloud_msg = PointCloud2()
            cloud_msg.header = Header()
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.header.frame_id = "map"

            cloud_msg.height = 1
            cloud_msg.width = len(points)
            cloud_msg.is_dense = False
            cloud_msg.is_bigendian = False
            cloud_msg.fields = [
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
            ]
            cloud_msg.point_step = 16
            cloud_msg.row_step = cloud_msg.point_step * len(points)
            cloud_msg.data = np.array(points, dtype=np.float32).tobytes()

            self.pc_pubs[gas].publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GasMapBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
