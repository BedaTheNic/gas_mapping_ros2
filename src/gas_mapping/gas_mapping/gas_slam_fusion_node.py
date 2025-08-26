import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32
from std_msgs.msg import Header
from gas_mapping_msgs.msg import GasReadingStamped  # Custom message
from rclpy.time import Time

class GasSLAMFusionNode(Node):
    def __init__(self):
        super().__init__('gas_slam_fusion_node')

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)

        self.create_subscription(Int32, '/mq2_gas_percent', self.mq2_callback, 10)
        self.create_subscription(Int32, '/mq3_gas_percent', self.mq3_callback, 10)
        self.create_subscription(Int32, '/mq9_gas_percent', self.mq9_callback, 10)
        self.create_subscription(Int32, '/mq135_gas_percent', self.mq135_callback, 10)

        # Publisher
        self.gas_pub = self.create_publisher(GasReadingStamped, '/gas_reading', 10)

        # Latest data storage
        self.latest_pose = None
        self.latest_odom = None
        self.latest_gas = {'mq2': 0, 'mq3': 0, 'mq9': 0, 'mq135': 0}
        self.last_log_time = self.get_clock().now()


    def pose_callback(self, msg):
        self.latest_pose = msg.pose
        self.publish_gas_reading()

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose
        self.publish_gas_reading()

    def mq2_callback(self, msg):
        self.latest_gas['mq2'] = msg.data
        self.publish_gas_reading()

    def mq3_callback(self, msg):
        self.latest_gas['mq3'] = msg.data
        self.publish_gas_reading()

    def mq9_callback(self, msg):
        self.latest_gas['mq9'] = msg.data
        self.publish_gas_reading()

    def mq135_callback(self, msg):
        self.latest_gas['mq135'] = msg.data
        self.publish_gas_reading()


    def publish_gas_reading(self):
        pose = self.latest_pose if self.latest_pose else self.latest_odom
        if pose is None:
            return

        msg = GasReadingStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        msg.mq2_percent = float(self.latest_gas['mq2'])
        msg.mq3_percent = float(self.latest_gas['mq3'])
        msg.mq9_percent = float(self.latest_gas['mq9'])
        msg.mq135_percent = float(self.latest_gas['mq135'])

        self.gas_pub.publish(msg)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds >= 2 * 1e9:
            self.last_log_time = now
            self.get_logger().info(
                f"[PUBLISH] x: {pose.position.x:.2f}, y: {pose.position.y:.2f}, "
                f"MQ2: {msg.mq2_percent}%, MQ3: {msg.mq3_percent}%, "
                f"MQ9: {msg.mq9_percent}%, MQ135: {msg.mq135_percent}%"
            )

def main(args=None):
    rclpy.init(args=args)
    node = GasSLAMFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

