
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPathNode(Node):
	def __init__(self):
		super().__init__('odom_to_path')
		self.declare_parameter('odom_topic', '/odom')
		self.declare_parameter('path_topic', '/path')
		self.declare_parameter('frame_id', 'odom')

		odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
		path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
		self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

		self.path_pub = self.create_publisher(Path, path_topic, 10)
		self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
		self.path_msg = Path()
		self.path_msg.header.frame_id = self.frame_id

	def odom_callback(self, msg: Odometry):
		pose = PoseStamped()
		pose.header = msg.header
		pose.pose = msg.pose.pose
		self.path_msg.header.stamp = msg.header.stamp
		self.path_msg.poses.append(pose)
		self.path_pub.publish(self.path_msg)

def main(args=None):
	rclpy.init(args=args)
	node = OdomToPathNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
