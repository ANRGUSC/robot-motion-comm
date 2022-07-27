import rclpy # ROS2 python library
from rclpy.node import Node

from std_msgs.msg import String # ros2 message type, to send over a topic

# create my own node class that inherits from Node
class MyNode_Pub(Node):
	def __init__(self):
		super().__init__("my_node_pub3")
		self.create_timer(1, self.publish_msg) 
		self.pub = self.create_publisher(String, "my_topic", 10)
		self.i = 1

	def publish_msg(self):
		msg = String()
		msg.data = input('HELLO!!! %d' % self.i)
		self.pub.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg.data)
		self.i+=1

def main():
	# initialize ros2 communications
	rclpy.init()
	# create a node
	node = MyNode_Pub()
	# spin the node, execution pauses here while the node
	rclpy.spin(node)
	# kill the node
	rclpy.shutdown()


if __name__ == '__main__':
	main()
