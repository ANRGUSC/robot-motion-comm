# The person (pub3), types in the message (publishes) to the robot (sub2, subscriber)
# The robot (sub2), converts the message to binary and moves according to the binary message
# Another person (someothername) reads in the bin msg from the video stream and converts it back to the original message

import rclpy # ROS2 python library
import time
from rclpy.node import Node

from std_msgs.msg import String # ros2 message type, to send over a topic
from geometry_msgs.msg import Twist # ros message type (Twist)

# create my own node class that inherits from Node
class MyNode_Sub(Node):
	def __init__(self):
		super().__init__("my_node_sub")
		self.cmd_vel_pub = self.create_publisher(Twist, '/tb3/cmd_vel', 10)
		self.sub = self.create_subscription(String, "my_topic", self.listener_callback, 10)
		self.sub # prevent unused var warning

	def listener_callback(self, msg):
		self.get_logger().info('I heard "%s"' % msg.data)
		bin_msg = ''.join(format(ord(i), '08b') for i in msg.data)
		self.get_logger().info('BINARY "%s"' % bin_msg)
		
		self.get_logger().info("START")
		self.start()
		
		prev = '0'
		for curr in bin_msg:
			if curr == '0' and prev == '1':
				z = 0.2 #BWD
			elif curr == '1' and prev == '0':
				z = -0.2  #FWD
			else:
				z = 0.0	 #no change
			self.move(z)
			prev = curr
			
		self.get_logger().info("STOP")
		self.stop()
	
	def move(self, speed):
		twist = Twist()
		if speed == 0.2:
			self.get_logger().info("BWD, TOWARDS 0")
		elif speed == -0.2:
			self.get_logger().info("FWD, TOWARDS 1")
		else:
			self.get_logger().info("Same spot")
		twist.linear.x = speed
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.5)
		twist.linear.x = 0.0
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.5)
		
	def start(self):
		twist = Twist()
		twist.linear.x = 0.2
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.5)
		twist.linear.x = -0.2
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.5)
		
		twist.linear.x = 0.0
		self.cmd_vel_pub.publish(twist)
		

	def stop(self):
		# rotate
		twist = Twist()
		twist.angular.z = -2.5
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.25)
		# move up, not side to side
		twist.angular.z = 0.0
		self.cmd_vel_pub.publish(twist)
		twist.linear.x = 0.2
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.25)
		# stop for reception to detect change in postion
		twist.linear.x = 0.0
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.5)
		# move back to original spot
		twist.linear.x = -0.2
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.25)
		twist.linear.x = 0.0
		self.cmd_vel_pub.publish(twist)
		twist.angular.z = 2.5
		self.cmd_vel_pub.publish(twist)
		time.sleep(0.25)
		twist.angular.z = 0.0
		self.cmd_vel_pub.publish(twist)

def main():
	# initialize ros2 communications
	rclpy.init()
	# create a node
	node = MyNode_Sub()
	# spin the node, execution pauses here while the node
	rclpy.spin(node)
	# kill the node
	rclpy.shutdown()


if __name__ == '__main__':
	main()
