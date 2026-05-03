import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
	def __init__(self):
		super().__init__('cmdvel_test_publisher')
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		self.timer = self.create_timer(1.0, self.publish_cmd)
		self.get_logger().info('Cmdvel Publisher started')

	def publish_cmd(self):
		msg = Twist()
		msg.linear.x = 0.2
		msg.linear.y = 0.5
		self.publisher_.publish(msg)

		self.get_logger().info(
			f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}'
		)

def main(args=None):
	rclpy.init(args=args)
	node = CmdVelPublisher()
	rclpy.spin(node)
	node.destory_node()
	rcply.shutdown()

if __name__ == 'main':
	main()
