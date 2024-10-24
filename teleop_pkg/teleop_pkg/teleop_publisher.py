import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg

class TeleopPublisher(Node):

	def __init__(self):
		super().__init__('teleop_publisher')
		self.wheel_publisher_ = self.create_publisher(ServoCtrlMsg, '/ctrl_pkg/servo_msg', 10)
		self.timer = self.create_timer(1.0, self.timer_callback)

	def timer_callback(self):
		msg = ServoCtrlMsg()
		msg.throttle = 0.5
		msg.angle = 0.0
		self.wheel_publisher_.publish(msg)
		self.get_logger().info(f'Publishing: throttle={msg.throttle}, angle={msg.angle}')
		
def main(args=None):
	rclpy.init(args=args)
	
	teleop_publisher = TeleopPublisher()
	
	rclpy.spin(teleop_publisher)
	
	teleop_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
