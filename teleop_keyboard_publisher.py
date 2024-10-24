import rclpy
from rclpy.node import Node

import keyboard
from deepracer_interfaces_pkg.msg import ServoCtrlMsg


class TeleopPublisher(Node):

    def __init__(self):
        super().__init__('teleop_publisher')
        self.publisher_ = self.create_publisher(ServoCtrlMsg, '/ctrl_pkg/servo_msg', 10)

        # Initialize default throttle and angle values
        self.throttle = 0.0
        self.angle    = 0.0
        
        # Bind keypress events to functions
        # Throttle: 'w' to go forward, 's' to go backwards
        keyboard.on_press_key('w', self.forward)
        keyboard.on_press_key('s', self.reverse)
        keyboard.on_release_key('w', self.stop_throttle)
        keyboard.on_release_key('s', self.stop_throttle)
        # Angle: 'a' to turn left, 'd' to turn right
        keyboard.on_press_key('a', self.turn_left)
        keyboard.on_press_key('d', self.turn_right)
        keyboard.on_release_key('a', self.straighten_angle)
        keyboard.on_release_key('d', self.straighten_angle)

    def forward(self, event):
        self.throttle = 0.5
        self.publish_message()

    def reverse(self, event):
        self.throttle = -0.5
        self.publish_message()

    def stop_throttle(self, event):
        self.throttle = 0.0
        self.publish_message()

    def turn_left(self, event):
        self.angle = -0.5
        self.publish_message()

    def turn_right(self, event):
        self.angle = 0.5
        self.publish_message()

    def straighten_angle(self, event):
        self.angle = 0.0
        self.publish_message()

    def publish_message(self):
        msg = ServoCtrlMsg()
        msg.throttle = self.throttle
        msg.angle    = self.angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: throttle={msg.throttle}, angle={msg.angle}')


def main(args=None):
    rclpy.init(args=args)

    teleop_publisher = TeleopPublisher()

    rclpy.spin(teleop_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
