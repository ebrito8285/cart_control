import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickToCmdVelNode(Node):
    def __init__(self):
        super().__init__('joystick_to_cmd_vel')
        self.subscription = self.create_subscription(
            Joy,
            'joy',  # Topic to which the joystick data is published
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()

        # Assuming axes[1] is the forward/backward axis and axes[0] is the left/right axis
        # Adjust indices and signs depending on your joystick configuration
        twist.linear.x = msg.axes[1]  # Forward/Backward
        twist.angular.z = msg.axes[0]  # Left/Right rotation

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    joystick_to_cmd_vel_node = JoystickToCmdVelNode()
    rclpy.spin(joystick_to_cmd_vel_node)
    joystick_to_cmd_vel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()