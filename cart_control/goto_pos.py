#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import apriltag
from cv_bridge import CvBridge

class GoToPosNode(Node):
    def __init__(self):
        super().__init__('goto_pos_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        self.target_x = 320  
        self.target_y = 240  
        self.aligning_phase = True
        self.initial_spin_complete = False 
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)

        if not self.initial_spin_complete:
            # Publish an initial Twist message to make the robot spin in place
            twist = Twist()
            twist.angular.z = 0.2
            self.publisher_.publish(twist)
            rclpy.sleep(1.0)
            self.initial_spin_complete = True

        for tag in tags:
            # Compute the error between the tag's position and target position
            error_x = tag.center[0] - self.target_x
            error_y = tag.center[1] - self.target_y

            twist = Twist()

            if self.aligning_phase:
                if abs(error_x) > 10:  # Threshold to consider as aligned
                    twist.angular.z = -error_x * 0.01
                else:
                    self.aligning_phase = False
            else:
                if abs(error_y) > 10:
                    twist.linear.x = -error_y * 0.01

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    goto_pos_node = GoToPosNode()
    rclpy.spin(goto_pos_node)
    goto_pos_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()