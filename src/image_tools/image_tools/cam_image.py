#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from image_tools.devices import Camera

class Cam2Image(Node):
    def __init__(self):
        #
        super().__init__("cam2image")

        #
        self.pub_img = self.create_publisher(Image, "image_raw", 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Setting camera
        self.declare_parameter("COM", 0)
        COM = self.get_parameter("COM").get_parameter_value().integer_value
        self.cam = Camera(COM)
        self.get_logger().info("Initialization of cam successfully")

    def timer_callback(self):
        img = self.cam.get_img()
        if not(img is None):
            msg_img = CvBridge().cv2_to_imgmsg(img)
            self.pub_img.publish(msg_img)


def main():
    rclpy.init()
    node = Cam2Image()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()