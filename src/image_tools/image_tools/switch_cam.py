#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from image_tools.devices import Camera

class SwitchCam(Node):
    def __init__(self):
        #
        super().__init__("switch_cam")

        #
        self.pub_img = self.create_publisher(Image, "image_raw", 10)
        self.sub_switch = self.create_subscription(
            Bool, "switch_cam", self.switch_callback, 10
        )

        # Setting camera
        self.declare_parameter("COM", 0)
        COM = self.get_parameter("COM").get_parameter_value().integer_value
        self.cam = Camera(COM)
        self.img_bri = CvBridge().cv2_to_imgmsg
        self.get_logger().info("Initialization of cam successfully")

    def switch_callback(self, msg: Bool):
        if msg.data:
            img = self.cam.get_img()
            if not(img is None):
                self.pub_img.publish(self.img_bri(img))
                self.get_logger().info("Send to topic /image_raw")


def main():
    rclpy.init()
    node = SwitchCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()