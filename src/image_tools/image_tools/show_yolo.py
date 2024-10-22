#! /usr/bin/python3

import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2DArray

class ShowYolo(Node):
    def __init__(self):
        #
        super().__init__("show_yolo")

        #
        self.sub_img = self.create_subscription(
            Image, "image_raw", self.image_callback, 10
        )
        self.sub_boxes = self.create_subscription(
            Detection2DArray, "detection_boxes", self.detection_boxes_callback, 10
        )

        #
        self.boundingbox = []
        self.img_bri = CvBridge().imgmsg_to_cv2

    def detection_boxes_callback(self, msg: Detection2DArray):
        self.boundingbox = []
        if len(msg.detections) > 0:
            for box in msg.detections:
                self.boundingbox.append(box.bbox)
            
    def image_callback(self, msg: Image):
        frame = self.img_bri(msg)
        if len(self.boundingbox) > 0:
            for box in self.boundingbox:
                center_x = int(box.center.position.x)
                center_y = int(box.center.position.y)
                h_size_x = int(box.size_x / 2)  
                h_size_y = int(box.size_y / 2)
                start_point = (center_x - h_size_x, center_y + h_size_y)
                end_point = (center_x + h_size_x, center_y - h_size_y)
                frame = cv2.rectangle(frame, start_point, end_point, (255, 0, 0), 2)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1): pass



def main():
    rclpy.init()
    node = ShowYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()