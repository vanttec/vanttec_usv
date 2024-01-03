#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class ImagePublisher(Node):
    def __init__(self, file_path):
        super().__init__("image_publisher")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(file_path)
        self.pub = self.create_publisher(Image, "/video", 10)

    def run(self):
        while(self.cap.isOpened()):
            ret, frame = self.cap.read() 
            if ret:
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            time.sleep(0.03) # ~30ms

        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    data_folder = os.path.join(get_package_share_directory('usv_perception'), 'data')
    video_f = os.path.join(data_folder, 'video.mp4')

    ip = ImagePublisher(video_f)
    print("Publishing...")
    ip.run()

    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
