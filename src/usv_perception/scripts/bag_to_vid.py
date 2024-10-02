#!/usr/bin/env python3
import cv2

from ament_index_python.packages import get_package_share_directory
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BagToVidNode(Node):

    def __init__(self):
        super().__init__('bag_to_vid_node')

        self.bridge = CvBridge()
        self.vid_sub_ = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.process_img, 10
        )
        self.count = 0
        self.interval = 1
    
    def process_img(self, msg):
        self.count+=1
        if self.count % self.interval == 0:
            img = self.bridge.imgmsg_to_cv2(msg)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            print("...")
            # cv2.imshow("imgg",img)
            cv2.imwrite("/home/max/splatting/inspeccion1/data/"+str(int(self.count/self.interval))+".png", img)

def main(args=None):
    rclpy.init(args=args)

    bag_to_vid_node = BagToVidNode()
    rclpy.spin(bag_to_vid_node)
    bag_to_vid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()