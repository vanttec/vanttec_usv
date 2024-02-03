#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from usv_interfaces.msg import Zbbox, ZbboxArray
from std_srvs.srv import SetBool
import pyzed.sl as sl

class Shapes(Node):
    def __init__(self):
        super().__init__('shapes')
        self.subscription = self.create_subscription(Image, '/bebblebrox/video', self.frame, 10)
        self.subscription

        self.publisher_ = self.create_publisher(ZbboxArray, '/shapes/detections', 10)
        self.bridge = CvBridge()

        self.state_srv = self.create_service(SetBool, '/shapes/state', self.set_node_state)

        self.should_run = False

        self.get_logger().info('-> shapes ready')
        
    def set_node_state(self, request, response):
        self.should_run = request.data

        self.get_logger().info('set detector node state: %b' % (request.data))

        return response

    def bgr2hsv(self, b,g,r, rangeChange = 10):
        '''Convierte un color bgr a hsv'''
        color = np.uint8([[[b, g, r]]]) 
        hsvColor = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        
        lowerLimit = hsvColor[0][0][0] - rangeChange, 100, 100
        upperLimit = hsvColor[0][0][0] + rangeChange, 255, 255

        return lowerLimit, upperLimit

    def createMask(self, frame, b, g, r, rangeChange = 10):
        '''Crea una mascara de acuerdo al color en bgr que se le de'''

        color_low_limit, color_upper_limit = self.bgr2hsv(b, g, r, rangeChange)
        lcolor = np.array(color_low_limit)
        ucolor = np.array(color_upper_limit)
        mask = cv2.inRange(frame, lcolor, ucolor)
        kernel = np.ones((3,3), np.uint8) 
        mask = cv2.erode(mask, kernel)

        return mask

    def find_shape(self, masks):
        msg = ZbboxArray()

        if self.should_run == False:
            return msg

        for color_idx, mask_color in enumerate(masks):

            contours, _ = cv2.findContours(mask_color, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours

            for cnt in contours:
                area_cnt = cv2.contourArea(cnt)
                if area_cnt > 200:
                    perimeter = cv2.arcLength(cnt, True)
                    approx_cnt = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
                    x, y, w, h = cv2.boundingRect(approx_cnt)
                    center_x, center_y = int(x + w / 2), int(y + h / 2)

                    area_circle = np.pi * (w / 2) ** 2
                    circularity = area_cnt / area_circle

                    # format
                    # 0xABC
                    # 0xABC & 0xF00 = type
                    # 0xABC & 0x0F0 = shape
                    # 0xABC & 0x00F = color
                        # red 0
                        # blue 1
                        # green 2
                        # yellow 3
                    
                    lbl = 0x200

                    if 0.95 < circularity < 1.05: # circle
                        lbl += 0x10
                    elif len(approx_cnt) == 4: # rectangle
                        lbl += 0x20
                    elif len(approx_cnt) == 3: # triangle
                        lbl += 0x30
                    elif len(approx_cnt) == 12: # plus
                        lbl += 0xF0
                    
                    lbl += color_idx 

                    msg.boxes.append(Zbbox(
                        x0 = x,
                        y0 = y,
                        x1 = x+w,
                        y1 = y+h,
                        prob = 1,
                        label = lbl,
                        uuid = sl.generate_unique_id(),
                    ))
                    
                    # cv2.drawContours(frame, [approx_cnt], 0, (0, 0, 0), 5)
                    # cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                    #
                    # cv2.putText(frame, f" {shape}", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                    # print(f"{shape} detected at: ({x}, {y})")

        return msg

    def frame(self, msg):

        self.get_logger().info('shape detection done')

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.blue_mask = self.createMask(self.hsv_frame, 255, 0, 0, 20)
        self.red_mask = self.createMask(self.hsv_frame, 0, 0, 255, 20)
        self.green_mask = self.createMask(self.hsv_frame, 0, 255, 0, 20)
        self.yellow_mask = self.createMask(self.hsv_frame, 0, 255, 255, 20)

        self.mask_list = [self.red_mask, self.green_mask, self.blue_mask, self.yellow_mask]
        
        objs = self.find_shape(self.mask_list)
        objs.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(objs)

def main(args=None):

    rclpy.init(args=args)

    shapes = Shapes()

    rclpy.spin(shapes)

    shapes.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

