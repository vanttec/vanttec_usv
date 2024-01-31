import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from usv_interfaces.msg import Zbbox, ZbboxArray

class Shapes(Node):
    def __init__(self):
        super().__init__('shapes')
        self.subscription = self.create_subscription(Image, '/bebblebrox/video', self.frame, 10)
        self.subscription

        self.publisher_ = self.create_publisher(ZbboxArray, '/shapes/detections', 10)
        self.bridge = CvBridge()

        self.hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.blue_mask = createMask(hsv_frame, 255, 0, 0, 20)
        self.red_mask = createMask(hsv_frame, 0, 0, 255, 20)
        self.green_mask = createMask(hsv_frame, 0, 255, 0, 20)
        self.yellow_mask = createMask(hsv_frame, 0, 255, 255, 20)

        self.mask_list = [blue_mask, red_mask, green_mask, yellow_mask]

    def find_shape(masks):
        contours, _ = cv2.findContours(mask_color, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours

        msg = ZbboxArray()

        for mask_color in masks:
            for cnt in contours:
                area_cnt = cv2.contourArea(cnt)
                if area_cnt > 200:
                    perimeter = cv2.arcLength(cnt, True)
                    approx_cnt = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
                    x, y, w, h = cv2.boundingRect(approx_cnt)
                    center_x, center_y = int(x + w / 2), int(y + h / 2)

                    area_circle = np.pi * (w / 2) ** 2
                    circularity = area_cnt / area_circle

                    shape = None
                    if 0.95 < circularity < 1.05:
                        shape = "Circle"
                    elif len(approx_cnt) == 4:
                        shape = "Rectangle"
                    elif len(approx_cnt) == 3:
                        shape = "Triangle"
                    elif len(approx_cnt) == 12:
                        shape = "Plus"

                    if shape != None:
                        msg.boxes.append(Zbbox(
                            x0 = x,
                            y0 = y,
                            x1 = x+w,
                            y1 = y+h,
                            prob = 1, # ?
                            label = 0xBEEF,
                            uuid = shape,
                        ))
                        
                        # cv2.drawContours(frame, [approx_cnt], 0, (0, 0, 0), 5)
                        # cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                        #
                        # cv2.putText(frame, f" {shape}", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0))
                        # print(f"{shape} detected at: ({x}, {y})")

        return detected_objects

    def frame(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        objs = find_shape(mask_list)
        objs.header = msg.header

        self.publisher.publish(objs)

    def bgr2hsv(b,g,r, rangeChange = 10):
        '''Convierte un color bgr a hsv'''
        color = np.uint8([[[b, g, r]]]) 
        hsvColor = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        
        lowerLimit = hsvColor[0][0][0] - rangeChange, 100, 100
        upperLimit = hsvColor[0][0][0] + rangeChange, 255, 255

        return lowerLimit, upperLimit

    def createMask(frame, b, g, r, rangeChange = 10):
        '''Crea una mascara de acuerdo al color en bgr que se le de'''

        color_low_limit, color_upper_limit = bgr2hsv(b, g, r, rangeChange)
        lcolor = np.array(color_low_limit)
        ucolor = np.array(color_upper_limit)
        mask = cv2.inRange(frame, lcolor, ucolor)
        kernel = np.ones((3,3), np.uint8) 
        mask = cv2.erode(mask, kernel)

        return mask

def main(args=None):

    rclpy.init(args=args)

    shapes = Shapes()

    rclpy.spin(shapes)

    shapes.destroy_node()
    rclpy.shutdown()
