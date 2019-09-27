#!/usr/bin/env python

import rospy
import sys
import numpy as np
import cv2 as cv
import math
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clever import srv
from std_srvs.srv import Trigger


image_pub = rospy.Publisher('/line_detection/image', Image, queue_size=1)
sobel_pub = rospy.Publisher('/line_detection/sobel', Image, queue_size=1)
flag_pub = rospy.Publisher('/line_detection/test_flag_image', Image, queue_size=1)

rospy.init_node('rect_detector')
bridge = CvBridge()

flags = glob.glob('flags/*.png')


def image_callback(data):
    print('callback')

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    img = take_rect(cv_image)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


def calculate_delta(delta_image):
    summary_delta = 0
    pixel_counter = 0

    for delta_row in delta_image:
        for delta_pixel in delta_row:
            summary_delta += (delta_pixel[0] + delta_pixel[1] + delta_pixel[2]) / 3
            pixel_counter += 1

    normalized_delta = summary_delta / pixel_counter

    if normalized_delta < 50:
        return True
    else:
        return False


def flag_compare(find_flag):
    for common_flag_name in flags:
        common_flag = cv.imread(common_flag_name, cv.COLOR_RGB2BGR)
        resize_common_flag = cv.resize(common_flag, (int(find_flag.shape[0]), int(find_flag.shape[1])))

        delta_image = find_flag - resize_common_flag
        if calculate_delta(delta_image):
            return common_flag_name

        flag_pub.publish(bridge.cv2_to_imgmsg(resize_common_flag, 'bgr8'))

    return False
    # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)


def take_rect(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    _img = cv.GaussianBlur(gray, (3, 3), 0)
    
    gradX = cv.Sobel(_img, ddepth=cv.CV_32F, dx=1, dy=0, ksize=3)
    gradY = cv.Sobel(_img, ddepth=cv.CV_32F, dx=0, dy=1, ksize=3)

    _img = cv.add(gradY, gradX)
    _img = cv.convertScaleAbs(_img)
    _img = cv.threshold(_img, 80, 255, cv.THRESH_BINARY)[1]

    sobel_pub.publish(bridge.cv2_to_imgmsg(_img, 'mono8'))
    contours = cv.findContours(_img.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[1]
        
    for cnt in contours:
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        area = int(rect[1][0]*rect[1][1])

        if 120 < area < 700:
            height = int(math.pow(math.pow(box[1][0] - box[0][0], 2) + math.pow(box[1][1] - box[0][1], 2), 0.5))
            wight = math.pow(math.pow(box[2][0] - box[1][0], 2) + math.pow(box[2][1] - box[1][1], 2), 0.5)

            # cv.drawContours(img, [box], 0, (255, 0, 0), 2)
            croped_flag = img[box[1][0]: int(box[1][0] + height), box[1][1]: int(box[1][1] + wight)]
            detected_flag = flag_compare(croped_flag)

            if detected_flag:
                cv.drawContours(img, [box], 0, (255, 0, 0), 2)
                # cv.putText(img, str(detected_flag), (box[1][0], box[1][1] - 15), )

    return img


print("Subscribe start")
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

while not rospy.is_shutdown():
    rospy.sleep(0.2)

