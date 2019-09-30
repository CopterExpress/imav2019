#!/usr/bin/env python

import os

import rospy
import cv2 as cv
import numpy as np
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flag_detector.msg import Flag

rospy.init_node('flag_detector')

debug_pub = rospy.Publisher('~debug', Image, queue_size=1)
flag_pub = rospy.Publisher('~flag', Flag, queue_size=1)
flag_msg = Flag()

method = cv.TM_CCOEFF_NORMED
threshold = 0.6
bridge = CvBridge()


images = []
w, h = 115, 73

flags = glob.glob("flags/*.png")
for flag in flags:
    images.append({
        'img': cv.resize(cv.imread(flag), (w, h)),
        'name': flag[6:-4]
    })


def image_callback(msg):
    if flag_pub.get_num_connections() == 0 and debug_pub.get_num_connections() == 0:
        return

    img = bridge.imgmsg_to_cv2(msg, 'bgr8')

    publish_debug = debug_pub.get_num_connections()
    if publish_debug:
        debug = img.copy()

    for flag in images:
        res = cv.matchTemplate(img, flag['img'], method)
        loc = np.where(res >= threshold)
        for pt in zip(*loc[::-1]):
            if debug_pub.get_num_connections():
                cv.rectangle(debug, pt, (pt[0] + w, pt[1] + h), (255, 0, 0), 2)
                cv.putText(debug, flag['name'], pt, cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0))
            flag_msg.header.stamp = msg.header.stamp
            flag_msg.country = flag['name']
            flag_pub.publish(flag_msg)

    if publish_debug:
        debug_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


image_sub = rospy.Subscriber('front_camera/image_raw', Image, image_callback, queue_size=1)
rospy.loginfo('Flag detector: ready')
rospy.spin()
