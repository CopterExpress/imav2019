import os

import rospy
import cv2 as cv
import numpy as np
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flag_detector.mag import Flag

image_pub = rospy.Publisher('/line_detection/image', Image, queue_size=1)
flag_message = rospy.Publisher('/detected_flag', Flag, queue_size=1)

rospy.init_node('flag_detector')


method = cv.TM_CCOEFF_NORMED
threshold = 0.6
bridge = CvBridge()


images = []
w, h = 115, 73

flags = glob.glob("flags/*.png")
for flag in flags:
    images.append({
        'img': cv.resize(cv.imread(flag), (w, h)),
        'name': flag
    })


def detect_image(image):
    img = image.copy()

    for flag in images:
        res = cv.matchTemplate(img, flag['img'], method)
        loc = np.where(res >= threshold)
        for pt in zip(*loc[::-1]):
            cv.rectangle(image, pt, (pt[0] + w, pt[1] + h), (255, 0, 0), 2)
            cv.putText(image, flag['name'], pt, cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0))
            flags = Flag(
                header=''
                country=''
            )
            flag_message.publish()
            break

    return image


def image_callback(data):
    print('callback')
    if image_pub.get_num_connections() == 0:
        return
    else
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

        img = detect_image(cv_image)
        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


if __name__ == "__main__":
    print("Subscribe start")
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
    rospy.spin()
