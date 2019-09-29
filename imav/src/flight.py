#!/usr/bin/env python

import math
import rospy
from clever import srv
from std_srvs.srv import Trigger
from zbar_ros_redux.msg import DetectedQr

# Servo interaction
import serial
import thread

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


ser = serial.Serial('usb-1a86_USB2.0-Serial-if00-port0', 115200, timeout=10)
servo_angle = 30
last_range = -1


def _arduino_comms():
    global last_range, servo_angle
    while True:
        last_range = int(ser.readline())
        ser.write(str(servo_angle))


thread.start_new_thread(_arduino_comms, [])


def qr_cb(msg):
    print 'qr', msg


qr_sub = rospy.Subscriber('qr_reader/qr', DetectedQr, qr_cb, queue_size=1,)


def scan():
    print navigate(x=0, y=0, z=0.7, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(5)
    print navigate(x=0, y=4, z=0, speed=0.5, frame_id='navigate_target')
    rospy.sleep(10)
    print navigate(x=0, y=-2, z=0.4, speed=0.5, frame_id='navigate_target')
    rospy.sleep(10)
    print navigate(x=2, y=0, z=0, speed=0.5, frame_id='navigate_target')
    rospy.sleep(5)
    land()


def fly_to_shelf():
    print navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(3)
    print navigate(x=4, y=0, z=0, speed=0.5, frame_id='navigate_target')
    rospy.sleep(10)
    print navigate(x=0, y=0, z=0, yaw=math.pi, speed=0.5, frame_id='navigate_target')
    rospy.sleep(3)
    print navigate(x=0, y=7.5, z=0, speed=0.5, frame_id='navigate_target')


#fly_to_shelf()

# rospy.spin()
