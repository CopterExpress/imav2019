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


# ser = serial.Serial('usb-1a86_USB2.0-Serial-if00-port0', 115200, timeout=10)
# servo_angle = 30
# last_range = -1


# def _arduino_comms():
#     global last_range, servo_angle
#     while True:
#         last_range = int(ser.readline())
#         ser.write(str(servo_angle))


# thread.start_new_thread(_arduino_comms, [])


def qr_cb(msg):
    print 'qr', msg


qr_sub = rospy.Subscriber('qr_reader/qr', DetectedQr, qr_cb, queue_size=1,)


def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='', tolerance=0.2, auto_arm=False, timeout=rospy.Duration(0)):
    start = rospy.get_rostime()
    res = navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            print 'navigating finished'
            break
        if timeout and rospy.get_rostime() - start > timeout:
            print 'navigating timeout'
            break
        rospy.sleep(0.2)

    return res


def takeoff():
    print navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(5)


def scan():
    print 'scan'
    navigate(x=0, y=4, z=0, speed=0.4, frame_id='navigate_target')
    rospy.sleep(10)
    print 'scan back'
    navigate(x=0, y=-2, z=0.4, speed=0.4, frame_id='navigate_target')
    rospy.sleep(10)
    print 'fly to window'
    navigate(x=2, y=0, z=0, speed=0.5, frame_id='navigate_target')
    rospy.sleep(7)
    print 'land'
    land()


def fly_to_shelf():
    print navigate(x=0, y=0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(3)
    print navigate(x=4, y=0, z=0, speed=0.5, frame_id='navigate_target')
    rospy.sleep(10)
    print navigate(x=0, y=0, z=0, yaw=math.pi, speed=0.5, frame_id='navigate_target')
    rospy.sleep(3)
    # print navigate(x=0, y=7.5, z=0, speed=0.5, frame_id='navigate_target')


def wait_aruco(_id):
    print 'waiting aruco %d' % _id
    while not rospy.is_shutdown():
        telem = get_telemetry('aruco_%d' % _id)
        if not math.isnan(telem.x):
            print 'got aruco %d' % _id
            break
        rospy.sleep(0.2)

def mission():
    print 'takeoff'
    print navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(5)

    print 'fly left'
    navigate(x=0, y=5, z=0, speed=0.2, frame_id='navigate_target')

    wait_aruco(3)

    print 'fly to aruco'
    navigate(x=0, y=0, z=1, speed=0.3, frame_id='aruco_3')
    rospy.sleep(8)

    print 'fly down'
    navigate(x=0, y=0, z=-0.5, speed=0.3, frame_id='navigate_target')
    rospy.sleep(4)

    scan()



# print 'take off'
# takeoff()
mission()

#fly_to_shelf()

# rospy.spin()
