#!/usr/bin/env python

import math
import rospy
import re
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


ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200, timeout=10)
servo_angle = 30
last_range = -1


# def _arduino_comms():
#     global last_range, servo_angle
#     while True:
#         # last_range = fl/oat(ser.readline())
#         ser.write(str(servo_angle) + '\n')
#         rospy.sleep(0.1)


def handle(pos):
    global servo_angle
    if pos == 'down':
        servo_angle = 100
    elif pos == 'keep':
        servo_angle = 80
    elif pos == 'drop':
        servo_angle = 175
    elif pos == 'back':
        servo_angle = 30
    else:
        raise Exception('incorrect position')


# thread.start_new_thread(_arduino_comms, ())


def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


packages = []
found_packages = {}


def qr_cb(msg):
    if msg.qr_message in packages:
        print '!! found package', msg.qr_message
        # telem = get_telemetry()
        # if not msg.qr_message in found_packages:
        #     found_packages[msg.qr_message] = {}
        # found_packages[msg.qr_message]['x'] = telem.x
        # found_packages[msg.qr_message]['y'] = telem.y
        # found_packages[msg.qr_message]['z'] = telem.z
    else:
        print 'skip package', msg.qr_message



current_shelf = ''


def shelf_cb(msg):
    if not current_shelf:
        return
    r = re.compile(r'\d{2}' + current_shelf, re.IGNORECASE)
    if r.match(msg.text):
        print 'found shelf number %s' % msg.text
        telem = get_telemetry()
        for package in packages:
            details = packages[package]
            dist = get_distance(telem.x, telem.y, telem.z, details['x'], details['y'], details['z'])
            if dist < 1:
                print 'package %s now belongs to shelf %s' % (package, msg.text)
                details['shelf'] = msg.text


def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='', yaw=0, tolerance=0.2, auto_arm=False, timeout=rospy.Duration(0)):
    start = rospy.get_rostime()
    res = navigate(x=x, y=y, z=z, speed=speed, yaw=yaw, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            print 'navigating finished'
            break
        if timeout and ((rospy.get_rostime() - start) > timeout):
            print 'navigating timeout'
            break
        rospy.sleep(0.1)

    return res


def read_csv():
    import csv
    with open('packages.csv') as csvfile:
        reader = csv.reader(csvfile)
        return list(row[0] for row in reader)


def takeoff():
    print navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(5)


def scan():
    print 'start scanning'
    print 'go left'
    navigate_wait(x=0, y=4.7, z=0, speed=0.3, frame_id='navigate_target')
    print 'go up'
    navigate_wait(x=0, y=0, z=2, speed=0.3, frame_id='navigate_target')

    wait_aruco(122)
    navigate_wait(x=0, y=0, z=2.5, speed=0.3, frame_id='aruco_122')

    print 'go right'
    navigate_wait(x=0, y=-4.5, z=0, speed=0.3, frame_id='navigate_target')

    wait_aruco(120)
    navigate_wait(x=0, y=0, z=2.5, speed=0.3, frame_id='aruco_120')

    print 'fly through window'
    navigate_wait(x=3, y=0, z=0, speed=0.5, frame_id='navigate_target')


def fly_to_shelf():
    print 'fly to shelf'
    navigate_wait(x=4, y=0, z=0, speed=0.5, frame_id='navigate_target', timeout=rospy.Duration(8))
    print 'adjust yaw'
    navigate(x=0, y=0, z=0, yaw=math.pi, speed=0.5, frame_id='navigate_target')
    rospy.sleep(3)


def wait_aruco(_id):
    print 'waiting aruco %d' % _id
    while not rospy.is_shutdown():
        telem = get_telemetry('aruco_%d' % _id)
        if not math.isnan(telem.x):
            print 'got aruco %d' % _id
            break
        rospy.sleep(0.2)


def pick_payload():
    handle('down')
    wait_aruco(45)
    navigate_wait(x=0, y=0, z=0.91, speed=0.3, yaw=math.pi, frame_id='aruco_45', tolerance=0.1)
    print 'backwards!'
    navigate(x=-0.7, speed=1.4, frame_id='navigate_target')
    rospy.sleep(2)
    navigate(z=0.2, speed=0.5, frame_id='navigate_target')
    handle('keep')


def land_to_target():
    wait_aruco(3)
    navigate_wait(x=0, y=0, z=1, speed=0.3, frame_id='aruco_3')
    land()


def mission():
    handle('back')

    print 'takeoff'
    print navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True, timeout=rospy.Duration(5))

    print navigate_wait(y=-1, speed=0.5, frame_id='navigate_target', timeout=rospy.Duration(2))

    fly_to_shelf()

    print 'search for marker'
    navigate(x=0, y=3, z=0, speed=0.2, frame_id='navigate_target')

    wait_aruco(120)

    print 'fly to aruco 120'
    navigate_wait(x=0, y=0, z=1, speed=0.3, frame_id='aruco_120')

    print 'fly down'
    navigate_wait(x=0, y=0, z=-0.3, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(1))

    global current_shelf
    current_shelf = 'A'
    qr_sub = rospy.Subscriber('qr_reader/qr', DetectedQr, qr_cb, queue_size=1)

    scan()

    qr_sub.unregister()


packages = read_csv()
print 'got packages', packages

# ensure get_telemetry works
print get_telemetry()

# print 'takeoff'
# print navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True, timeout=rospy.Duration(5))
# pick_payload()

# print 'take off'
# takeoff()
mission()
# print 'tkof'
# print navigate(z=1.5, frame_id='body', auto_arm=True)
# wait_aruco(0)
# navigate(x=0, y=0, z=1, speed=0.3, frame_id='aruco_0')

# print 'finish'

#fly_to_shelf()

# rospy.spin()
