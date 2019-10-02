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
found_shelves = {}


# def save_shelve(name, qr=True):
#     print 'Found shelve', name
#     if not name in found_shelves:
#         found_shelves[name] = {}
#     try:
#         telem = get_telemetry()
#         found_shelves[name]['x'] = telem.x
#         found_shelves[name]['y'] = telem.y
#         found_shelves[name]['z'] = telem.z
#     except Exception as e:
#         print e


# def qr_cb(msg):
#     if msg.qr_message in packages:
#         print '!!! found package: ', msg.qr_message
#         try:
#             telem = get_telemetry()
#             if not msg.qr_message in found_packages:
#                 found_packages[msg.qr_message] = {}
#             found_packages[msg.qr_message]['x'] = telem.x
#             found_packages[msg.qr_message]['y'] = telem.y
#             found_packages[msg.qr_message]['z'] = telem.z
#             print found_packages
#         except Exception as e:
#             print e
#     else:
#         print 'skip package', msg.qr_message


def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='', yaw=0, tolerance=0.2, auto_arm=False, timeout=rospy.Duration(0)):
    start = rospy.get_rostime()
    res = navigate(x=x, y=y, z=z, speed=speed, yaw=yaw, frame_id=frame_id, auto_arm=auto_arm)
    if not res.success:
        print res
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


def wait_aruco(_id, timeout=rospy.Duration(0)):
    print 'wait aruco %d' % _id
    start = rospy.get_rostime()
    while not rospy.is_shutdown():
        telem = get_telemetry('aruco_%d' % _id)
        if not math.isnan(telem.x):
            print 'got aruco %d' % _id
            break
        if timeout and ((rospy.get_rostime() - start) > timeout):
            print 'wait aruco %d timeout' % _id
            break
        rospy.sleep(0.2)


def navigate_and_wait_aruco(_id, x=0, y=0, z=0, yaw=0, speed=0.5, frame_id='', timeout=rospy.Duration(0)):
    print 'navigate and wait for aruco %d' % _id
    res = navigate(x=x, y=y, z=z, speed=speed, yaw=yaw, frame_id=frame_id)
    if not res.success:
        print res
        return res
    wait_aruco(_id=_id, timeout=timeout)
    return res


current_line = ''
current_shelf = 0

shelf_search_enabled = False
package_search_enabled = False


# for testing
# found_packages = {'Y864H': {'shelf':1}, 'C933S': {'shelf':2}, 'G853F': {'shelf':5}}
# found_shelves = {1: {'code':'A21'}, 5: {'code':'A24'}}


def print_current_search_results():
    print 'packages', found_packages
    print 'shelves', found_shelves
    for package in found_packages:
        shelf = found_packages[package]['shelf']
        if shelf in found_shelves:
            print '=== package:', package, 'shelf: ', found_shelves[shelf]['code'], '==='
        else:
            print '=== package:', package, 'shelf: ', shelf, '==='


def is_shelf_id(text):
    r = re.compile(r'\d{2}' + current_line, re.IGNORECASE)
    if r.match(text):
        return True


def qr_cb(msg):
    if is_shelf_id(msg.qr_message):
        if shelf_search_enabled:
            # if current_shelf in found_shelves:
            #     # set the next shelf
            #     print 'found next shelf code', current_shelf+1, msg.qr_message
            #     found_shelves[current_shelf+1] = {'code': msg.qr_message}
            # else:
            if current_shelf not in found_shelves:
                print 'found shelf code', current_shelf, msg.qr_message
                found_shelves[current_shelf] = {'code': msg.qr_message}

                print_current_search_results()

    elif msg.qr_message in packages:
        if package_search_enabled:
            print '!!! found package: ', msg.qr_message, 'shelf: ', current_shelf
            found_packages[msg.qr_message] = {'shelf': current_shelf}

            print_current_search_results()
        else:
            print 'found package: ', msg.qr_message, 'search disabled'

    else:
        print 'skip package', msg.qr_message


def read_csv(index):
    import csv
    with open('packages_demo.csv') as csvfile:
        reader = csv.reader(csvfile)
        return list(reader)[5][1:]


def takeoff():
    print navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(5)


def scan():
    global current_line
    current_line = 'A'

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


LOWER_Z = .55
DEADZONE_LOWER_Z = 1.35
DEADZONE_UPPER_Z = 1.9
UPPER_Z = 3.5

BETWEEN_SHELVES = 1.6


def scan_up(current_z=LOWER_Z, aruco=None):
    global current_shelf, shelf_search_enabled, package_search_enabled

    current_shelf += 1

    print 'fly to lower z'
    navigate_wait(z=LOWER_Z-current_z, speed=0.3, frame_id='navigate_target')

    shelf_search_enabled = True
    package_search_enabled = True

    print 'fly to deadzone lower z'
    navigate_wait(z=DEADZONE_LOWER_Z-LOWER_Z, speed=0.3, frame_id='navigate_target')

    package_search_enabled = False
    current_shelf += 1

    print 'fly to deadzone upper z'
    navigate_wait(z=DEADZONE_UPPER_Z-DEADZONE_LOWER_Z, speed=0.3, frame_id='navigate_target')

    package_search_enabled = True

    print 'fly to upper z'
    if aruco:
        navigate_and_wait_aruco(_id=aruco, z=UPPER_Z-DEADZONE_UPPER_Z, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(8))
        navigate_wait(z=UPPER_Z, speed=0.3, frame_id='aruco_'+str(aruco), timeout=rospy.Duration(5))
    else:
        navigate_wait(z=UPPER_Z-DEADZONE_UPPER_Z, speed=0.3, frame_id='navigate_target')

    package_search_enabled = False
    shelf_search_enabled = False


def scan_down(aruco=None):
    global current_shelf, shelf_search_enabled, package_search_enabled

    current_shelf += 1

    if aruco:
        navigate_wait(x=0, y=0, z=UPPER_Z, speed=0.3, frame_id='aruco_' + str(aruco), timeout=rospy.Duration(5))

    package_search_enabled = True
    shelf_search_enabled = True

    print 'fly to deadzone upper z'
    navigate_wait(z=DEADZONE_UPPER_Z-UPPER_Z, speed=0.3, frame_id='navigate_target')

    package_search_enabled = False

    print 'fly to deadzone lower z'
    navigate_wait(z=DEADZONE_LOWER_Z-DEADZONE_UPPER_Z, speed=0.3, frame_id='navigate_target')

    current_shelf += 1
    package_search_enabled = True

    print 'fly to lower z'
    navigate_wait(z=LOWER_Z-DEADZONE_LOWER_Z, speed=0.3, frame_id='navigate_target')

    shelf_search_enabled = False
    package_search_enabled = False


def fly_to_next():
    navigate_wait(y=BETWEEN_SHELVES, speed=0.3, frame_id='navigate_target')  # TODO: y?


# def scan2():
#     global current_line, current_shelf
#     current_line = 'A'
#     current_shelf = 0

#     print 'start scanning'
#     navigate_and_wait_aruco(_id=105, y=4, speed=0.3, frame_id='navigate_target')
#     navigate_wait(x=0, y=0, z=1, speed=0.3, frame_id='aruco_105')

#     print 'fly down'
#     navigate_wait(x=0, y=0, z=-0.45, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(5))

#     # TODO: enable QR package shelf 1
#     # TODO: enable QR shelf 1

#     print 'fly up'
#     navigate_wait(x=0, y=0, z=0.86, speed=0.3, frame_id='navigate_target')

#     # TODO: disable QR package shelf 1
#     # TODO: enable QR shelf 2

#     print 'fly up dead zone'
#     navigate_wait(z=0.5, speed=0.3, frame_id='navigate_target')

#     # TODO: enable QR package shelf 2

#     navigate_and_wait_aruco(_id=105, z=1.60, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(10))
#     navigate_wait(z=3.50, speed=0.3, frame_id='aruco_105', timeout=rospy.Duration(5))

#     # TODO: disable QR package shelf 2
#     # TODO: disable QR shelf 2

#     # ======================== NEXT SHELF ========================
#     navigate_wait(y=1.4, speed=0.3, frame_id='navigate_target')  # TODO: y?

#     # TODO: enable QR package shelf 3
#     # TODO: enable QR shelf 3

#     navigate_wait(x=0, y=0, z=-1.6, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(10))

#     # TODO: disable QR package shelf 3

#     print 'fly down dead zone'
#     navigate_wait(z=-0.5, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(10))

#     # TODO: disable QR shelf 3
#     # TODO: enable QR shelf 4
#     # TODO: enable QR package shelf 4

#     navigate_wait(z=-0.86, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(10))

#     # TODO: disable QR shelf 4
#     # TODO: disable QR package shelf 4


def fly_to_shelf():
    print 'fly to shelf'
    navigate_wait(x=4, y=0, z=0, speed=0.5, frame_id='navigate_target', timeout=rospy.Duration(8))
    print 'adjust yaw'
    navigate(x=0, y=0, z=0, yaw=math.pi, speed=0.5, frame_id='navigate_target')
    rospy.sleep(5)


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
    wait_aruco(142)
    navigate_wait(x=0, y=0, z=0.6, speed=0.3, frame_id='aruco_142')
    land() # TODO: check landing params


def scan_line_a():
    global current_line
    current_line = 'A'

    print 'start scanning'
    navigate_and_wait_aruco(_id=105, y=4, speed=0.3, frame_id='navigate_target')
    navigate_wait(x=0, y=0, z=1, speed=0.3, frame_id='aruco_105')

    print 'fly down'
    navigate_wait(x=0, y=0, z=-0.45, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(5))

    scan_up(current_z=0.55, aruco=105)
    fly_to_next()
    scan_down()
    fly_to_next()
    scan_up()
    fly_to_next()
    scan_down(aruco=106)


def scan_line_b():
    global current_line
    current_line = 'B'

    print 'start scanning'
    navigate_and_wait_aruco(_id=108, y=-6, speed=0.3, frame_id='navigate_target')
    navigate_wait(x=0, y=0, z=1, speed=0.3, frame_id='aruco_108')

    print 'fly down'
    navigate_wait(x=0, y=0, z=-0.45, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(5))

    scan_up(current_z=0.55, aruco=108)
    fly_to_next()
    scan_down()
    fly_to_next()
    scan_up()
    fly_to_next()
    scan_down(aruco=109)


def fly_through_shelf(aruco, count_right=0, up=0, z=2.7, dist_window=3):
    navigate_and_wait_aruco(_id=aruco, y=-count_right*BETWEEN_SHELVES, z=up, speed=-0.2, frame_id='navigate_target', timeout=rospy.Duration(15))
    navigate_wait(z=z, frame_id='aruco'+str(aruco), timeout=rospy.Duration(15))

    print 'fly through window'
    navigate_wait(x=dist_window, y=0, z=0, speed=0.8, frame_id='navigate_target')


def _mission():
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
    navigate_wait(x=0, y=0, z=-0.45, speed=0.3, frame_id='navigate_target', timeout=rospy.Duration(1))

    global current_line
    current_line = 'A'
    qr_sub = rospy.Subscriber('qr_reader/qr', DetectedQr, qr_cb, queue_size=1)

    scan()

    qr_sub.unregister()


def mission():
    print 'takeoff'
    print navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True, timeout=rospy.Duration(5))

    print 'fly over the flag'
    print navigate_wait(y=-1, speed=0.5, frame_id='navigate_target', timeout=rospy.Duration(2))

    fly_to_shelf()

    print 'start scanning qr'
    qr_sub = rospy.Subscriber('qr_reader/qr', DetectedQr, qr_cb, queue_size=1)

    scan_line_a()

    fly_through_shelf(aruco=107)  # TODO: count_right

    scan_line_b()

    print 'stop scanning qr'
    qr_sub.unregister()

    fly_through_shelf(aruco=110)  # TODO: count_right

    print 'fly right to landing'
    count_to_landing = 1
    # TODO: dist right
    navigate_and_wait_aruco(_id=111, y=-count_to_landing*BETWEEN_SHELVES, speed=-0.3, frame_id='navigate_target', timeout=rospy.Duration(10))

    print 'fly to landing'
    # TODO: dist forward
    navigate_and_wait_aruco(_id=142, x=4, speed=-0.5, frame_id='navigate_target', timeout=rospy.Duration(10))
    navigate_wait(x=0, y=0, z=0.5, speed=0.2, frame_id='aruco_142', tolerance=0.12)
    land()


packages = read_csv(5)
print 'got packages', packages

# ensure get_telemetry works
print get_telemetry()

# print 'takeoff'
# print navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True, timeout=rospy.Duration(5))
# pick_payload()

# print 'take off'
# takeoff()
# mission()
# print 'tkof'
# print navigate(z=1.5, frame_id='body', auto_arm=True)
# wait_aruco(0)
# navigate(x=0, y=0, z=1, speed=0.3, frame_id='aruco_0')

# print 'finish'

#fly_to_shelf()
