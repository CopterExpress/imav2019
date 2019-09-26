#!/usr/bin/env python

import rospy
from clever import srv
from std_srvs.srv import Trigger
from zbar_ros_redux.msg import DetectedQr

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def qr_cb(msg):
    print 'qr', msg


qr_sub = rospy.Subscriber('qr_reader/qr', DetectedQr, qr_cb, queue_size=1,)


rospy.spin()
