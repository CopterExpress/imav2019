#!/bin/bash

. /opt/ros/melodic/setup.bash
. /home/sf/devel/imav_ws/devel/setup.bash

ROS_MASTER_URI=http://sfnano.local:11311 rqt 2>/dev/null &
#ROS_MASTER_URI=http://sfnano.local:11311 rviz
sshpass -p '123' ssh sf@sfnano.local

