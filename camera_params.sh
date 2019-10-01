#!/bin/sh

# Manual exposure
v4l2-ctl -d /dev/video1 -c exposure_auto=1
# Exposure value
v4l2-ctl -d /dev/video1 -c exposure_absolute=50
# Powerline frequency (50 Hz)
v4l2-ctl -d /dev/video1 -c power_line_frequency=1

