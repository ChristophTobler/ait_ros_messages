#!/usr/bin/python

#   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name AIT nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# File: omni_vio_sensor_republisher.py
# Created on: 28.04.16
# Author: Christoph Tobler

from __future__ import print_function, division

__author__ = 'christoph'

import sys
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu
from ait_ros_messages.msg import VioSensorMsg
import time
import numpy as np
import genpy


def vio_sensor_cb1(data):
    global left1_pub, right1_pub

    left1_pub.publish(data.left_image)
    right1_pub.publish(data.right_image)

def vio_sensor_cb2(data):
    global left2_pub, right2_pub

    left2_pub.publish(data.left_image)
    right2_pub.publish(data.right_image)

def vio_sensor_cb3(data):
    global left3_pub, right3_pub

    left3_pub.publish(data.left_image)
    right3_pub.publish(data.right_image)

def vio_sensor_cb4(data):
    global left4_pub, right4_pub

    left4_pub.publish(data.left_image)
    right4_pub.publish(data.right_image)


if __name__ == "__main__":

    rospy.init_node('image_publisher')

    left1_pub = rospy.Publisher("left1_image", Image, queue_size=1)
    right1_pub = rospy.Publisher("right1_image", Image, queue_size=1)
    rospy.Subscriber("/vio_sensor_1", VioSensorMsg, vio_sensor_cb1, queue_size=1)

    left2_pub = rospy.Publisher("left2_image", Image, queue_size=1)
    right2_pub = rospy.Publisher("right2_image", Image, queue_size=1)
    rospy.Subscriber("/vio_sensor_2", VioSensorMsg, vio_sensor_cb2, queue_size=1)

    left3_pub = rospy.Publisher("left3_image", Image, queue_size=1)
    right3_pub = rospy.Publisher("right3_image", Image, queue_size=1)
    rospy.Subscriber("/vio_sensor_3", VioSensorMsg, vio_sensor_cb3, queue_size=1)

    left4_pub = rospy.Publisher("left4_image", Image, queue_size=1)
    right4_pub = rospy.Publisher("right4_image", Image, queue_size=1)
    rospy.Subscriber("/vio_sensor_4", VioSensorMsg, vio_sensor_cb4, queue_size=1)

    rospy.spin()
