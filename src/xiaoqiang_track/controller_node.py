#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Randoms
#

import rospy
from xiaoqiang_track.msg import TrackTarget
from geometry_msgs.msg import Twist
from pid import PID
import math

SPEED_PUB = None
pid_controller = None
max_linear_speed = 0
max_angle_speed = 0
lost_flag = False


def update_target(target):
    global lost_flag
    if SPEED_PUB == None:
        return
    if target.x < 1 or target.y < 1:
        lost_flag = True
        speed = Twist()
        SPEED_PUB.publish(speed)
        return
    speed = Twist()
    rospy.loginfo(target)
    pid_controller.update(target.x - 320)
    output = pid_controller.output
    rospy.loginfo("output: {output}".format(output=output))
    if output > max_angle_speed:
        output = max_angle_speed
    speed.angular.z = output
    speed.linear.x = max_linear_speed * math.exp(-(target.x - 320.0)*(target.x - 320.0)/ (2 * 40*40))
    
    SPEED_PUB.publish(speed)
        
if __name__ == "__main__":
    rospy.init_node("xiaoqiang_tracking_controller", anonymous=True)
    SPEED_PUB = rospy.Publisher("~cmd_vel", Twist)
    # set up pid controller
    p = float(rospy.get_param("~p", 0.2))
    i = float(rospy.get_param("~i", 0.0))
    d = float(rospy.get_param("~d", 0.0))
    sample_rate = rospy.get_param("~sample_rate", 30)
    max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.8))
    max_angle_speed = float(rospy.get_param("~max_angle_speed", 1))
    pid_controller = PID(p,i,d)
    pid_controller.setSampleTime(1 / sample_rate)

    
    rospy.Subscriber("~target", TrackTarget, update_target)
    rospy.spin()