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

SPEED_PUB = None

def update_target(target):
    if SPEED_PUB == None:
        return
    speed = Twist()
    speed.linear.x = 0.0
    if target.x > 320 + 60:
        speed.angular.z = -0.4
    elif target.x < 320 - 60 and target.x > 10:
        speed.angular.z = 0.4
    elif target.x < 10:
        speed.angular.z = 0.0
        speed.linear.x = 0.0
    else:
        speed.angular.z = 0.0
        speed.linear.x = 0.4
    SPEED_PUB.publish(speed)
        
if __name__ == "__main__":
    rospy.init_node("xiaoqiang_tracking_controller", anonymous=True)
    SPEED_PUB = rospy.Publisher("~cmd_vel", Twist)
    rospy.Subscriber("~target", TrackTarget, update_target)
    rospy.spin()