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
from xiaoqiang_track.srv import TrackKernel
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
from engines.baidu_track import BaiduTrack
import json

frame = None
def get_one_frame():
    def set_frame(new_frame):
        global frame
        if frame is not None:
            return
        frame = new_frame

    sub = rospy.Subscriber("~image", Image, set_frame)
    time_count = 0
    timeout = rospy.get_param("timeout", 5000)
    while frame is None and time_count < timeout:
        time.sleep(0.01)
        time_count += 10
    # unregister to improve performance
    sub.unregister()
    if frame is None:
        rospy.logerr("Cannot get image from image topic")
    return frame

if __name__ == "__main__":
    rospy.init_node("track_kernel_node", anonymous=True)
    talk_pub = rospy.Publisher("~text", String, queue_size=10)
    client = BaiduTrack()
    # 告诉用户站在前面
    words = String()
    words.data = "请站在我前面"
    talk_pub.publish(words)
    # 提醒用户调整好距离
    frame = get_one_frame()
    no_kernel_flag = True
    while no_kernel_flag:
        if frame is not None:
            rect = client.get_body_rect(frame)
            if rect is None:
                words = String()
                words.data = "我没有看到人,请站到我前面"
                talk_pub.publish(words)
                time.sleep(4)
            
            else:
                words = String()
                words.data = "我看到人了,请站到我前面"
                talk_pub.publish(words)
                print(json.dumps(rect, indent=4))
                time.sleep(4)
        frame = None
        frame = get_one_frame()
        

    # 告诉用户可以开始走了

    while not rospy.is_shutdown():
        time.sleep(0.1)

