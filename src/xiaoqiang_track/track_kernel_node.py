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

import json
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from xiaoqiang_track.srv import TrackKernel

from engines.baidu_track import BaiduTrack
from tracker import Tracker

FRAME = None
BODY_TRACKER = None
IMAGE_PUB = None
BRIDGE = CvBridge()


def get_one_frame():
    global FRAME
    FRAME = None

    def set_frame(new_frame):
        global FRAME
        if FRAME is not None:
            return
        FRAME = new_frame

    sub = rospy.Subscriber("~image", Image, set_frame)
    time_count = 0
    timeout = rospy.get_param("timeout", 5000)
    while FRAME is None and time_count < timeout:
        time.sleep(0.01)
        time_count += 10
    # unregister to improve performance
    sub.unregister()
    if FRAME is None:
        rospy.logerr("Cannot get image from image topic")
    return FRAME


def update_frame(new_frame):
    try:
        cv_image = BRIDGE.imgmsg_to_cv2(new_frame, "bgr8")
    except CvBridgeError, e:
        rospy.logerr(e)
    if BODY_TRACKER is None:
        return
    BODY_TRACKER.update_frame(cv_image)
    IMAGE_PUB.publish(BRIDGE.cv2_to_imgmsg(cv_image, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("track_kernel_node", anonymous=True)
    talk_pub = rospy.Publisher("~text", String, queue_size=10)
    IMAGE_PUB = rospy.Publisher("~processed_image", Image, queue_size=10)
    client = BaiduTrack()
    BODY_TRACKER = Tracker(rospy.get_param("~tracker", "KCF"))
    # 告诉用户站在前面
    words = String()
    words.data = "请站在我前面"
    talk_pub.publish(words)
    # 提醒用户调整好距离
    FRAME = get_one_frame()
    body_rect = None
    while True:
        if FRAME is not None:
            body_rect = client.get_body_rect(FRAME)
            if body_rect is None:
                words = String()
                words.data = "我没有看到人,请站到我前面"
                talk_pub.publish(words)
                time.sleep(4)
            elif body_rect[0] + body_rect[2] / 2 > 370 or body_rect[0] - body_rect[2] / 2 < 270:
                words = String()
                words.data = "请站到镜头中间来"
                talk_pub.publish(words)
                time.sleep(4)
            else:
                words = String()
                words.data = "我看到人了,开始追踪"
                talk_pub.publish(words)
                rospy.loginfo(json.dumps(body_rect, indent=4))
                time.sleep(4)
                break
        time.sleep(4)
        FRAME = get_one_frame()

    # 告诉用户可以开始走了
    try:
        cv_image = BRIDGE.imgmsg_to_cv2(FRAME, "bgr8")
    except CvBridgeError, e:
        rospy.logerr(e)
    BODY_TRACKER.init_track(body_rect, cv_image)
    rospy.Subscriber("~image", Image, update_frame)
    while not rospy.is_shutdown():
        time.sleep(10)
        FRAME = get_one_frame()
        body_rect = client.get_body_rect(FRAME)
        if body_rect is None:
            words = String()
            words.data = "我没有看到人,请站到我前面"
            talk_pub.publish(words)
            time.sleep(4)
            continue
        else:
            words = String()
            words.data = "更新追踪位置"
            talk_pub.publish(words)
            rospy.loginfo(json.dumps(body_rect, indent=4))
            time.sleep(4)
        try:
            cv_image = BRIDGE.imgmsg_to_cv2(FRAME, "bgr8")
        except CvBridgeError, e:
            rospy.logerr(e)
        BODY_TRACKER = Tracker(rospy.get_param("~tracker", "KCF"))
        BODY_TRACKER.init_track(body_rect, cv_image)
