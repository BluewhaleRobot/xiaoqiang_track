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
import math
import time

import cv2
import rospy
from aip import AipBodyAnalysis
from cv_bridge import CvBridge, CvBridgeError


def get_rect(points):
    left = -1
    top = -1
    right = -1
    bottom = -1
    for point in points:
        if left == -1 or point["x"] < left:
            left = point["x"]
        if top == -1 or point["y"] < top:
            top = point["y"]
        if right == -1 or point["x"] > right:
            right = point["x"]
        if bottom == -1 or point["y"] > bottom:
            bottom = point["y"]

    return [left, top, right - left, bottom - top]


def is_near(point1, point2):
    if abs(point1[0] - 320) + abs(point1[1] - 240) >= abs(point2[0] - 320) + abs(point2[1] - 240):
        return False
    else:
        return True


class BaiduTrack:

    def __init__(self):
        self.app_id = rospy.get_param("~app_id")
        self.api_key = rospy.get_param("~api_key")
        self.secret_key = rospy.get_param("~secret_key")
        self.client = AipBodyAnalysis(
            self.app_id, self.api_key, self.secret_key)
        self.bridge = CvBridge()

    def get_body_rect(self, frame):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError, e:
            print e
        ok, buf = cv2.imencode(".jpg", cv_image)
        if not ok:
            rospy.logerr("image encode error")
        body_info = self.client.bodyAnalysis(buf)
        rospy.loginfo(body_info)
        if body_info["person_num"] > 0:
            center_person = None
            center_point = None
            # 找最靠近图像中间的人
            for person_info in body_info["person_info"]:
                right_hip = person_info["body_parts"]["right_hip"]
                left_hip = person_info["body_parts"]["left_hip"]
                nose = person_info["body_parts"]["nose"]

                persion_location = get_rect([right_hip, left_hip, nose])

                if persion_location[3] < 50:
                    persion_location[3] = 50
                current_center = [persion_location[0] + persion_location[2] /
                                  2, persion_location[1] + persion_location[3] / 2]
                if center_point == None:
                    center_point = current_center
                    center_person = persion_location
                if is_near(current_center, center_point):
                    center_point = current_center
                    center_person = persion_location

                # 同时考虑大小，取较大的

            cv2.rectangle(
                cv_image, (int(center_person[0]), int(
                    center_person[1])),
                (int(center_person[0] + center_person[2]),
                 int(center_person[1] + center_person[3])),
                (255, 0, 0))
            return (int(center_person[0]), int(center_person[1]),
                    int(center_person[2]), int(center_person[3]))
        return None
