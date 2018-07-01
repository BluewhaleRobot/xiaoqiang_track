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

import tempfile
import time

import rospy
from aip import AipBodyAnalysis
from cv_bridge import CvBridge, CvBridgeError

import cv2
import json

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
        img, buf = cv2.imencode(".jpg", cv_image)
        body_info = self.client.bodyAnalysis(buf)
        if body_info["person_num"] > 0:
            persion_location = body_info["person_info"][0]["location"]
            cv2.rectangle(
                cv_image, (int(persion_location["left"]), int(
                    persion_location["top"])),
                (int(persion_location["left"] + persion_location["width"]),
                 int(persion_location["top"] + persion_location["height"])),
                (255, 0, 0))
            cv2.imshow("image", cv_image)
            cv2.waitKey()
            return {
                "left": int(persion_location["left"]),
                "top": int(persion_location["top"]),
                "right": int(persion_location["left"] + persion_location["width"]),
                "bottom": int(persion_location["top"] + persion_location["height"])
            }
        cv2.imshow("image", cv_image)
        cv2.waitKey()
        return None
