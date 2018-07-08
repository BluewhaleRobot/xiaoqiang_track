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

import sys

import cv2
import rospy

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')


class Tracker:

    def __init__(self, tracker_type="KCF"):
        tracker_types = ['BOOSTING', 'MIL',
                         'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        if tracker_type not in tracker_types:
            rospy.logerr("Unkown tracker type {tracker}".format(
                tracker=tracker_type))
            return
        self.tracker_type = tracker_type

        if int(minor_ver) < 3:
            self.tracker = cv2.Tracker_create(self.tracker_type)
        else:
            if self.tracker_type == 'BOOSTING':  # 会漂，追踪还行
                self.tracker = cv2.TrackerBoosting_create()
            if self.tracker_type == 'MIL':  # 会不停的漂，追踪还行
                self.tracker = cv2.TrackerMIL_create()
            if self.tracker_type == 'KCF':  # 追踪效果很差
                self.tracker = cv2.TrackerKCF_create()
            if self.tracker_type == 'TLD':  # 效果很不好
                self.tracker = cv2.TrackerTLD_create()
            if self.tracker_type == 'MEDIANFLOW':  # 有尺度会丢失，追踪效果不好
                self.tracker = cv2.TrackerMedianFlow_create()
            if self.tracker_type == 'GOTURN':  # 无法运行
                self.tracker = cv2.TrackerGOTURN_create()
        self.init_flag = False

    def init_track(self, bbox, frame):
        self.tracker.init(frame, bbox)
        self.init_flag = True

    def update_frame(self, frame):
        if not self.init_flag:
            return None
        ok, bbox = self.tracker.update(frame)
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        else:
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        cv2.putText(frame, self.tracker_type + " Tracker", (100, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        return (int(bbox[0]), int(bbox[1]), int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
