/******************************************************************************
*
* The MIT License (MIT)
*
* Copyright (c) 2018 Bluewhale Robot
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Randoms
*******************************************************************************/

#ifndef __POSE_TRACKER_H__
#define __POSE_TRACKER_H__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

namespace XiaoqiangTrack
{
class PoseTracker
{
  public:
    PoseTracker(ros::NodeHandle nh) : nh(nh){};
    ~PoseTracker(){};
    static cv::Rect2d getRect(std::vector<cv::Point> points)
    {
        float left = -1;
        float top = -1;
        float right = -1;
        float bottom = -1;
        for (int i = 0; i < points.size(); i++)
        {
            cv::Point point = points[i];
            if (left < 0 || point.x < left)
                left = point.x;
            if (top < 0 || point.y < top)
                top = point.y;
            if (right < 0 || point.x > right)
                right = point.x;
            if (bottom < 0 || point.y > bottom)
                bottom = point.y;
        }

        return cv::Rect2d(left, top, right - left, bottom - top);
    };

    static float abs(float num)
    {
        if (num < 0)
            return -num;
        return num;
    };

    static bool isNear(cv::Point point1, cv::Point point2)
    {
        if (abs(point1.x - 320) + abs(point1.y - 240) >= abs(point2.x - 320) + abs(point2.y - 240))
            return false;
        return true;
    };

    static cv::Rect2d selectTarget(std::vector<cv::Rect2d> targets)
    {
        // 首先选择较大的目标，然后选择靠近中心的目标
        // 面积和中心距离的加权
        float current_target_value = -100;
        cv::Rect target_rect;
        for (int i = 0; i < targets.size(); i++)
        {
            cv::Rect2d target = targets[i];
            float target_value = target.width * target.height * 0.01 - abs(target.x - 320);
            if (target_value > current_target_value)
            {
                current_target_value = target_value;
                target_rect = target;
            }
        }
        return target_rect;
    }

    virtual cv::Rect2d getBodyRect(sensor_msgs::Image frame){};

  private:
    ros::NodeHandle nh;
};
} // namespace XiaoqiangTrack

#endif