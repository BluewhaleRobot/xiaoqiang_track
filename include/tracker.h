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

#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <ros/ros.h>

#define SSTR(x) static_cast<std::ostringstream &>(           \
                    (std::ostringstream() << std::dec << x)) \
                    .str()

namespace XiaoqiangTrack
{
class Tracker
{
  public:
    Tracker(std::string tracker_main_type, std::string tracker_aided_type);
    void initTracker(cv::Mat &frame, cv::Rect2d &bbox);
    int updateFrame(cv::Mat &frame, cv::Rect2d &bbox);
    void reset(cv::Mat frame, cv::Rect2d &bbox, bool reset_all = false);
    void stop();

  private:
    cv::Ptr<cv::Tracker> tracker_main;
    cv::Ptr<cv::Tracker> tracker_aided;
    bool tracker_main_status;
    bool tracker_aided_status;
    bool inited_flag;
    std::string tracker_main_type;
    std::string tracker_aided_type;
    cv::Rect2d rect_main;
    cv::Rect2d rect_aided;
    cv::Ptr<cv::Tracker> getTracker(std::string tracker_type);
    bool getTrackType(std::string tracker_type);
};
} // namespace XiaoqiangTrack

#endif