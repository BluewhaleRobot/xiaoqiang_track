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

#include "tracker.h"

namespace XiaoqiangTrack
{
Tracker::Tracker(std::string tracker_main_type, std::string tracker_aided_type)
    : tracker_main_type(tracker_main_type), tracker_aided_type(tracker_aided_type)
{
    inited_flag = false;
    tracker_main_status = false;
    tracker_aided_status = false;
    if (!getTrackType(tracker_main_type))
    {
        ROS_FATAL_STREAM("unknow tracker type: " << tracker_main_type);
        exit(1);
    }

    if (!getTrackType(tracker_aided_type))
    {
        ROS_FATAL_STREAM("unknow tracker type: " << tracker_aided_type);
        exit(1);
    }
}

bool Tracker::getTrackType(std::string tracker_type)
{
    std::vector<std::string> tracker_types{
        "BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE"};
    for (int i = 0; i < tracker_types.size(); i++)
    {
        if (tracker_types[i] == tracker_type)
            return true;
    }
    return false;
}

cv::Ptr<cv::Tracker> Tracker::getTracker(std::string tracker_main_type)
{
    cv::Ptr<cv::Tracker> tracker;

#if (CV_MINOR_VERSION < 3)
    {
        tracker = cv::Tracker::create(tracker_main_type);
    }
#else
    {
        if (tracker_main_type == "BOOSTING")
            tracker = cv::TrackerBoosting::create();
        if (tracker_main_type == "MIL")
            tracker = cv::TrackerMIL::create();
        if (tracker_main_type == "KCF")
        {
            cv::TrackerKCF::Params param;
            param.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
            param.desc_npca = 0;
            param.compress_feature = true;
            param.compressed_size = 2;
            cv::Ptr<cv::TrackerKCF> mtracker = cv::TrackerKCF::create(param);
            // mtracker->setFeatureExtractor(sobelExtractor);
            tracker = mtracker;
        }
        if (tracker_main_type == "TLD")
            tracker = cv::TrackerTLD::create();
        if (tracker_main_type == "MEDIANFLOW")
            tracker = cv::TrackerMedianFlow::create();
        if (tracker_main_type == "GOTURN")
            tracker = cv::TrackerGOTURN::create();
        if (tracker_main_type == "MOSSE")
            tracker = cv::TrackerMOSSE::create();
    }
#endif
    return tracker;
}

void Tracker::initTracker(cv::Mat &frame, cv::Rect2d &bbox)
{
    rect_main = bbox;
    rect_aided = bbox;
    if (!tracker_main_status)
    {
        tracker_main = getTracker(tracker_main_type);
        tracker_main_status = tracker_main->init(frame, rect_main);
    }

    if (!tracker_aided_status)
    {
        tracker_aided = getTracker(tracker_aided_type);
        tracker_aided_status = tracker_aided->init(frame, rect_aided);
    }

    if (tracker_main_status)
    {
        bbox = rect_main;
    }
    else
    {
        bbox = rect_aided;
    }

    cv::rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1);
    inited_flag = true;
}

int Tracker::updateFrame(cv::Mat &frame, cv::Rect2d &bbox)
{
    if (!inited_flag)
        return 0;
    double timer = (double)cv::getTickCount();

    // Update the tracking result

    tracker_main_status = tracker_main->update(frame, rect_main);
    tracker_aided_status = tracker_aided->update(frame, rect_aided);

    // 错误结果，判定为无效
    if (rect_main.width > 300 || rect_main.height > 300)
    {
        tracker_main_status = false;
    }
    // 错误结果，判定为无效
    if (rect_aided.width > 300 || rect_aided.height > 300)
    {
        tracker_aided_status = false;
    }

    if (tracker_main_status)
    {
        bbox = rect_main;
    }
    else
    {
        bbox = rect_aided;
    }

    // Calculate Frames per second (FPS)
    float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

    if (tracker_aided_status || tracker_main_status)
    {
        // Tracking success : Draw the tracked object
        rectangle(frame, rect_main, cv::Scalar(255, 0, 0), 2, 1);
        rectangle(frame, rect_aided, cv::Scalar(0, 0, 255), 2, 1);
    }
    else
    {
        // Tracking failure detected.
        putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
    }

    // Display tracker type on frame
    putText(frame, tracker_main_type + " Tracker", cv::Point(100, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);

    // Display FPS on frame
    putText(frame, "FPS : " + SSTR(int(fps)), cv::Point(100, 50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
    // 全部追踪中，状态0, 一个不追踪，状态1，全不追踪状态2
    if (tracker_main_status && tracker_aided_status)
        return 2;
    else if (!tracker_main_status && !tracker_aided_status)
        return 0;
    else
        return 1;
}

void Tracker::reset(cv::Mat frame, cv::Rect2d &bbox, bool reset_all)
{
    inited_flag = false;
    double dis1 = cv::norm(cv::Point(rect_main.x + rect_main.width / 2, rect_main.y + rect_main.height / 2) - cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2));
    double dis2 = cv::norm(cv::Point(rect_aided.x + rect_aided.width / 2, rect_aided.y + rect_aided.height / 2) - cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2));
    if (!tracker_main_status || reset_all || dis1 > 80)
    {
        tracker_main_status = false;
        tracker_main->clear();
    }

    if (!tracker_aided_status || reset_all || dis2 > 80)
    {
        tracker_aided_status = false;
        tracker_aided->clear();
    }

    initTracker(frame, bbox);
}

void Tracker::stop()
{
    inited_flag = false;
}

} // namespace XiaoqiangTrack
