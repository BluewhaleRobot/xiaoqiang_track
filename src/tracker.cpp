#include "tracker.h"

namespace XiaoqiangTrack
{
Tracker::Tracker(std::string tracker_type)
{
    init_flag = false;
    this->tracker_type = tracker_type;
    tracking_status1 = false;
    tracking_status2 = false;
}

void Tracker::initTracker(cv::Mat &frame, cv::Rect2d &bbox)
{
    // #if (CV_MINOR_VERSION < 3)
    //     {
    //         tracker = cv::Tracker::create(tracker_type);
    //     }
    // #else
    //     {
    //         if (tracker_type == "BOOSTING")
    //             tracker = cv::TrackerBoosting::create();
    //         if (tracker_type == "MIL")
    //             tracker = cv::TrackerMIL::create();
    //         if (tracker_type == "KCF")
    //         {
    //             cv::TrackerKCF::Params param;
    //             param.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
    //             param.desc_npca = 0;
    //             param.compress_feature = true;
    //             param.compressed_size = 2;
    //             cv::Ptr<cv::TrackerKCF> mtracker = cv::TrackerKCF::create(param);
    //             // mtracker->setFeatureExtractor(sobelExtractor);
    //             tracker = mtracker;
    //         }
    //         if (tracker_type == "TLD")
    //             tracker = cv::TrackerTLD::create();
    //         if (tracker_type == "MEDIANFLOW")
    //             tracker = cv::TrackerMedianFlow::create();
    //         if (tracker_type == "GOTURN")
    //             tracker = cv::TrackerGOTURN::create();
    //         if (tracker_type == "MOSSE")
    //             tracker = cv::TrackerMOSSE::create();
    //     }
    // #endif
    ROS_INFO_STREAM("initTracker OK1");
    ROS_INFO_STREAM(bbox);
    rect1 = bbox;
    rect2 = bbox;
    if (!tracking_status1)
    {
        tracker1 = cv::TrackerMedianFlow::create();
        tracking_status1 = tracker1->init(frame, rect1);
    }

    ROS_INFO_STREAM("initTracker OK2");

    if (!tracking_status2)
    {
        tracker2 = cv::TrackerMOSSE::create();
        tracking_status2 = tracker2->init(frame, rect2);
    }

    ROS_INFO_STREAM("initTracker OK3");

    if (tracking_status1 && !tracking_status2)
    {
        bbox = rect1;
    }
    else if (!tracking_status1 && tracking_status2)
    {
        bbox = rect2;
    }
    else if (tracking_status1 && tracking_status2)
    {
        // bbox.x = (rect1.x + rect2.x) / 2;
        // bbox.y = (rect1.y + rect2.y) / 2;
        // bbox.width = (rect1.width + rect2.width) / 2;
        // bbox.height = (rect1.height + rect2.height) / 2;
        bbox = rect2;
    }

    ROS_INFO_STREAM("initTracker OK4");

    cv::rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1);
    init_flag = true;
}

int Tracker::updateFrame(cv::Mat &frame, cv::Rect2d &bbox)
{
    if (!init_flag)
        return 0;
    double timer = (double)cv::getTickCount();

    // Update the tracking result

    tracking_status1 = tracker1->update(frame, rect1);
    tracking_status2 = tracker2->update(frame, rect2);

    // 错误结果，判定为无效
    if (rect1.width > 300 || rect1.height > 300)
    {
        tracking_status1 = false;
    }
    // 错误结果，判定为无效
    if (rect2.width > 300 || rect2.height > 300)
    {
        tracking_status2 = false;
    }

    if (tracking_status1 && !tracking_status2)
    {
        bbox = rect1;
    }
    else if (!tracking_status1 && tracking_status2)
    {
        bbox = rect2;
    }
    else if (tracking_status1 && tracking_status2)
    {
        // bbox.x = (rect1.x + rect2.x) / 2;
        // bbox.y = (rect1.y + rect2.y) / 2;
        // bbox.width = (rect1.width + rect2.width) / 2;
        // bbox.height = (rect1.height + rect2.height) / 2;
        bbox = rect2;
    }

    // Calculate Frames per second (FPS)
    float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

    if (tracking_status2 || tracking_status1)
    {
        // Tracking success : Draw the tracked object
        rectangle(frame, rect1, cv::Scalar(255, 0, 0), 2, 1);
        rectangle(frame, rect2, cv::Scalar(0, 0, 255), 2, 1);
    }
    else
    {
        // Tracking failure detected.
        putText(frame, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
    }

    // Display tracker type on frame
    putText(frame, tracker_type + " Tracker", cv::Point(100, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);

    // Display FPS on frame
    putText(frame, "FPS : " + SSTR(int(fps)), cv::Point(100, 50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
    // ROS_INFO_STREAM("KCF: " << tracking_status1);
    // ROS_INFO_STREAM("MOOSE: " << tracking_status2);
    // 全部追踪中，状态0, 一个不追踪，状态1，全不追踪状态2
    if (tracking_status1 && tracking_status2)
        return 2;
    else if (!tracking_status1 && !tracking_status2)
        return 0;
    else
        return 1;
}

void Tracker::reset(cv::Mat frame, cv::Rect2d &bbox, bool reset_all)
{
    init_flag = false;
    double dis1 = cv::norm(cv::Point(rect1.x + rect1.width / 2, rect1.y + rect1.height / 2)
         - cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2));
    double dis2 = cv::norm(cv::Point(rect2.x + rect2.width / 2, rect2.y + rect2.height / 2)
         - cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2));
    if (!tracking_status1 || reset_all || dis1 > 80)
    {
        tracking_status1 = false;
        tracker1->clear();
    }

    if (!tracking_status2 || reset_all || dis2 > 80)
    {
        tracking_status2 = false;
        tracker2->clear();
    }

    initTracker(frame, bbox);
}

void Tracker::stop()
{
    init_flag = false;
}

} // namespace XiaoqiangTrack
