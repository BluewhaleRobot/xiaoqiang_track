#include "tracker.h"

namespace XiaoqiangTrack
{
Tracker::Tracker(std::string tracker_type)
{
    init_flag = false;
    this->tracker_type = tracker_type;
}

void Tracker::initTracker(cv::Mat &frame, cv::Rect2d &bbox)
{
#if (CV_MINOR_VERSION < 3)
    {
        tracker = cv::Tracker::create(tracker_type);
    }
#else
    {
        if (tracker_type == "BOOSTING")
            tracker = cv::TrackerBoosting::create();
        if (tracker_type == "MIL")
            tracker = cv::TrackerMIL::create();
        if (tracker_type == "KCF")
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
        if (tracker_type == "TLD")
            tracker = cv::TrackerTLD::create();
        if (tracker_type == "MEDIANFLOW")
            tracker = cv::TrackerMedianFlow::create();
        if (tracker_type == "GOTURN")
            tracker = cv::TrackerGOTURN::create();
        if (tracker_type == "MOSSE")
            tracker = cv::TrackerMOSSE::create();
    }
#endif
    cv::rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1);
    tracker->init(frame, bbox);
    init_flag = true;
}

bool Tracker::updateFrame(cv::Mat &frame, cv::Rect2d &bbox)
{
    if (!init_flag)
        return false;
    double timer = (double)cv::getTickCount();

    // Update the tracking result
    bool ok = tracker->update(frame, bbox);

    // Calculate Frames per second (FPS)
    float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

    if (ok)
    {
        // Tracking success : Draw the tracked object
        rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1);
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
    return ok;
}

void Tracker::reset(cv::Mat frame, cv::Rect2d &bbox)
{
    init_flag = false;
    tracker->clear();
    initTracker(frame, bbox);
}

} // namespace XiaoqiangTrack
