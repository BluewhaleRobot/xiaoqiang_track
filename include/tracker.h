#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#define SSTR(x) static_cast<std::ostringstream &>(           \
                    (std::ostringstream() << std::dec << x)) \
                    .str()

namespace XiaoqiangTrack
{
class Tracker
{
  public:
    Tracker(std::string tracker_type);
    void initTracker(cv::Mat &frame, cv::Rect2d &bbox);
    bool updateFrame(cv::Mat &frame, cv::Rect2d &bbox);
    void reset(cv::Mat frame, cv::Rect2d &bbox);

  private:
    cv::Ptr<cv::Tracker> tracker;
    bool init_flag;
    std::string tracker_type;
};

static void sobelExtractor(const cv::Mat img, const cv::Rect roi, cv::Mat &feat)
{
    cv::Mat sobel[2];
    cv::Mat patch;
    cv::Rect region = roi;

    // extract patch inside the image
    if (roi.x < 0)
    {
        region.x = 0;
        region.width += roi.x;
    }
    if (roi.y < 0)
    {
        region.y = 0;
        region.height += roi.y;
    }
    if (roi.x + roi.width > img.cols)
        region.width = img.cols - roi.x;
    if (roi.y + roi.height > img.rows)
        region.height = img.rows - roi.y;
    if (region.width > img.cols)
        region.width = img.cols;
    if (region.height > img.rows)
        region.height = img.rows;

    patch = img(region).clone();
    cvtColor(patch, patch, CV_BGR2GRAY);

    // add some padding to compensate when the patch is outside image border
    int addTop, addBottom, addLeft, addRight;
    addTop = region.y - roi.y;
    addBottom = (roi.height + roi.y > img.rows ? roi.height + roi.y - img.rows : 0);
    addLeft = region.x - roi.x;
    addRight = (roi.width + roi.x > img.cols ? roi.width + roi.x - img.cols : 0);

    copyMakeBorder(patch, patch, addTop, addBottom, addLeft, addRight, cv::BORDER_REPLICATE);

    Sobel(patch, sobel[0], CV_32F, 1, 0, 1);
    Sobel(patch, sobel[1], CV_32F, 0, 1, 1);

    merge(sobel, 2, feat);

    feat.convertTo(feat, CV_64F);
    feat = feat / 255.0 - 0.5; // normalize to range -0.5 .. 0.5
};
} // namespace XiaoqiangTrack

#endif