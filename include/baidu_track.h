#ifndef __BAIDU_TRACK_H__
#define __BAIDU_TRACK_H__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <json/json.h>
#include "body_analysis.h"
#include "xiaoqiang_track_point.h"

namespace XiaoqiangTrack
{

class BaiduTrack
{

  public:
    BaiduTrack();
    ~BaiduTrack(void);
    static std::vector<float> get_rect(std::vector<Point> points)
    {
        float left = -1;
        float top = -1;
        float right = -1;
        float bottom = -1;
        for (int i = 0; i < points.size(); i++)
        {
            Point point = points[i];
            if (left < 0 || point.x < left)
                left = point.x;
            if (top < 0 || point.y < top)
                top = point.y;
            if (right < 0 || point.x > right)
                right = point.x;
            if (bottom < 0 || point.y > bottom)
                bottom = point.y;
        }

        return std::vector<float>{left, top, right - left, bottom - top};
    };

    static float abs(float num)
    {
        if (num < 0)
            return -num;
        return num;
    };

    static bool isNear(Point point1, Point point2)
    {
        if (abs(point1.x - 320) + abs(point1.y - 240) >= abs(point2.x - 320) + abs(point2.y - 240))
            return false;
        return true;
    };

    std::vector<int32_t> getBodyRect(sensor_msgs::Image frame);

  private:
    std::string app_id;
    std::string api_key;
    std::string secret_key;
    // aip::Bodyanalysis* client;
};

} // namespace XiaoqiangTrack

#endif
