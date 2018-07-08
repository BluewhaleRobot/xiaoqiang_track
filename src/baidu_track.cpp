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

#include "baidu_track.h"

namespace XiaoqiangTrack
{

BaiduTrack::BaiduTrack(ros::NodeHandle nh) : PoseTracker(nh)
{
    ros::param::param<std::string>("~app_id", app_id, "");
    ros::param::param<std::string>("~api_key", api_key, "");
    ros::param::param<std::string>("~secret_key", secret_key, "");
    client = new aip::Bodyanalysis(app_id, api_key, secret_key);
}

BaiduTrack::~BaiduTrack(void)
{
    free(client);
}

cv::Rect2d BaiduTrack::getBodyRect(sensor_msgs::Image frame)
{
    cv::Mat cv_image = cv_bridge::toCvCopy(frame, "bgr8")->image;
    std::vector<uchar> buf;
    if (!cv::imencode(".jpg", cv_image, buf))
    {
        ROS_ERROR_STREAM("Encode image error");
    }
    std::string image_data = std::string(buf.begin(), buf.end());
    Json::Value body_info = client->body_analysis(image_data, aip::null);

    int person_num = body_info["person_num"].asInt();
    if (person_num > 0)
    {
        cv::Rect2d center_person;
        Json::Value persons = body_info["person_info"];
        std::vector<cv::Rect2d> bodys;
        for (int i = 0; i < persons.size(); i++)
        {
            Json::Value person_info = persons[i];
            cv::Point right_hip(
                person_info["body_parts"]["right_hip"]["x"].asFloat(),
                person_info["body_parts"]["right_hip"]["y"].asFloat());
            cv::Point left_hip(
                person_info["body_parts"]["left_hip"]["x"].asFloat(),
                person_info["body_parts"]["left_hip"]["y"].asFloat());
            cv::Point nose(
                person_info["body_parts"]["nose"]["x"].asFloat(),
                person_info["body_parts"]["nose"]["y"].asFloat());
            std::vector<cv::Point> rect_points{right_hip, left_hip, nose};
            cv::Rect2d person_location = getRect(rect_points);
            if (person_location.height < 50)
            {
                // 最小宽度50
                person_location.height = 50;
            }

            if (person_location.width > 200)
            {
                // 正常不应该这么宽，识别错误
                continue;
            }
            bodys.push_back(person_location);
        }
        center_person = selectTarget(bodys);
        return center_person;
    }
    return cv::Rect2d(0, 0, 0, 0);
}

} // namespace XiaoqiangTrack