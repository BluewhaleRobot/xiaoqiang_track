
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

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include "baidu_track.h"

using namespace cv;
using namespace std;
using namespace XiaoqiangTrack;

sensor_msgs::Image last_frame;
Ptr<Tracker> tracker;
Rect2d body_rect;
ros::Publisher image_pub;
BaiduTrack client;

sensor_msgs::Image get_one_frame()
{
    return last_frame;
}

void update_frame(const sensor_msgs::ImageConstPtr &new_frame)
{
    last_frame = *new_frame;
    return;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(new_frame, "bgr8");
    cv::Mat cv_image = cv_ptr->image;
    if (tracker == NULL)
        return;
    tracker->update(cv_image, body_rect);
    image_pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xiaoqiang_track_node");
    ros::NodeHandle nh;
    ros::Publisher talk_pub = nh.advertise<std_msgs::String>("~text", 10);
    // image_pub = nh.advertise<sensor_msgs::Image>("~processed_image", 10);
    // ros::Subscriber image_sub = nh.subscribe<sensor_msgs::ImageConstPtr>("~image", 10, &update_frame);

    // // 告诉用户站在前面
    // std_msgs::String words;
    // words.data = "请站在我前面";
    // talk_pub.publish(words);
    // // 提醒用户调整好距离
    // sensor_msgs::Image frame = get_one_frame();
    // body_rect.x = -1;
    // body_rect.y = -1;
    // while (1)
    // {
    //     if (frame.data.size() != 0)
    //     {
    //         std::vector<int> rect = client.getBodyRect(frame);
    //         if (rect.size() == 0)
    //         {
    //             words.data = "我没有看到人,请站到我前面";
    //             talk_pub.publish(words);
    //             sleep(4);
    //         }
    //         else if (body_rect.x + body_rect.width / 2 > 370 || body_rect.x - body_rect.width / 2 < 270)
    //         {
    //             body_rect.x = rect[0];
    //             body_rect.y = rect[1];
    //             body_rect.width = rect[2];
    //             body_rect.height = rect[3];
    //             words.data = "请站到镜头中间来";
    //             talk_pub.publish(words);
    //             sleep(4);
    //         }
    //     }
    //     sleep(4);
    //     frame = get_one_frame();
    // }
}