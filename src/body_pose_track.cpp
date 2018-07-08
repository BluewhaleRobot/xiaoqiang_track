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

#include "body_pose_track.h"

namespace XiaoqiangTrack
{
BodyTrack::BodyTrack(ros::NodeHandle nh):PoseTracker(nh)
{
    
    ROS_INFO_STREAM("Waitting for get_body_pose service");
    client = nh.serviceClient<body_pose::BodyPose>("get_body_pose");
    client.waitForExistence();
    ROS_INFO_STREAM("Waitting for get_body_pose service succeed");
}

cv::Rect2d BodyTrack::getBodyRect(sensor_msgs::Image frame)
{
    body_pose::BodyPose req = body_pose::BodyPose();
    req.request.image = frame;
    if (!client.call(req))
    {
        ROS_ERROR_STREAM("call get_body_pose service failed");
        return cv::Rect2d(0, 0, 0, 0);
    }

    if (req.response.body_poses.size() == 0)
    {
        return cv::Rect2d(0, 0, 0, 0);
    }

    std::vector<cv::Rect2d> bodys;
    for (int i = 0; i < req.response.body_poses.size(); i++)
    {
        cv::Rect2d person_location = bodyInfo2Rect2d(req.response.body_poses[i]);
        if (person_location.width < 50)
            person_location.width = 50;
        if (person_location.width > 200)
            continue;
        if (person_location.height <= 1)
            continue;
        bodys.push_back(person_location);
    }
    // select target
    ROS_INFO_STREAM("Found " << bodys.size() << " people");
    cv::Rect2d center_person = selectTarget(bodys);
    return center_person;
}

cv::Rect2d BodyTrack::bodyInfo2Rect2d(body_pose::BodyInfo body_info)
{
    std::vector<cv::Point> points;
    if (body_info.nose[0] > 0)
        points.push_back(cv::Point(body_info.nose[0], body_info.nose[1]));
    if (body_info.right_eye[0] > 0)
        points.push_back(cv::Point(body_info.right_eye[0], body_info.right_eye[1]));
    if (body_info.left_eye[0] > 0)
        points.push_back(cv::Point(body_info.left_eye[0], body_info.left_eye[1]));
    if (body_info.right_ear[0] > 0)
        points.push_back(cv::Point(body_info.right_ear[0], body_info.right_ear[1]));
    if (body_info.left_ear[0] > 0)
        points.push_back(cv::Point(body_info.left_ear[0], body_info.left_ear[1]));
    if (body_info.right_arm_top[0] > 0)
        points.push_back(cv::Point(body_info.right_arm_top[0], body_info.right_arm_top[1]));
    if (body_info.left_arm_top[0] > 0)
        points.push_back(cv::Point(body_info.left_arm_top[0], body_info.left_arm_top[1]));
    if (body_info.right_leg_top[0] > 0)
        points.push_back(cv::Point(body_info.right_leg_top[0], body_info.right_leg_top[1]));
    if (body_info.left_leg_top[0] > 0)
        points.push_back(cv::Point(body_info.left_leg_top[0], body_info.left_leg_top[1]));
    return getRect(points);
}

} // namespace XiaoqiangTrack