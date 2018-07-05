#include "body_pose_track.h"

namespace XiaoqiangTrack
{
BodyTrack::BodyTrack(ros::NodeHandle nh)
{
    ROS_INFO_STREAM("Waitting for get_body_pose service");
    client = nh.serviceClient<body_pose::BodyPose>("get_body_pose");
    client.waitForExistence();
    ROS_INFO_STREAM("Waitting for get_body_pose service succeed");
}

BodyTrack::~BodyTrack()
{
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
        ROS_INFO_STREAM("Found no body in image");
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
        if (person_location.height == 0)
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