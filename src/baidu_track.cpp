#include "baidu_track.h"

namespace XiaoqiangTrack
{

BaiduTrack::BaiduTrack()
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
    // client->access_token = "";
    Json::Value body_info = client->body_analysis(image_data, aip::null);
    ROS_INFO_STREAM(body_info.toStyledString());
    if(body_info.get("error_code", "").asString() == "null")
    {
        ROS_INFO_STREAM("Call API Failed");
        client->access_token = "";
    }
        
    int person_num = body_info["person_num"].asInt();
    if (person_num > 0)
    {
        ROS_INFO_STREAM("getBodyRect OK1");
        cv::Rect2d center_person;
        Json::Value persons = body_info["person_info"];

        ROS_INFO_STREAM("getBodyRect OK2");
        std::vector<cv::Rect2d> bodys;
        for (int i = 0; i < persons.size(); i++)
        {
            ROS_INFO_STREAM("getBodyRect OK3");
            Json::Value person_info = persons[i];
            Point right_hip(
                person_info["body_parts"]["right_hip"]["x"].asFloat(),
                person_info["body_parts"]["right_hip"]["y"].asFloat());
            Point left_hip(
                person_info["body_parts"]["left_hip"]["x"].asFloat(),
                person_info["body_parts"]["left_hip"]["y"].asFloat());
            Point nose(
                person_info["body_parts"]["nose"]["x"].asFloat(),
                person_info["body_parts"]["nose"]["y"].asFloat());
            std::vector<Point> rect_points{right_hip, left_hip, nose};
            std::vector<float> person_location = get_rect(rect_points);
            ROS_INFO_STREAM("getBodyRect OK4");
            if (person_location[2] < 50)
            {
                // 最小宽度50
                person_location[2] = 50;
            }

            if(person_location[2] > 200)
            {
                // 正常不应该这么宽，识别错误
                continue;
            }
            bodys.push_back(cv::Rect2d(person_location[0], person_location[1], person_location[2], person_location[3]));

            ROS_INFO_STREAM("getBodyRect OK5");
        }
        // center_person
        ROS_INFO_STREAM("bodys");
        for(int i=0;i<bodys.size(); i++)
        {
            ROS_INFO_STREAM("rect: " << bodys[i]);
        }
        center_person = selectTarget(bodys);
        ROS_INFO_STREAM("center persion: " << center_person);
        cv::rectangle(
            cv_image,
            cvPoint(int(center_person.x), int(center_person.y)),
            cvPoint(int(center_person.x + center_person.width), int(center_person.y + center_person.height)),
            cvScalar(255, 0, 0));
        return center_person;
    }
    return cv::Rect2d(0,0,0,0);
}

} // namespace XiaoqiangTrack