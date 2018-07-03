#include "baidu_track.h"

namespace XiaoqiangTrack
{

BaiduTrack::BaiduTrack()
{
    ros::param::param<std::string>("~app_id", app_id, "");
    ros::param::param<std::string>("~api_key", api_key, "");
    ros::param::param<std::string>("~secret_key", secret_key, "");
    // client = new aip::Bodyanalysis(app_id, api_key, secret_key);
}

BaiduTrack::~BaiduTrack(void)
{
    // free(client);
}

std::vector<int32_t> BaiduTrack::getBodyRect(sensor_msgs::Image frame)
{
    cv::Mat cv_image = cv_bridge::toCvCopy(frame, "bgr8")->image;
    std::vector<uchar> buf;
    if (!cv::imencode(".jpg", cv_image, buf))
    {
        ROS_ERROR_STREAM("Encode image error");
    }
    std::string image_data = std::string(buf.begin(), buf.end());
    Json::Value body_info("");
    // Json::Value body_info = client->body_analysis(image_data, aip::null);
    ROS_INFO_STREAM(body_info.toStyledString());
    int person_num = body_info["person_num"].asInt();
    if (person_num > 0)
    {
        std::vector<float> center_person;
        Point center_point(-1, -1);
        Json::Value persons = body_info["persion_info"];
        for (int i = 0; i < persons.size(); i++)
        {
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
            std::vector<float> persion_location = get_rect(rect_points);
            if (persion_location[3] < 50)
            {
                // 最小宽度50
                persion_location[3] = 50;
            }
            Point current_center(
                persion_location[0] + persion_location[2] / 2,
                persion_location[1] + persion_location[3] / 2);
            if (center_point.x < 0)
            {
                center_point = current_center;
                center_person = persion_location;
            }

            if (isNear(current_center, center_point))
            {
                center_point = current_center;
                center_person = persion_location;
            }
        }
        cv::rectangle(
            cv_image,
            cvPoint(int(center_person[0]), int(center_person[1])),
            cvPoint(int(center_person[0] + center_person[2]), int(center_person[1] + center_person[3])),
            cvScalar(255, 0, 0));
        return std::vector<int32_t>{
            int(center_person[0]), int(center_person[1]),
            int(center_person[2]), int(center_person[3])};
    }
    return std::vector<int32_t>();
}

} // namespace XiaoqiangTrack