/**
 * Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * @author baidu aip
 */

#ifndef __AIP_BODYANALYSIS_H__
#define __AIP_BODYANALYSIS_H__

#include "base/base.h"

namespace aip {

    class Bodyanalysis: public AipBase
    {
    public:

    
        std::string _body_analysis =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/body_analysis";
        
        std::string _body_attr =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/body_attr";
        
        std::string _body_num =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/body_num";
        

        Bodyanalysis(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        /**
         * body_analysis
         * 对于输入的一张图片（可正常解码，且长宽比适宜），检测图片中的所有人体，**输出每个人体的14个关键点**，包含四肢、脖颈、鼻子等部位，**以及人体的坐标信息和数量**。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value body_analysis(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_body_analysis, null, data, null);

            return result;
        }
        
        /**
         * body_attr
         * 对于输入的一张图片（可正常解码，且长宽比适宜），输出图片中的所有人体的静态属性，包含**性别、年龄阶段、衣着（含类别/颜色/纹理）、是否戴帽子、是否戴眼镜、是否撑伞、是否使用手机、身体朝向**。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * type gender,<br>age,<br>lower_wear,<br>upper_wear,<br>headwear,<br>glasses,<br>upper_color,<br>lower_color,<br>cellphone,<br>upper_wear_fg,<br>upper_wear_texture,<br>lower_wear_texture,<br>orientation,<br>umbrella | 1）可选值说明：<br>gender-性别，age-年龄阶段，lower_wear-下身服饰，upper_wear-上身服饰，headwear-是否戴帽子，glasses-是否戴眼镜，upper_color-上身服饰颜色，lower_color-下身服饰颜色，cellphone-是否使用手机，upper_wear_fg-上身服饰细分类，upper_wear_texture-上身服饰纹理，lower_wear_texture-下身服饰纹理，orientation-身体朝向，umbrella-是否撑伞；<br>2）type 参数值可以是可选值的组合，用逗号分隔；**如果无此参数默认输出全部14个属性**
         */
        Json::Value body_attr(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_body_attr, null, data, null);

            return result;
        }
        
        /**
         * body_num
         * 对于输入的一张图片（可正常解码，且长宽比适宜），**识别和统计图像中的人体个数**，以俯拍角度为主要识别视角，**支持特定框选区域的人数统计，并可以输出渲染图片（人体头顶热力图）**。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * area 特定框选区域坐标，逗号分隔，如‘x1,y1,x2,y2,x3,y3...xn,yn'，默认尾点和首点相连做闭合，**此参数为空或无此参数默认识别整个图片的人数**
         * show 是否输出渲染的图片，默认不返回，**选true时返回渲染后的图片(base64)**，其它无效值或为空则默认false
         */
        Json::Value body_num(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_body_num, null, data, null);

            return result;
        }
        

    };
}
#endif