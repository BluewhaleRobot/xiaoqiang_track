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

#ifndef __AIP_IMAGESEARCH_H__
#define __AIP_IMAGESEARCH_H__

#include "base/base.h"

namespace aip {

    class Imagesearch: public AipBase
    {
    public:

    
        std::string _same_hq_add =
            "https://aip.baidubce.com/rest/2.0/realtime_search/same_hq/add";
        
        std::string _same_hq_search =
            "https://aip.baidubce.com/rest/2.0/realtime_search/same_hq/search";
        
        std::string _same_hq_delete =
            "https://aip.baidubce.com/rest/2.0/realtime_search/same_hq/delete";
        
        std::string _similar_add =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/similar/add";
        
        std::string _similar_search =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/similar/search";
        
        std::string _similar_delete =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/similar/delete";
        
        std::string _product_add =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/product/add";
        
        std::string _product_search =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/product/search";
        
        std::string _product_delete =
            "https://aip.baidubce.com/rest/2.0/image-classify/v1/realtime_search/product/delete";
        

        Imagesearch(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        /**
         * same_hq_add
         * 相同图检索包含入库、检索、删除三个子接口；**在正式使用之前请在[控制台](https://console.bce.baidu.com/ai/#/ai/imagesearch/overview/index)创建应用后，在应用详情页申请建库，建库成功后方可正常使用入库、检索、删除三个接口**。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 检索时原样带回,最长256B。
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value same_hq_add(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_add, null, data, null);

            return result;
        }
        
        /**
         * same_hq_search
         * 相同图检索包含入库、检索、删除三个子接口；**在正式使用之前请在[控制台](https://console.bce.baidu.com/ai/#/ai/imagesearch/overview/index)创建应用后，在应用详情页申请建库，建库成功后方可正常使用入库、检索、删除三个接口**。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         * tag_logic 检索时tag之间的逻辑， 0：逻辑and，1：逻辑or
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value same_hq_search(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_search, null, data, null);

            return result;
        }
        
        /**
         * same_hq_delete_by_image
         * 删除相同图
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value same_hq_delete_by_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_delete, null, data, null);

            return result;
        }
        
        /**
         * same_hq_delete_by_sign
         * 删除相同图
         * @param cont_sign 图片签名（和image二选一，image优先级更高）
         * options 可选参数:
         */
        Json::Value same_hq_delete_by_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_same_hq_delete, null, data, null);

            return result;
        }
        
        /**
         * similar_add
         * 该请求用于实时检索相似图片集合。即对于输入的一张图片（可正常解码，且长宽比适宜），返回自建图库中相似的图片集合。相似图检索包含入库、检索、删除三个子接口；**在正式使用之前请在[控制台](https://console.bce.baidu.com/ai/#/ai/imagesearch/overview/index)创建应用后，在应用详情页申请建库，建库成功后方可正常使用入库、检索、删除三个接口。**
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 检索时原样带回,最长256B。
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         */
        Json::Value similar_add(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_add, null, data, null);

            return result;
        }
        
        /**
         * similar_search
         * 相似图检索包含入库、检索、删除三个子接口；**在正式使用之前请在[控制台](https://console.bce.baidu.com/ai/#/ai/imagesearch/overview/index)创建应用后，在应用详情页申请建库，建库成功后方可正常使用入库、检索、删除三个接口。**
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * tags 1 - 65535范围内的整数，tag间以逗号分隔，最多2个tag。样例："100,11" ；检索时可圈定分类维度进行检索
         * tag_logic 检索时tag之间的逻辑， 0：逻辑and，1：逻辑or
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value similar_search(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_search, null, data, null);

            return result;
        }
        
        /**
         * similar_delete_by_image
         * 删除相似图
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value similar_delete_by_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_delete, null, data, null);

            return result;
        }
        
        /**
         * similar_delete_by_sign
         * 删除相似图
         * @param cont_sign 图片签名（和image二选一，image优先级更高）
         * options 可选参数:
         */
        Json::Value similar_delete_by_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_similar_delete, null, data, null);

            return result;
        }
        
        /**
         * product_add
         * 该请求用于实时检索商品类型图片相同或相似的图片集合，适用于电商平台或商品展示等场景，即对于输入的一张图片（可正常解码，且长宽比适宜），返回自建商品库中相同或相似的图片集合。商品检索包含入库、检索、删除三个子接口；**在正式使用之前请在[控制台](https://console.bce.baidu.com/ai/#/ai/imagesearch/overview/index)创建应用后，在应用详情页申请建库，建库成功后方可正常使用入库、检索、删除三个接口**。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * brief 检索时原样带回,最长256B。**请注意，检索接口不返回原图，仅反馈当前填写的brief信息，所以调用该入库接口时，brief信息请尽量填写可关联至本地图库的图片id或者图片url、图片名称等信息**
         * class_id1 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * class_id2 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         */
        Json::Value product_add(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_add, null, data, null);

            return result;
        }
        
        /**
         * product_search
         * 完成入库后，可使用该接口实现商品检索。**请注意，检索接口不返回原图，仅反馈当前填写的brief信息，请调用入库接口时尽量填写可关联至本地图库的图片id或者图片url等信息**
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * class_id1 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * class_id2 商品分类维度1，支持1-60范围内的整数。检索时可圈定该分类维度进行检索
         * pn 分页功能，起始位置，例：0。未指定分页时，默认返回前300个结果；接口返回数量最大限制1000条，例如：起始位置为900，截取条数500条，接口也只返回第900 - 1000条的结果，共计100条
         * rn 分页功能，截取条数，例：250
         */
        Json::Value product_search(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_search, null, data, null);

            return result;
        }
        
        /**
         * product_delete_by_image
         * 删除商品
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value product_delete_by_image(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_delete, null, data, null);

            return result;
        }
        
        /**
         * product_delete_by_sign
         * 删除商品
         * @param cont_sign 图片签名（和image二选一，image优先级更高）
         * options 可选参数:
         */
        Json::Value product_delete_by_sign(
            std::string const & cont_sign,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["cont_sign"] = cont_sign;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_product_delete, null, data, null);

            return result;
        }
        

    };
}
#endif