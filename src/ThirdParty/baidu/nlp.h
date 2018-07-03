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

#ifndef __AIP_NLP_H__
#define __AIP_NLP_H__

#include "base/base.h"

namespace aip {

    class Nlp: public AipBase
    {
    public:
        
        std::string _lexer =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/lexer";
        
        std::string _lexer_custom =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/lexer_custom";
        
        std::string _dep_parser =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/depparser";
        
        std::string _word_embedding =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/word_emb_vec";
        
        std::string _dnnlm_cn =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/dnnlm_cn";
        
        std::string _word_sim_embedding =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/word_emb_sim";
        
        std::string _simnet =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/simnet";
        
        std::string _comment_tag =
            "https://aip.baidubce.com/rpc/2.0/nlp/v2/comment_tag";
        
        std::string _sentiment_classify =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/sentiment_classify";
        
        std::string _keyword =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/keyword";
        
        std::string _topic =
            "https://aip.baidubce.com/rpc/2.0/nlp/v1/topic";
        

        Nlp(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }

        
        /**
         * lexer
         * 词法分析接口向用户提供分词、词性标注、专名识别三大功能；能够识别出文本串中的基本词汇（分词），对这些词汇进行重组、标注组合后词汇的词性，并进一步识别出命名实体。
         * @param text 待分析文本（目前仅支持UTF8编码），长度不超过65536字节
         * options 可选参数:
         */
        Json::Value lexer(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_lexer, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * lexer_custom
         * 词法分析接口向用户提供分词、词性标注、专名识别三大功能；能够识别出文本串中的基本词汇（分词），对这些词汇进行重组、标注组合后词汇的词性，并进一步识别出命名实体。
         * @param text 待分析文本（目前仅支持UTF8编码），长度不超过65536字节
         * options 可选参数:
         */
        Json::Value lexer_custom(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_lexer_custom, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * dep_parser
         * 依存句法分析接口可自动分析文本中的依存句法结构信息，利用句子中词与词之间的依存关系来表示词语的句法结构信息（如“主谓”、“动宾”、“定中”等结构关系），并用树状结构来表示整句的结构（如“主谓宾”、“定状补”等）。
         * @param text 待分析文本（目前仅支持UTF8编码），长度不超过256字节
         * options 可选参数:
         * mode 模型选择。默认值为0，可选值mode=0（对应web模型）；mode=1（对应query模型）
         */
        Json::Value dep_parser(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_dep_parser, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * word_embedding
         * 词向量表示接口提供中文词向量的查询功能。
         * @param word 文本内容（UTF8编码），最大64字节
         * options 可选参数:
         */
        Json::Value word_embedding(
            std::string const & word,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["word"] = word;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_word_embedding, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * dnnlm_cn
         * 中文DNN语言模型接口用于输出切词结果并给出每个词在句子中的概率值,判断一句话是否符合语言表达习惯。
         * @param text 文本内容（UTF8编码），最大512字节，不需要切词
         * options 可选参数:
         */
        Json::Value dnnlm_cn(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_dnnlm_cn, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * word_sim_embedding
         * 输入两个词，得到两个词的相似度结果。
         * @param word_1 词1（UTF8编码），最大64字节
         * @param word_2 词1（UTF8编码），最大64字节
         * options 可选参数:
         * mode 预留字段，可选择不同的词义相似度模型。默认值为0，目前仅支持mode=0
         */
        Json::Value word_sim_embedding(
            std::string const & word_1,
            std::string const & word_2,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["word_1"] = word_1;
            data["word_2"] = word_2;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_word_sim_embedding, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * simnet
         * 短文本相似度接口用来判断两个文本的相似度得分。
         * @param text_1 待比较文本1（UTF8编码），最大512字节
         * @param text_2 待比较文本2（UTF8编码），最大512字节
         * options 可选参数:
         * model 默认为"BOW"，可选"BOW"、"CNN"与"GRNN"
         */
        Json::Value simnet(
            std::string const & text_1,
            std::string const & text_2,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text_1"] = text_1;
            data["text_2"] = text_2;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_simnet, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * comment_tag
         * 评论观点抽取接口用来提取一条评论句子的关注点和评论观点，并输出评论观点标签及评论观点极性。
         * @param text 评论内容（UTF8编码），最大10240字节
         * options 可选参数:
         * type 评论行业类型，默认为4（餐饮美食）
         */
        Json::Value comment_tag(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_comment_tag, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * sentiment_classify
         * 对包含主观观点信息的文本进行情感极性类别（积极、消极、中性）的判断，并给出相应的置信度。
         * @param text 文本内容（UTF8编码），最大102400字节
         * options 可选参数:
         */
        Json::Value sentiment_classify(
            std::string const & text,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["text"] = text;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_sentiment_classify, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * keyword
         * 文章标签服务能够针对网络各类媒体文章进行快速的内容理解，根据输入含有标题的文章，输出多个内容标签以及对应的置信度，用于个性化推荐、相似文章聚合、文本内容分析等场景。
         * @param title 篇章的标题，最大80字节
         * @param content 篇章的正文，最大65535字节
         * options 可选参数:
         */
        Json::Value keyword(
            std::string const & title,
            std::string const & content,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["title"] = title;
            data["content"] = content;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_keyword, null, data.toStyledString(), null);

            return result;
        }
        
        /**
         * topic
         * 对文章按照内容类型进行自动分类，首批支持娱乐、体育、科技等26个主流内容类型，为文章聚类、文本内容分析等应用提供基础技术支持。
         * @param title 篇章的标题，最大80字节
         * @param content 篇章的正文，最大65535字节
         * options 可选参数:
         */
        Json::Value topic(
            std::string const & title,
            std::string const & content,
            const std::map<std::string, std::string> & options)
        {
            Json::Value data;
            
            data["title"] = title;
            data["content"] = content;

            std::map<std::string, std::string>::const_iterator it;
            for (it = options.begin(); it != options.end(); it++)
            {
                data[it->first] = it->second;
            }

            Json::Value result =
                this->request(_topic, null, data.toStyledString(), null);

            return result;
        }
        
    };
}
#endif