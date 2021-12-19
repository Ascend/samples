/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File main.cpp
* Description: dvpp sample main func
*/

#include <iostream>
#include <stdlib.h>
#include <dirent.h>

#include "classify_process.h"
#include "acllite/AclLiteError.h"
#include "acllite/AclLiteResource.h"

using namespace std;

namespace {
const uint32_t kModelWidth = 512;
const uint32_t kModelHeight = 512;
const std::string kModelPath = "../model/human_segmentation.om";
}

int main(int argc, char *argv[]) {
    //检查应用程序执行时的输入,程序执行的参数为输入视频文件路径
    if((argc < 2) || (argv[1] == nullptr)){
        ACLLITE_LOG_ERROR("Please input: ./main <image_dir>");
        return ACLLITE_ERROR;
    }

    //init acl resource
    AclLiteResource aclDev;
    AclLiteError ret = aclDev.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }
    //实例化分类推理对象,参数为分类模型路径,模型输入要求的宽和高
    ClassifyProcess classify(kModelPath, kModelWidth, kModelHeight);
    //初始化分类推理的acl资源, 加载模型和申请推理输入使用的内存
    ret = classify.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Classification Init resource failed");
        return ACLLITE_ERROR;
    }
    //使用opencv打开视频文件
    string videoFile = string(argv[1]);
    cv::VideoCapture capture(videoFile);
    if (!capture.isOpened()) {
        cout << "Movie open Error" << endl;
        return ACLLITE_ERROR;
    }
    //逐帧推理
    while(1) {
        //读取一帧图片
        cv::Mat frame;
        if (!capture.read(frame))
        {
            ACLLITE_LOG_INFO("Video capture return false");
            break;
        }
        //对帧图片进行预处理
        AclLiteError ret = classify.Preprocess(frame);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",
                      videoFile.c_str());
            continue;
        }
        //将预处理的图片送入模型推理,并获取推理结果
        std::vector<InferenceOutput> inferOutputs;
        ret = classify.Inference(inferOutputs);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return ACLLITE_ERROR;
        }

        //解析推理输出,并将推理得到的物体类别,置信度和图片送到presenter server显示
        ret = classify.Postprocess(frame, inferOutputs);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return ACLLITE_ERROR;
        }
    }

    ACLLITE_LOG_INFO("Execute video classification success");
    return ACLLITE_OK;
}
