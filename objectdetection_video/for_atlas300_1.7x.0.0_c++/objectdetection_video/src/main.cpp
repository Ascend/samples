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

#include "object_detect.h"
#include "utils.h"
using namespace std;

namespace {
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
const char* kModelPath = "../model/yolov3_BGR.om";
const char* kAppConf = "../script/object_detection.conf";
}

int main(int argc, char *argv[]) {
    //检查应用程序执行时的输入,程序执行的参数为输入视频文件路径
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
    //实例化目标检测类,参数为分类模型路径,模型输入要求的宽和高
    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);
    //初始化分类推理的acl资源, 加载模型和申请推理输入使用的内存
    Result ret = detect.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }

    //使用opencv打开视频文件
    string videoFile = string(argv[1]);
    printf("open %s\n", videoFile.c_str());
    cv::VideoCapture capture(videoFile);
    if (!capture.isOpened()) {
        cout << "Movie open Error" << endl;
        return FAILED;
    }
    //逐帧推理
    while(1) {
        //读取一帧图片
        cv::Mat frame;
        if (!capture.read(frame)) {
            INFO_LOG("Video capture return false");
            break;
        }
        //对帧图片进行预处理
        Result ret = detect.Preprocess(frame);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
                      videoFile.c_str());
            continue;
        }
        //将预处理的图片送入模型推理,并获取推理结果
        aclmdlDataset* inferenceOutput = nullptr;
        ret = detect.Inference(inferenceOutput);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        //解析推理输出,并将推理得到的物体类别, 位置, 置信度和图片送到presenter server显示
        ret = detect.Postprocess(frame, inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }

    INFO_LOG("Execute video object detection success");
    return SUCCESS;
}
