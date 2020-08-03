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
const char* kModelPath = "../model/yolov3_yuv.om";
}

int main(int argc, char *argv[]) {
    //检查应用程序执行时的输入,程序执行要求输入图片目录参数
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
    //实例化目标检测对象,参数为分类模型路径,模型输入要求的宽和高
    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);
    //初始化分类推理的acl资源, 模型和内存
    Result ret = detect.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }

    //获取图片目录下所有的图片文件名
    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    Utils::GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ERROR_LOG("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return FAILED;
    }

    //逐张图片推理
    ImageData image;
    for (string imageFile : fileVec) {
        Utils::ReadImageFile(image, imageFile);
        if (image.data == nullptr) {
            ERROR_LOG("Read image %s failed", imageFile.c_str());
            return FAILED;
        }

        //预处理图片:读取图片,讲图片缩放到模型输入要求的尺寸
        ImageData resizedImage;
        Result ret = detect.Preprocess(resizedImage, image);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }
        //将预处理的图片送入模型推理,并获取推理结果
        aclmdlDataset* inferenceOutput = nullptr;
        ret = detect.Inference(inferenceOutput, resizedImage);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        //解析推理输出,并将推理得到的物体类别和位置标记到图片上
        ret = detect.Postprocess(image, inferenceOutput, imageFile);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
