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
#include "colorize_process.h"
#include "atlasutil/atlas_utils.h"
#include "atlasutil/atlas_error.h"
using namespace std;

namespace {
uint32_t kModelWidth = 224;
uint32_t kModelHeight = 224;
const char* kModelPath = "../model/colorization.om";
}

int main(int argc, char *argv[]) {
    //检查应用程序执行时的输入,程序执行要求输入图片目录参数
    if((argc < 2) || (argv[1] == nullptr)){
        ATLAS_LOG_ERROR("Please input: ./main <image_dir>");
        return 1;
    }
    //实例化分类推理对象,参数为分类模型路径,模型输入要求的宽和高
    ColorizeProcess colorize(kModelPath, kModelWidth, kModelHeight);
    //初始化分类推理的acl资源, 模型和内存
    AtlasError ret = colorize.init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Classification Init resource failed");
        return 1;
    }
    //获取图片目录下所有的图片文件名
    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ATLAS_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return 1;
    }
    //逐张图片推理
    for (string imageFile : fileVec) {
        //预处理图片:读取图片,讲图片缩放到模型输入要求的尺寸
        ret = colorize.preprocess(imageFile);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }
        //将预处理的图片送入模型推理,并获取推理结果
        std::vector<InferenceOutput> inferenceOutput;
        ret = colorize.inference(inferenceOutput);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        //解析推理输出,并将推理得到的物体类别标记到图片上
        ret = colorize.postprocess(imageFile, inferenceOutput);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Process model inference output data failed");
            return 1;
        }
    }

    ATLAS_LOG_INFO("Execute sample success");
    return 0;
}
