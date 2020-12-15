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
* Description: AIPainting sample main func
*/

#include <cstdint>
#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include "painting_process.h"
#include "utils.h"

using namespace std;

#define ModelInputNums 2

//模型路径
const char* omModelPath = "../model/AIPainting_v2.om";
//该模型有2个输入,分别为一个9维的object vector和一个255*255*17的layout tensor(NHWC)
uint32_t ModelInputSize[ModelInputNums][4] = {{9, 1, 1, 1},
                                              {1, 256, 256, 17}};
bool g_isDevice = false;

int main(int argc, char *argv[]) {

    //实例化推理对象,参数为模型路径和模型输入大小
    PaintingProcess painting(omModelPath, ModelInputSize);
    //初始化推理的acl资源, 模型和内存
    Result ret = painting.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("AIPainting init resource failed");
        return FAILED;
    }

    //推理
    while(1)
    {
        //预处理,接收用户的交互请求
        Result ret = painting.Preprocess();
        if (ret != SUCCESS) {
            INFO_LOG("Continue receiving...");
        }
        else{
            //将模型输入送入模型推理,获取推理结果
            aclmdlDataset* inferenceOutput = nullptr;
            ret = painting.Inference(inferenceOutput);
            if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
                ERROR_LOG("Inference model inference output data failed");
                return FAILED;
            }
            //解析推理输出图片,推流到PresenterServer
            ret = painting.Postprocess(inferenceOutput);
            if (ret != SUCCESS) {
                ERROR_LOG("Process model inference output data failed");
                return FAILED;
            }
            INFO_LOG("Inference once success");
        }
    }

    return SUCCESS;
}
