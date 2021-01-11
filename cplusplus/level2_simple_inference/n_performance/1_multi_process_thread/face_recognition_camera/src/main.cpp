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
#include <thread>
#include <time.h>
#include <fstream>

#include "resource_load.h"
#include "face_detection.h"
#include "face_feature_mask.h"
#include "face_recognition.h"
#include "face_post_process.h"
#include "utils.h"
#include "face_register.h"
#include "mind_camera.h"

using namespace std;
extern "C" {
#include "camera.h"
#include "driver/peripheral_api.h"
}

namespace {
uint32_t kModelWidth1 = 304;
uint32_t kModelHeight1 = 300;
const char* kModelPath1 = "../model/face_detection.om";

uint32_t kModelWidth2 = 40;
uint32_t kModelHeight2 = 40;
const char* kModelPath2 = "../model/vanillacnn.om";

uint32_t kModelWidth3 = 304;
uint32_t kModelHeight3 = 300;
const char* kModelPath3 = "../model/sphereface.om";

}

void CreateRegisterTask(aclrtContext context)
{
    aclrtSetCurrentContext(context);
    FaceRegister faceRegister;
    faceRegister.Process();
}

int main(int argc, char *argv[]) {

    //实例化目标检测对象,参数为分类模型路径,模型输入要求的宽和高
    ModelInfoParams param;
    param.modelPath1   = kModelPath1;
    param.modelWidth1  = kModelWidth1;
    param.modelHeight1 = kModelHeight1;
    param.modelPath2   = kModelPath2;
    param.modelWidth2  = kModelWidth2;
    param.modelHeight2 = kModelHeight2;
    param.modelPath3   = kModelPath3;
    param.modelWidth3  = kModelWidth3;
    param.modelHeight3 = kModelHeight3;

    //初始化分类推理的acl资源, 模型和内存
    Result ret = ResourceLoad::GetInstance().Init(param);
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed\n");
        return FAILED;
    }

    aclrtContext context;
    aclrtGetCurrentContext(&context);
    thread task1(CreateRegisterTask,  ref(context));
    task1.detach();

    MindCamera mindCamera;
    mindCamera.Process();

    ResourceLoad::GetInstance().DestroyResource();
    INFO_LOG("Execute sample success");
    return SUCCESS;
}
