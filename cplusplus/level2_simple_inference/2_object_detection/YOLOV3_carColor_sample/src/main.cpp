/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include <vector>
#include "object_detect.h"
#include "color_classify.h"
#include "AclLiteUtils.h"
#include "AclLiteError.h"
#include "AclLiteResource.h"
using namespace std;

int main(int argc, char *argv[])
{
    int argNum = 2;
    if ((argc < argNum) || (argv[1] == nullptr)) {
        ACLLITE_LOG_ERROR("Please input: ./main <image_dir>");
        return ACLLITE_ERROR;
    }
    AclLiteResource aclDev;
    AclLiteError ret = aclDev.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    ObjectDetect detect;
    ret = detect.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init detect failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    ColorClassify classify;
    ret = classify.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init classify failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return ACLLITE_ERROR;
    }

    ImageData image;
    for (string imageFile : fileVec) {
        ReadJpeg(image, imageFile);
        if (image.data == nullptr) {
            ACLLITE_LOG_ERROR("Read image %s failed", imageFile.c_str());
            return ACLLITE_ERROR;
        }

        ImageData resizedImage, yuvImage;
        ret = detect.PreProcess(resizedImage, image, yuvImage);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",
                              imageFile.c_str());
            continue;
        }
        std::vector<InferenceOutput> detectInferOutput;
        ret = detect.Inference(detectInferOutput, resizedImage);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return ACLLITE_ERROR;
        }
        vector<CarInfo> carInfo;
        ret = detect.PostProcess(image, detectInferOutput, carInfo);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return ACLLITE_ERROR;
        }

        int flag = 0;
        ret = classify.PreProcess(yuvImage, carInfo, flag);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference input data failed");
            return ACLLITE_ERROR;
        }
        if (flag ==1) {
            continue;
        }
        std::vector<InferenceOutput> classifyInferOutput;
        ret = classify.Inference(carInfo, classifyInferOutput);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference input data failed");
            return ACLLITE_ERROR;
        }
        
        ret = classify.PostProcess(classifyInferOutput, carInfo, imageFile);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return ACLLITE_ERROR;
        }
    }
    ACLLITE_LOG_INFO("Execute sample success");
    return ACLLITE_OK;
}
