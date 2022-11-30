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
#include "object_detect.h"
#include "utils.h"
using namespace std;

namespace {
uint32_t g_modelWidth = 416;
uint32_t g_modelHeight = 416;
const char* g_modelPath = "../model/yolov3.om";
}

int main(int argc, char *argv[])
{
    // Check input args: the path of input picture, and ignore hidden dotfile directory
    uint32_t argNum = 2;
    if ((argc != argNum) || (argv[1] == nullptr)) {
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
    // Instantiate the object detection class
    ObjectDetect detect(g_modelPath, g_modelWidth, g_modelHeight);
    // Initialize the acl resources, dvpp, load model,
    // and malloc input memory of input which is const
    Result ret = detect.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }

    // Get all the image file path in the image directory
    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    Utils::GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ERROR_LOG("Failed to read image from %s, and hidden dotfile "
                  "directory is ignored", inputImageDir.c_str());
        return FAILED;
    }

    ImageData image;
    for (string imageFile : fileVec) {
        Utils::ReadImageFile(image, imageFile);
        if (image.data == nullptr) {
            ERROR_LOG("Read image %s failed", imageFile.c_str());
            return FAILED;
        }

        // Preprocess: copy image to device, convert to yuv, and resize
        ImageData resizedImage;
        ImageData yuvImage;
        Result ret = detect.Preprocess(resizedImage, image, yuvImage);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
                      imageFile.c_str());
            continue;
        }
        // Send the resized picture to the model for inference
        // and get the inference results
        aclmdlDataset* inferenceOutput = nullptr;
        ret = detect.Inference(inferenceOutput, resizedImage);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        // Analyze the inference output, mark the object category and
        // location by the inference result
        ret = detect.Postprocess(yuvImage, inferenceOutput, imageFile);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
