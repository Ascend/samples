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
#include "camera.h"
#include <sys/time.h>
using namespace std;

namespace {
uint32_t kModelWidth = 304;
uint32_t kModelHeight = 300;
const char* kModelPath = "../model/face_detection.om";

shared_ptr<ImageData> g_imagedata = make_shared<ImageData>();
}

int main(int argc, char *argv[]) {
    bool bSetChannelId = true;
    int channelId = 0;
    //Check the input when the application is executed, the program execution requires the input of the camera channel
    if((argc < 2) || (argv[1] == nullptr)){
        INFO_LOG("Please input: ./main ChannelID(Channel-0  Channel-1), default Channel-0\n");
        bSetChannelId =false;
    }
    //Instantiate the target detection object, the parameter is the classification model path, the width and height required by the model input
    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);
    //Initialize the acl resources, models and memory for classification inference
    Result ret = detect.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed\n");
        return FAILED;
    }

    //Get camera channel id
    if(bSetChannelId)
    {
        string channelName = string(argv[1]);
        Utils::GetChannelID(channelName, channelId);
        if(0xFF == channelId){
            INFO_LOG("channelId = %d  ERROR \n", channelId);
            return FAILED;
        }
    }

    Camera  cameraDevice(channelId);
    if(false == cameraDevice.IsOpened(channelId))
    {
        if (cameraDevice.Open(channelId)) {
            ERROR_LOG("Failed to open channelId =%d.\n", channelId);
            return FAILED;
        }
    }

    void * buffer = nullptr;
    int size = cameraDevice.GetCameraDataSize(channelId);

    aclError aclRet = acldvppMalloc(&buffer, size);
    g_imagedata->data.reset((uint8_t*)buffer, [](uint8_t* p) { acldvppFree((void *)p); });

    ImageData resizedImage;
    aclmdlDataset* inferenceOutput = nullptr;

    while(1)
    {

        //Reasoning picture by picture
        cameraDevice.Read(channelId, *(g_imagedata.get()));
        if (g_imagedata->data == nullptr) {
            ERROR_LOG("Read image %d failed\n", channelId);
            return FAILED;
        }

        //Preprocess the picture: read the picture and zoom the picture to the size required by the model input
        ret = detect.Preprocess(resizedImage, *(g_imagedata.get()));
        if (ret != SUCCESS) {
            ERROR_LOG("Preprocess image %d failed, continue to read next\n",  channelId);
            return FAILED;
        }

        //Send the preprocessed pictures to the model for inference and get the inference results
        ret = detect.Inference(inferenceOutput, resizedImage);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed\n");
            return FAILED;
        }

        //Analyze the inference output and mark the object category and location obtained by the inference on the picture
        ret = detect.Postprocess(*(g_imagedata.get()), inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed\n");
            return FAILED;
        }
    }

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
