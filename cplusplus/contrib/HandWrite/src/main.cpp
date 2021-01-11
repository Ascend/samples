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
#include<sstream>
#include <iostream>
#include<fstream>
#include<string>
#include<vector>
#include <stdlib.h>
#include <dirent.h>

#include "object_detect.h"
#include "utils.h"
#include "camera.h"
#include <sys/time.h>
using namespace std;

namespace {
uint32_t kModelWidth = 112;
uint32_t kModelHeight = 112;
const char* kModelPath = "../model/resnet.om";

shared_ptr<ImageData> g_imagedata = make_shared<ImageData>();
}

int main(int argc, char *argv[]) {
    bool bSetChannelId = true;
    int channelId = 0;
    //检查应用程序执行时的输入,程序执行要求输入camera channel
    if((argc < 2) || (argv[1] == nullptr)){
        INFO_LOG("Please input: ./main ChannelID(Channel-0  Channel-1), default Channel-0\n");
        bSetChannelId =false;
    }
    //实例化目标检测对象,参数为分类模型路径,模型输入要求的宽和高
    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);
    //初始化分类推理的acl资源, 模型和内存
    Result ret = detect.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed\n");
        return FAILED;
    }

    //获取camera channel id
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
    ImageResults inferenceOutput;


    // read word dict
    std::ifstream in("./lexicon3755.txt");
    std::string name;
    vector<string> dict;
    if(in)
    {
        int i=0;
        while (getline (in, name))
        {
            dict.push_back(name);
        }
    }

    // set 2 slices one predict
    bool istwo = true;
    while(1)
    {
        if(istwo)
        {
            istwo=false;
            continue;
        }
        istwo=true;
        //逐张图片推理
        cameraDevice.Read(channelId, *(g_imagedata.get()));
        if (g_imagedata->data == nullptr) {
            ERROR_LOG("Read image %d failed\n", channelId);
            return FAILED;
        }

        inferenceOutput = detect.Inference(*(g_imagedata.get()), dict);

        if ((ret != SUCCESS)) {

            ERROR_LOG("Inference model inference output data failed\n");
            return FAILED;
        }

        ret = detect.Postprocess(*(g_imagedata.get()), inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed\n");
            return FAILED;
        }
    }

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
