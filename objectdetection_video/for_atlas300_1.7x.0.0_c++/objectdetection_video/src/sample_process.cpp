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

* File sample_process.cpp
* Description: handle acl resource
*/
#include "sample_process.h"
#include <iostream>
#include "model_process.h"
#include "acl/acl.h"
#include "utils.h"

using namespace std;
using namespace cv;
using namespace ascend::presenter;

SampleProcess::SampleProcess():deviceId_(0), context_(nullptr), stream_(nullptr)
{
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}

Result SampleProcess::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed");
        return FAILED;
    }
    INFO_LOG("acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("open device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create stream failed");
        return FAILED;
    }
    INFO_LOG("create stream success");

    return SUCCESS;
}

Result SampleProcess::VideoProcess(string input_path)
{
    const char* omModelPath = "../model/yolov3_BGR.om";
    
    Mat frame;

    VideoCapture capture(input_path);
    if(!capture.isOpened())
    {
        ERROR_LOG("video open failed");
        return FAILED;
    }

    // model init
    ModelProcess processModel;
    aclError ret = processModel.LoadModelFromFile(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFile failed");
        return FAILED;
    }
    ret = processModel.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }
    ret = processModel.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    	
    Channel *channel = nullptr;
    OpenChannelParam param;
    param.host_ip = "192.168.1.223";  //IP address of Presenter Server
    param.port = 7008;  //port of present service
    param.channel_name = "objectdetection-video";
    param.content_type = ContentType::kVideo;  //content type is IMAGE
    INFO_LOG("OpenChannel start");
    PresenterErrorCode errorCode = OpenChannel(channel, param);
    if (errorCode != PresenterErrorCode::kNone) {
	ERROR_LOG("OpenChannel failed %d",static_cast<int>(errorCode));
	return FAILED;
    }
	
    uint32_t size = MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT*3;
    void *image_buf_ptr = nullptr;	
    ret = aclrtMalloc(&image_buf_ptr, (size_t)(size), ACL_MEM_MALLOC_HUGE_FIRST);  
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("new image buffer failed, size=%d!", size);
	return FAILED;
    }
	
    ret = processModel.CreateInput(image_buf_ptr, size);
    if (ret != SUCCESS) {
	ERROR_LOG("execute CreateInput failed");
	return FAILED;
    }
	
    while(1) {
        if(!capture.read(frame))
        {
            INFO_LOG("Video capture return false");
            break;
        }
     
	cv::Mat mat_rs;
	cv::resize(frame, mat_rs, cv::Size(MODEL_INPUT_WIDTH,MODEL_INPUT_HEIGHT));
	if (mat_rs.empty()) {
            continue;
	}

        ret = aclrtMemcpy((u_int8_t *)image_buf_ptr, size, mat_rs.ptr<u_int8_t>(), size, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            continue;
        }

        ret = processModel.Execute();
        if (ret != SUCCESS) {
            ERROR_LOG("execute inference failed");
            return FAILED;
        }

        aclmdlDataset *modelOutput = processModel.GetModelOutputData();
        if (modelOutput == nullptr) {
            ERROR_LOG("get model output data failed");
            return FAILED;
        }

        ret = Utils::PresentOutputFrame(channel , modelOutput, frame);
        INFO_LOG("PresentOutputFrame end");
        if (ret != SUCCESS) {
            ERROR_LOG("pull model output data failed");
            return FAILED;
        }
        
    }
    aclrtFree(image_buf_ptr);
    delete channel;
    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}
