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

SampleProcess::SampleProcess():deviceId_(0), context_(nullptr), stream_(nullptr)
{
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}


Result SampleProcess::InitResource()
{
    // TODO:
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

Result SampleProcess::MainProcess(string input_path)
{
    vector<string> file_vec;

    // get filename for all input
    Utils::GetAllFiles(input_path, file_vec);
    if (file_vec.empty()) {
        ERROR_LOG("Failed to deal all empty path=%s.", input_path.c_str());
        return FAILED;
    }

    //  load model and get modelDesc(model description)
    const char* omModelPath = "../model/vgg_ssd.om";
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

    // create output dataset(for inference result saving) according to modelDesc
    ret = processModel.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    // create image_desc for saving input image.
    shared_ptr<ImageDesc> image_desc = nullptr;
    MAKE_SHARED_NO_THROW(image_desc, ImageDesc);
    if (image_desc == nullptr) {
        ERROR_LOG("Failed to MAKE_SHARED_NO_THROW for ImageDesc.");
        return FAILED;
    }

    // traversal files for preprocess, inference, and then post process
    for (string path : file_vec) {
        // arrange image information, if failed, skip this image
        if (!Utils::PreProcess(image_desc, path)) {
            continue;
        }

        // create input for the model inference.
        ret = processModel.CreateInput((void*) image_desc->data.get(), image_desc->size);
        if (ret != SUCCESS) {
            ERROR_LOG("execute CreateInput failed");
            return FAILED;
        }

        // execute inference.
        ret = processModel.Execute();
        if (ret != SUCCESS) {
            ERROR_LOG("execute inference failed");
            return FAILED;
        }

        // release model input buffer
        processModel.DestroyInput();

        aclmdlDataset *modelOutput = processModel.GetModelOutputData();
        if (modelOutput == nullptr) {
            ERROR_LOG("get model output data failed");
            return FAILED;
        }

        // post process.
        ret = Utils::PostProcess(path, modelOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("pull model output data failed");
            return FAILED;
        }

    }
    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    // clear resources.
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

    //TODO:
    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");

}
