/**
* @file sample_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "sample_process.h"
#include <iostream>
#include "acl/acl.h"
#include "atlasutil/atlas_model.h"
#include <memory>
#include <sstream>
#include <string.h>
#include <dirent.h>
#include <string>
#include <sys/stat.h>
#include <stdio.h>
#include <cstdlib>
using namespace std;

SampleProcess::SampleProcess() :deviceId_(0), context_(nullptr), stream_(nullptr){
}

SampleProcess::~SampleProcess(){
    DestroyResource();
}

AtlasError SampleProcess::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl init failed");
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl open device %d failed", deviceId_);
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("open device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl create context failed");
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl create stream failed");
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("create stream success");

    // get run mode
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);

    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR;
    }

    ATLAS_LOG_INFO("get run mode success");
    return ATLAS_OK;
}

AtlasError SampleProcess::DumpModelOutputResult(vector<InferenceOutput>& modelOutput){
    stringstream ssfile;
    size_t outputNum = 1;
    static int executeNum = 0;
//    string folderPath = "./output";
//    if (NULL == opendir(folderPath.c_str())) {
//        mkdir(folderPath.c_str(), 0775);
//    }
    for (size_t i = 0; i < outputNum; ++i) {
        ssfile << "./" << "output" << ++executeNum << "_" << i << ".bin";
        string outputFileName = ssfile.str();
        FILE *outputFile = fopen(outputFileName.c_str(), "wb");
        if (outputFile) {
            void* data = modelOutput[i].data.get();
            uint32_t len  = modelOutput[i].size;
            fwrite(data, len, sizeof(char), outputFile);
            fclose(outputFile);
            outputFile = nullptr;
        } else {
            ATLAS_LOG_ERROR("create output file [%s] failed", outputFileName.c_str());
            return ATLAS_ERROR;
        }
    }
    return ATLAS_OK;

}

AtlasError SampleProcess::CreateInput() {
    
    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ATLAS_LOG_ERROR("Acl malloc image buffer failed.");
        return ATLAS_ERROR;
    }

    AtlasError ret = model_.CreateInput(inputBuf_, inputDataSize_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create mode input dataset failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError SampleProcess::Process()
{
    // model init
    const char* omModelPath = "../model/voice.om";
    AtlasError ret = model_.Init(omModelPath);//
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("execute LoadModelFromFileWithMem failed");
        return ATLAS_ERROR;
    }

    inputDataSize_ = model_.GetModelInputSize(0);

    ret = CreateInput();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create model input failed");
        return ATLAS_ERROR;
    }

    // loop begin
    string testFile[] = {
        "../data/nihao.bin",
        "../data/xinpian.bin"
    };
    //string cmd("mkdir -p ./output");
    //system(cmd.c_str());
    for (size_t index = 0; index < sizeof(testFile) / sizeof(testFile[0]); ++index) {
        ATLAS_LOG_INFO("start to process file:%s", testFile[index].c_str());
        // model process
        uint32_t devBufferSize = 0;
        void* picDevBuffer = nullptr;
        ReadBinFile(testFile[index], picDevBuffer, devBufferSize);
        if (picDevBuffer == nullptr) {
            ATLAS_LOG_ERROR("get pic buffer failed,index is %zu", index);
            return ATLAS_ERROR;
        }

        if (runMode_ == ACL_HOST) {
            aclError ret = aclrtMemcpy(inputBuf_, devBufferSize,
                                       picDevBuffer, devBufferSize,
                                       ACL_MEMCPY_HOST_TO_DEVICE);
            if (ret != ACL_ERROR_NONE) {
                ATLAS_LOG_ERROR("Copy resized image data to device failed.");
                return ATLAS_ERROR;
            }
        } else {
        aclError ret = aclrtMemcpy(inputBuf_, devBufferSize,
                                       picDevBuffer, devBufferSize,
                                       ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
                ATLAS_LOG_ERROR("Copy resized image data to device failed.");
                return ATLAS_ERROR;
            }
        }

        std::vector<InferenceOutput> inferOutputs;
        ret = model_.Execute(inferOutputs);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Execute model inference failed");
            return ATLAS_ERROR;
        }

        // print the top 5 confidence values with indexes.use function DumpModelOutputResult
        // if want to dump output result to file in the current directory
        DumpModelOutputResult(inferOutputs);
    }
    ret = system("python3 ../scripts/postprocess.py");
    if (ret){
        ATLAS_LOG_INFO("execut python3 ../scripts/postprocess.py error");
        return ATLAS_ERROR;
    }
    // loop end
    return ATLAS_OK;
}

void SampleProcess::DestroyResource()
{
    model_.DestroyInput();
    model_.DestroyResource();

    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }
    ATLAS_LOG_INFO("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ATLAS_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("reset device failed");
    }
    ATLAS_LOG_INFO("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("finalize acl failed");
    }
    ATLAS_LOG_INFO("end to finalize acl");

}
