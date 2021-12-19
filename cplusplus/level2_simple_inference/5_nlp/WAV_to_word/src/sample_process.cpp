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
#include "acllite/AclLiteModel.h"
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

AclLiteError SampleProcess::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl init failed");
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl open device %d failed", deviceId_);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("open device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl create context failed");
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl create stream failed");
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("create stream success");

    // get run mode
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);

    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    ACLLITE_LOG_INFO("get run mode success");
    return ACLLITE_OK;
}

AclLiteError SampleProcess::DumpModelOutputResult(vector<InferenceOutput>& modelOutput){
    stringstream ssfile;
    size_t outputNum = 1;
    static int executeNum = 0;
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
            ACLLITE_LOG_ERROR("create output file [%s] failed", outputFileName.c_str());
            return ACLLITE_ERROR;
        }
    }
    return ACLLITE_OK;

}

AclLiteError SampleProcess::CreateInput() {
    
    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Acl malloc image buffer failed.");
        return ACLLITE_ERROR;
    }

    AclLiteError ret = model_.CreateInput(inputBuf_, inputDataSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError SampleProcess::Process()
{
    // model init
    const char* omModelPath = "../model/voice.om";
    AclLiteError ret = model_.Init(omModelPath);//
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("execute LoadModelFromFileWithMem failed");
        return ACLLITE_ERROR;
    }

    inputDataSize_ = model_.GetModelInputSize(0);

    ret = CreateInput();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create model input failed");
        return ACLLITE_ERROR;
    }

    // loop begin
    string testFile[] = {
        "../data/nihao.bin",
        "../data/xinpian.bin"
    };
    //string cmd("mkdir -p ./output");
    //system(cmd.c_str());
    for (size_t index = 0; index < sizeof(testFile) / sizeof(testFile[0]); ++index) {
        ACLLITE_LOG_INFO("start to process file:%s", testFile[index].c_str());
        // model process
        uint32_t devBufferSize = 0;
        void* picDevBuffer = nullptr;
        ReadBinFile(testFile[index], picDevBuffer, devBufferSize);
        if (picDevBuffer == nullptr) {
            ACLLITE_LOG_ERROR("get pic buffer failed,index is %zu", index);
            return ACLLITE_ERROR;
        }

        if (runMode_ == ACL_HOST) {
            aclError ret = aclrtMemcpy(inputBuf_, devBufferSize,
                                       picDevBuffer, devBufferSize,
                                       ACL_MEMCPY_HOST_TO_DEVICE);
            if (ret != ACL_SUCCESS) {
                ACLLITE_LOG_ERROR("Copy resized image data to device failed.");
                return ACLLITE_ERROR;
            }
        } else {
        aclError ret = aclrtMemcpy(inputBuf_, devBufferSize,
                                       picDevBuffer, devBufferSize,
                                       ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
                ACLLITE_LOG_ERROR("Copy resized image data to device failed.");
                return ACLLITE_ERROR;
            }
        }

        std::vector<InferenceOutput> inferOutputs;
        ret = model_.Execute(inferOutputs);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Execute model inference failed");
            return ACLLITE_ERROR;
        }

        // print the top 5 confidence values with indexes.use function DumpModelOutputResult
        // if want to dump output result to file in the current directory
        DumpModelOutputResult(inferOutputs);
    }
    ret = system("python3 ../scripts/postprocess.py");
    if (ret){
        ACLLITE_LOG_INFO("execut python3 ../scripts/postprocess.py error");
        return ACLLITE_ERROR;
    }
    // loop end
    return ACLLITE_OK;
}

void SampleProcess::DestroyResource()
{
    model_.DestroyInput();
    model_.DestroyResource();

    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("reset device failed");
    }
    ACLLITE_LOG_INFO("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("finalize acl failed");
    }
    ACLLITE_LOG_INFO("end to finalize acl");

}
