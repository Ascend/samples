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
*/

#pragma once
#include "acl/acl.h"
#include <iostream>
#include <cstdint>
#include "acl/ops/acl_dvpp.h"

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR] " fmt "\n", ##args)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef struct PicDesc {
    std::string picName;
    int width;
    int height;
}PicDesc;

static void* GetDeviceBufferOfPicture(const PicDesc &picDesc, uint32_t &devPicBufferSize);
static char* ReadBinFile(std::string fileName, uint32_t& fileSize);
static Result SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize);
void SetInput(void *inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight);
void GetOutput(void **outputBuffer, int &outputSize);
void DestroyResource();
void DestroyDecodeResource();

int32_t deviceId_ = 0;
aclrtContext context_ = nullptr;
aclrtStream stream_ = nullptr;
acldvppChannelDesc *dvppChannelDesc_;

void* decodeOutDevBuffer_; // decode output buffer
acldvppPicDesc *decodeOutputDesc_; //decode output desc
uint32_t decodeDataSize_;

void *inDevBuffer_;  // decode input buffer
uint32_t inDevBufferSize_; // dvpp input buffer size

uint32_t inputWidth_; // input pic width
uint32_t inputHeight_; // input pic height
aclrtRunMode runMode;

