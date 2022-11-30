/**
* Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
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
#ifndef PLUSPLUS_LEVEL2_SIMPLE_INFERENCE_0_DATA_PROCESS_JPEGE_SRC_MAIN_H
#define PLUSPLUS_LEVEL2_SIMPLE_INFERENCE_0_DATA_PROCESS_JPEGE_SRC_MAIN_H

#pragma once
#include <iostream>
#include "acl/acl.h"
#include <fstream>
#include <cstring>
#include <cstdint>
#include <sys/stat.h>
#include "acl/ops/acl_dvpp.h"
#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stderr, "[ERROR]  " fmt "\n", ##args)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef struct PicDesc {
    std::string picName;
    int width;
    int height;
}PicDesc;

uint32_t AlignmentHelper(uint32_t origSize, uint32_t alignment);
uint32_t ComputeEncodeInputSize(int inputWidth, int inputHeight);
char *GetPicdevBuffer4Jpege(const PicDesc &picDesc, uint32_t &PicBufferSize);
void SetInput4Jpege(char *inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight);
Result SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize);
void DestroyEncodeResource();
void DestroyResource();


#endif