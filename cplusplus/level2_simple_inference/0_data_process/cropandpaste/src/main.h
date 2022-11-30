/**
* @file utils.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include <iostream>
#include <unistd.h>
#include <dirent.h>
#include <fstream>
#include <cstring>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <map>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include <cstdint>

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stderr, "[ERROR] " fmt "\n", ##args)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef struct PicDesc {
    std::string picName;
    int width;
    int height;
}PicDesc;

int32_t deviceId_;
aclrtContext context_;
aclrtStream stream_;

aclvdecChannelDesc *vdecChannelDesc_;
acldvppStreamDesc *streamInputDesc_;
acldvppPicDesc *picOutputDesc_;
acldvppChannelDesc *dvppChannelDesc_;
uint32_t inBufferSize;
uint32_t inputWidth;
uint32_t inputHeight;
aclrtRunMode runMode;

PicDesc inPicDesc;
PicDesc outPicDesc;
