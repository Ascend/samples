
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
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stderr, "[ERROR]  " fmt "\n", ##__VA_ARGS__)

typedef enum Result { SUCCESS = 0, FAILED = 1 } Result;

typedef struct InputArea {
    const char *inputFileName;
    acldvppPixelFormat inputFormat;
    acldvppPixelFormat outputFormat;
    uint32_t inputWidth;
    uint32_t inputHeight;
} InputArea;

typedef struct Area {
    uint32_t cropLeftOffset;
    uint32_t cropTopOffset;
} Area;

class Utils {
public:
    static Result VpcReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize);

    static Result WriteToFile(const char *fileName, const void *dataDev, uint32_t dataSize);

    static Result ReadFile(uint32_t fileLen, FILE *fp, aclrtRunMode &runMode, void *&dataHost,
        aclrtMemcpyKind &copyKind);

    static void Deleter(FILE *fp);
};
