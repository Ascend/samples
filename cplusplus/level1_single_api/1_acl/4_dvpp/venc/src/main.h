/**
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
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR]  " fmt "\n", ##args)

#define RGBU8_IMAGE_SIZE(width, height) ((width) * (height) * 3)
#define YUV420SP_SIZE(width, height) ((width) * (height) * 3 / 2)

#define ALIGN_UP(num, align) (((num) + (align) - 1) & ~((align) - 1))
#define ALIGN_UP2(num) ALIGN_UP(num, 2)
#define ALIGN_UP16(num) ALIGN_UP(num, 16)
#define ALIGN_UP128(num) ALIGN_UP(num, 128)

typedef struct PicDesc {
    std::string picName;
    int width;
    int height;
}PicDesc;

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
}Result;


