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
#include <string>
#include <iostream>
#include "acl/acl.h"
#include <vector>
#include <memory>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stdout, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define YUV420SP_SIZE(width, height) ((width) * (height) * 3 / 2)
#define SHARED_PTR_DVPP_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { acldvppFree(p); }))
typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

struct DataInfo {
    void* data;
    size_t size;
};

typedef struct PicDesc {
    uint32_t width;
    uint32_t height;
    uint32_t jpegDecodeSize;
    std::shared_ptr<uint8_t> data = nullptr;
    cv::Mat origImage;
    bool isLastFrame;
} PicDesc;

struct Rect {
    uint32_t ltX = 0;
    uint32_t ltY = 0;
    uint32_t rbX = 0;
    uint32_t rbY = 0;
};

struct BBox {
    Rect rect;
    uint32_t score;
    std::string text;
};

struct Resolution {
    uint32_t width = 0;
    uint32_t height = 0;
};

class Utils {
public:
    static void* CopyDataToDevice(void* data, uint32_t dataSize, aclrtMemcpyKind policy);

    static void* CopyDataDeviceToDevice(void* deviceData, uint32_t dataSize);

    static void* CopyDataHostToDevice(void* deviceData, uint32_t dataSize);

    static void* CopyDataDeviceToLocal(void* deviceData, uint32_t dataSize);
};

