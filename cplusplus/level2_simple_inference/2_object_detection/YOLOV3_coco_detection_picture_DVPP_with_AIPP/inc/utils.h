/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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

#ifndef YOLOV3_COCO_DETECTION_PICTURE_DVPP_WITH_AIPP_INC_UTILS_H
#define YOLOV3_COCO_DETECTION_PICTURE_DVPP_WITH_AIPP_INC_UTILS_H

#pragma once
#include <string>
#include <iostream>
#include "acl/acl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stderr, "[ERROR]  " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

struct DataInfo {
    void* data;
    size_t size;
};

typedef struct PicDesc {
    std::string picName;
    uint32_t width;
    uint32_t height;
    uint32_t jpegDecodeSize;
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

class RunStatus {
public:
    static void SetDeviceStatus(bool isDevice)
    {
        isDevice_ = isDevice;
    }
    static bool GetDeviceStatus()
    {
        return isDevice_;
    }
private:
    RunStatus() = default;
    ~RunStatus() = default;
    static bool isDevice_;
};

class Utils {
public:
    /**
    * @brief create device buffer of pic
    * @param [in] picDesc: pic desc
    * @param [out] picDevBuffer: device memory of picture
    * @param [out] devPicBufferSize: size of pic
    * @return device buffer of pic
    */
    static Result GetDeviceBufferOfPicture(PicDesc &picDesc, void *&picDevBuffer, uint32_t &devPicBufferSize);

    /**
    * @brief create buffer of file
    * @param [in] fileName: file name
    * @param [out] inputBuff: input data buffer
    * @param [out] fileSize: size of file
    * @return result
    */
    static Result ReadBinFile(const std::string &fileName, void *&inputBuff, uint32_t &fileSize);

    /**
    * @brief Check whether the path is a file.
    * @param [in] fileName: fold to check
    * @return result
    */
    static Result CheckPathIsFile(const std::string &fileName);

    static void* CopyDataToDevice(void* data, uint32_t dataSize, aclrtMemcpyKind policy);

    static void* CopyDataDeviceToDevice(void* deviceData, uint32_t dataSize);

    static void* CopyDataHostToDevice(void* deviceData, uint32_t dataSize);

    static void* CopyDataDeviceToLocal(void* deviceData, uint32_t dataSize);
};

#endif