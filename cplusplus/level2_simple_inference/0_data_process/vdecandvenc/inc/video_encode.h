/**
* @file sample_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include "utils.h"
#include "acl/acl.h"
#include "atlas_model.h"
#include "venc_process.h"

/**
* VideoEncode
*/
class VideoEncode {
public:
    /**
    * @brief Constructor
    */
    VideoEncode();

    /**
    * @brief Destructor
    */
    ~VideoEncode();

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource(uint32_t width, uint32_t height);

    /**
    * @brief venc process
    * @return result
    */
    Result DoVencProcess(ImageData& image);

    /**
    * @brief destroy resource
    * @return result
    */
    void DestroyResource();

private:

    bool ReadImageToDeviceMem(ImageData& image, void *&dataDev, uint32_t &dataSize);

    pthread_t threadId_;
    char *outFolder_;
    // 1：YUV420 semi-planner（nv12）; 2：YVU420 semi-planner（nv21）
    acldvppPixelFormat format_;
    /* 0：H265 main level
    * 1：H264 baseline level
    * 2：H264 main level
    * 3：H264 high level
    */
    acldvppStreamFormat enType_;
    bool isDivece;
    VencProcess processVenc_;
};

