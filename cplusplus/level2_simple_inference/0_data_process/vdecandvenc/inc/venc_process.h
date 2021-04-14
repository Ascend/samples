/**
* @file venc_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include <cstdint>
#include <iostream>
#include "utils.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

/**
 * VdecProcess
 */
class VencProcess {
public:
    /**
    * @brief Constructor
    */
    VencProcess();

    /**
    * @brief Destructor
    */
    ~VencProcess();

    /**
    * @brief venc global init
    * @param [in] threadId: index of thread
    * @param [in] format: format of pic
    * @param [in] enType: type of input stream
    * @return result
    */
    Result InitResource(pthread_t threadId, acldvppPixelFormat format, acldvppStreamFormat enType,
    uint32_t width, uint32_t height);

    /**
    * @brief set venc input
    * @param [in] inBufferDev: input buffer
    * @param [in] inBufferSize: buffer size
    */
    void SetInput(void *inBufferDev, uint32_t inBufferSize);

    /**
    * @brief venc process
    * @return result
    */
    Result Process();

    /**
    * @brief destroy venc resource
    */
    void DestroyResource();

private:
    /**
    * @brief create picture description
    */
    Result CreatePicDesc();

    /**
    * @brief set venc frame config
    * @param [in] eos: Whether it is the end frame: 0: no; 1: end frame
    * @param [in] forceIFrame: forced restart of I-frame interval: 0: Not forced; 1: Forced restart of I-frame
    */
    Result SetFrameConfig(uint8_t eos, uint8_t forceIFrame);

    Result SendEosFrame();

    pthread_t threadId_;
    aclvencChannelDesc *vencChannelDesc_;
    aclvencFrameConfig *vencFrameConfig_;
    acldvppPicDesc *inputPicputDesc_;
    void *inBufferDev_;
    uint32_t inBufferSize_;
    acldvppPixelFormat format_;
    acldvppStreamFormat enType_;
};

