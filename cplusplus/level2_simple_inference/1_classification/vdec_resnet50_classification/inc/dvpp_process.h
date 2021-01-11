/**
* @file dvpp_process.h
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
 * DvppProcess
 */
class DvppProcess {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    explicit DvppProcess(aclrtStream &stream);

    /**
    * @brief Destructor
    */
    virtual ~DvppProcess();

    /**
    * @brief dvpp global init
    * @return result
    */
    Result InitResource();

    /**
    * @brief init dvpp output para
    * @param [in] modelInputWidth: model input width
    * @param [in] modelInputHeight: model input height
    * @return result
    */
    Result InitOutputPara(int modelInputWidth, int modelInputHeight);

    /**
    * @brief set dvpp input
    * @param [in] inputWidth:width of pic
    * @param [in] inputHeight:height of pic
    * @param [in] format:format of pic
    */
    void SetInput(int inputWidth, int inputHeight, acldvppPixelFormat format);

    /**
    * @brief gett dvpp output
    * @param [in] outputBuffer: pointer which points to dvpp output buffer
    * @param [out] outputSize: output size
    */
    void GetOutput(void **outputBuffer, uint32_t &outputSize);

    /**
    * @brief dvpp process
    * @return result
    */
    Result Process(void *buffer,  uint32_t size);

    /**
    * @brief destroy resource
    */
    void DestroyResource();

    void DestroyOutputPara();

private:
    Result InitResizeInputDesc();
    Result InitResizeOutputDesc();
    Result ProcessResize();
    void DestroyResizeResource();


    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;
    acldvppResizeConfig *resizeConfig_;

    acldvppPicDesc *resizeOutputDesc_; // resize output desc
    acldvppPicDesc *resizeInputDesc_; // resize input desc

    void *resizeOutBufferDev_; // resize output buffer
    void *picOutBufferDev_;

    uint32_t resizeInBufferSize_;  // resize input size
    uint32_t resizeOutBufferSize_;  // resize output size
    uint32_t inputWidth_; // input pic width
    uint32_t inputHeight_; // input pic height
    uint32_t modelInputWidth_; // model input width
    uint32_t modelInputHeight_; // model input height
    acldvppPixelFormat format_; // pic format
};

