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
#include "utils.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

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
    Result InitDvppOutputPara(int modelInputWidth, int modelInputHeight);

    /**
    * @brief gett dvpp output
    * @param [in] outputBuffer: pointer which points to dvpp output buffer
    * @param [out] outputSize: output size
    */
    void GetDvppOutput(void **outputBuffer, int &outputSize);

    /**
    * @brief dvpp process
    * @return result
    */
    Result Process(PicDesc& srcImage);

private:
    Result InitResizeInputDesc(PicDesc& srcImage);
    Result InitResizeOutputDesc();
    Result ProcessResize();
    void DestroyResizeResource();

    void DestroyResource();

    void DestroyOutputPara();

    acldvppChannelDesc *dvppChannelDesc_;
    aclrtStream stream_;
    acldvppResizeConfig *resizeConfig_;

    acldvppPicDesc *resizeInputDesc_; // resize input desc
    acldvppPicDesc *resizeOutputDesc_; // resize output desc

    void *resizeOutBufferDev_; // resize output buffer
    uint32_t resizeOutBufferSize_;  // resize output size

    uint32_t modelInputWidth_; // model input width
    uint32_t modelInputHeight_; // model input height
    uint32_t resizeOutWidthStride_; // resize output width aligned
    uint32_t resizeOutHeightStride_; // resize output height aligned
};

