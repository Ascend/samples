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
    * @brief set dvpp input
    * @param [in] inDevBuffer: device buffer of input pic
    * @param [in] inDevBufferSize: device buffer size of input pic
    * @param [in] picDesc: picture description
    */
    void SetInput(void *inDevBuffer, uint32_t inDevBufferSize, const PicDesc &picDesc);

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
    Result Process();

private:
    Result InitDecodeOutputDesc();
    Result ProcessDecode();
    void DestroyDecodeResource();

    Result InitResizeInputDesc();
    Result InitResizeOutputDesc();
    Result ProcessResize();
    void DestroyResizeResource();

    void DestroyResource();

    void DestroyOutputPara();

    acldvppChannelDesc *dvppChannelDesc_;
    aclrtStream stream_;
    acldvppResizeConfig *resizeConfig_;

    void* decodeOutDevBuffer_; // decode output buffer
    acldvppPicDesc *decodeOutputDesc_; //decode output desc

    acldvppPicDesc *resizeInputDesc_; // resize input desc
    acldvppPicDesc *resizeOutputDesc_; // resize output desc

    void *inDevBuffer_;  // decode input buffer
    uint32_t inDevBufferSize_; // dvpp input buffer size
    uint32_t jpegDecodeOutputSize_; // jpeg decode output size

    uint32_t decodeOutputWidth_; // decode output width
    uint32_t decodeOutputWidthStride_; // decode output width aligned
    uint32_t decodeOutputHeight_; // decode output height

    void *resizeOutBufferDev_; // resize output buffer
    uint32_t resizeOutBufferSize_;  // resize output size

    uint32_t modelInputWidth_; // model input width
    uint32_t modelInputHeight_; // model input height
    uint32_t resizeOutWidthStride_; // resize output width aligned
    uint32_t resizeOutHeightStride_; // resize output height aligned
};

