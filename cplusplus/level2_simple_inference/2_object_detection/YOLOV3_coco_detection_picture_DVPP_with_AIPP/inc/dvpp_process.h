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

#ifndef YOLOV3_COCO_DETECTION_PICTURE_DVPP_WITH_AIPP_INC_DVPP_PROCESS_H
#define YOLOV3_COCO_DETECTION_PICTURE_DVPP_WITH_AIPP_INC_DVPP_PROCESS_H

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

    acldvppChannelDesc *g_dvppChannelDesc_;
    aclrtStream g_stream_;
    acldvppResizeConfig *g_resizeConfig_;

    void* g_decodeOutDevBuffer_; // decode output buffer
    acldvppPicDesc *g_decodeOutputDesc_; // decode output desc

    acldvppPicDesc *g_resizeInputDesc_; // resize input desc
    acldvppPicDesc *g_resizeOutputDesc_; // resize output desc

    void *g_inDevBuffer_;  // decode input buffer
    uint32_t g_inDevBufferSize_; // dvpp input buffer size
    uint32_t g_jpegDecodeOutputSize_; // jpeg decode output size

    uint32_t g_decodeOutputWidth_; // decode output width
    uint32_t g_decodeOutputWidthStride_; // decode output width aligned
    uint32_t g_decodeOutputHeight_; // decode output height

    void *g_resizeOutBufferDev_; // resize output buffer
    uint32_t g_resizeOutBufferSize_;  // resize output size

    uint32_t g_modelInputWidth_; // model input width
    uint32_t g_modelInputHeight_; // model input height
    uint32_t g_resizeOutWidthStride_; // resize output width aligned
    uint32_t g_resizeOutHeightStride_; // resize output height aligned
};

#endif