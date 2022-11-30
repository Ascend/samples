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

#ifndef YOLOV3_CARCOLOR_SAMPLE_INC_RESIZEHELPER_H
#define YOLOV3_CARCOLOR_SAMPLE_INC_RESIZEHELPER_H

#pragma once
#include <cstdint>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "AclLiteUtils.h"

class ResizeHelper {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    ResizeHelper(aclrtStream &stream, acldvppChannelDesc *dvppChannelDesc,
               uint32_t width, uint32_t height);

    /**
    * @brief Destructor
    */
    ~ResizeHelper();

    /**
    * @brief dvpp global init
    * @return result
    */
    AclLiteError InitResource();

    /**
    * @brief init dvpp output para
    * @param [in] modelInputWidth: model input width
    * @param [in] modelInputHeight: model input height
    * @return result
    */
    AclLiteError InitOutputPara(int modelInputWidth, int modelInputHeight);
   
    /**
    * @brief gett dvpp output
    * @param [in] outputBuffer: pointer which points to dvpp output buffer
    * @param [out] outputSize: output size
    */
    void GetOutput(void **outputBuffer, int &outputSize);

    /**
    * @brief dvpp process
    * @return result
    */
    AclLiteError Process(ImageData& resizedImage, ImageData& srcImage);

private:
    AclLiteError InitResizeResource(ImageData& inputImage);
    AclLiteError InitResizeInputDesc(ImageData& inputImage);
    AclLiteError InitResizeOutputDesc();

    void DestroyResizeResource();
    void DestroyResource();
    void DestroyOutputPara();

    aclrtStream g_stream_;
    void *g_vpcOutBufferDev_; // vpc output buffer
    acldvppPicDesc *g_vpcInputDesc_; // vpc input desc
    acldvppPicDesc *g_vpcOutputDesc_; // vpc output desc
    acldvppResizeConfig *g_resizeConfig_;
    acldvppChannelDesc *g_dvppChannelDesc_;

    uint8_t *g_inDevBuffer_;  // input pic dev buffer
    uint32_t g_vpcOutBufferSize_;  // vpc output size
    Resolution g_size_;
};

#endif