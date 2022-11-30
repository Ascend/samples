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

#ifndef YOLOV3_COCO_DETECTION_PICTURE_INC_DVPP_RESIZE_H
#define YOLOV3_COCO_DETECTION_PICTURE_INC_DVPP_RESIZE_H

#pragma once
#include <cstdint>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "utils.h"

class DvppResize {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    DvppResize(aclrtStream &stream, acldvppChannelDesc *dvppChannelDesc,
               uint32_t width, uint32_t height);

    /**
    * @brief Destructor
    */
    ~DvppResize();

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
    * @brief gett dvpp output
    * @param [in] outputBuffer: pointer which points to dvpp output buffer
    * @param [out] outputSize: output size
    */
    void GetOutput(void **outputBuffer, int &outputSize);

    /**
    * @brief dvpp process
    * @return result
    */
    Result Process(ImageData& resizedImage, ImageData& srcImage);

private:
    Result InitResizeResource(ImageData& inputImage);
    Result InitResizeInputDesc(ImageData& inputImage);
    Result InitResizeOutputDesc();

    void DestroyResizeResource();
    void DestroyResource();
    void DestroyOutputPara();

    aclrtStream stream_;
    acldvppChannelDesc *g_dvppChannelDesc_;
    acldvppResizeConfig *g_resizeConfig_;

    acldvppPicDesc *g_vpcInputDesc_; // vpc input desc
    acldvppPicDesc *g_vpcOutputDesc_; // vpc output desc

    uint8_t *g_inDevBuffer_;  // input pic dev buffer
    void *g_vpcOutBufferDev_; // vpc output buffer
    uint32_t g_vpcOutBufferSize_;  // vpc output size
    Resolution g_size_;
    acldvppPixelFormat g_format_;
};

#endif