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

#ifndef YOLOV3_COCO_DETECTION_PICTURE_WITH_FREETYPE_INC_DVPP_CROPANDPASTE_H
#define YOLOV3_COCO_DETECTION_PICTURE_WITH_FREETYPE_INC_DVPP_CROPANDPASTE_H

#pragma once
#include <cstdint>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "utils.h"

class DvppCropAndPaste {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    DvppCropAndPaste(aclrtStream &stream, acldvppChannelDesc *dvppChannelDesc,
               uint32_t width, uint32_t height);

    /**
    * @brief Destructor
    */
    ~DvppCropAndPaste();

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
    * @brief dvpp process
    * @return result
    */
    Result Process(ImageData& resizedImage, ImageData& srcImage);

private:
    Result InitCropAndPasteResource(ImageData& inputImage);
    Result InitCropAndPasteInputDesc(ImageData& inputImage);
    Result InitCropAndPasteOutputDesc();

    void DestroyCropAndPasteResource();

    aclrtStream g_stream_;
    acldvppChannelDesc *g_dvppChannelDesc_;

    // IN/OUT Desc
    acldvppPicDesc *g_vpcInputDesc_;
    acldvppPicDesc *g_vpcOutputDesc_;

    uint32_t g_originalImageWidth_;
    uint32_t g_originalImageHeight_;

    acldvppRoiConfig *g_cropArea_;
    acldvppRoiConfig *g_pasteArea_;

    // output buffer
    void *g_vpcOutBufferDev_;
    uint32_t g_vpcOutBufferSize_;

    // model [W][H]
    Resolution g_size_;
    acldvppPixelFormat g_format_;
};

#endif