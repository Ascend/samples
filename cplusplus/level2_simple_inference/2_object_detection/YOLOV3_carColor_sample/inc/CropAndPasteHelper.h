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

#ifndef YOLOV3_CARCOLOR_SAMPLE_INC_CROPANDPASTEHELPER_H
#define YOLOV3_CARCOLOR_SAMPLE_INC_CROPANDPASTEHELPER_H

#pragma once
#include <cstdint>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

class CropAndPasteHelper {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    CropAndPasteHelper(aclrtStream &stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t lt_horz, uint32_t lt_vert,
                       uint32_t rb_horz, uint32_t rb_vert);

    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    CropAndPasteHelper(aclrtStream &stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t width, uint32_t height,
                       uint32_t lt_horz, uint32_t lt_vert,
                       uint32_t rb_horz, uint32_t rb_vert);

    /**
    * @brief Destructor
    */
    ~CropAndPasteHelper();

    /**
    * @brief dvpp global init
    * @return AclLiteError
    */
    AclLiteError InitResource();

    /**
    * @brief init dvpp output para
    * @param [in] modelInputWidth: model input width
    * @param [in] modelInputHeight: model input height
    * @return AclLiteError
    */
    AclLiteError InitOutputPara(int modelInputWidth, int modelInputHeight);

    /**
    * @brief dvpp process
    * @return AclLiteError
    */
    AclLiteError Process(ImageData& resizedImage, ImageData& srcImage);
    AclLiteError ProcessCropPaste(ImageData& cropedImage, ImageData& srcImage);

private:
    AclLiteError InitCropAndPasteResource(ImageData& inputImage);
    AclLiteError InitCropAndPasteInputDesc(ImageData& inputImage);
    AclLiteError InitCropAndPasteOutputDesc();

    void DestroyCropAndPasteResource();

private:
    aclrtStream g_stream_;
    void *g_vpcOutBufferDev_;
    acldvppRoiConfig *g_cropArea_;
    acldvppRoiConfig *g_pasteArea_;
    acldvppPicDesc *g_vpcInputDesc_;
    acldvppPicDesc *g_vpcOutputDesc_;
    acldvppChannelDesc *g_dvppChannelDesc_;

    uint32_t g_ltHorz_;
    uint32_t g_rbHorz_;
    uint32_t g_ltVert_;
    uint32_t g_rbVert_;
    uint32_t g_vpcOutBufferSize_;
    uint32_t g_originalImageWidth_ = 0;
    uint32_t g_originalImageHeight_ = 0;
    Resolution g_size_;
};

#endif