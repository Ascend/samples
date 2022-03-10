/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File AclLiteImageProc.h
* Description: handle dvpp process
*/
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
    uint32_t lt_horz, uint32_t lt_vert,uint32_t rb_horz, uint32_t rb_vert);

    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    CropAndPasteHelper(aclrtStream &stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t width, uint32_t height);

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
    AclLiteError ProcessResolution(ImageData& cropedImage, ImageData& srcImage);

private:
    AclLiteError InitCropAndPasteResource(ImageData& inputImage);
    AclLiteError InitCropAndPasteResourceResolution(ImageData& inputImage);
    AclLiteError InitCropAndPasteInputDesc(ImageData& inputImage);
    AclLiteError InitCropAndPasteInputDescResolution(ImageData& inputImage);
    AclLiteError InitCropAndPasteOutputDesc();

    void DestroyCropAndPasteResource();

private:
    aclrtStream stream_;
    void *vpcOutBufferDev_;
    acldvppRoiConfig *cropArea_;
    acldvppRoiConfig *pasteArea_;
    acldvppPicDesc *vpcInputDesc_;
    acldvppPicDesc *vpcOutputDesc_;
    acldvppChannelDesc *dvppChannelDesc_;

    uint32_t ltHorz_;
    uint32_t rbHorz_;
    uint32_t ltVert_;
    uint32_t rbVert_;
    uint32_t vpcOutBufferSize_;
    uint32_t originalImageWidth_ = 0;
    uint32_t originalImageHeight_ = 0;
    Resolution size_;
};