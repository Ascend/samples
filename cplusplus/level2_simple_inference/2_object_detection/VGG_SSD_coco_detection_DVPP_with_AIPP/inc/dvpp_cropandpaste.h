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

* File dvpp_process.h
* Description: handle dvpp process
*/
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

    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;


    // IN/OUT Desc
    acldvppPicDesc *vpcInputDesc_;
    acldvppPicDesc *vpcOutputDesc_;

    uint32_t originalImageWidth_;
    uint32_t originalImageHeight_;

    acldvppRoiConfig *cropArea_;
    acldvppRoiConfig *pasteArea_;

    // output buffer
    void *vpcOutBufferDev_;
    uint32_t vpcOutBufferSize_;

    //model [W][H]
    Resolution size_;
    acldvppPixelFormat format_;
};

