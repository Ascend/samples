/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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
*/
#ifndef ACLLITE_IMAGE_PROC_H
#define ACLLITE_IMAGE_PROC_H
#pragma once
#include <cstdint>
#include "AclLiteUtils.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

class AclLiteImageProc {
public:
    /**
    * @brief Constructor
    */
    AclLiteImageProc();
    /**
    * @brief Destructor
    */
    ~AclLiteImageProc();

    AclLiteError Init();
    AclLiteError Resize(ImageData& dest, ImageData& src,
                        uint32_t width, uint32_t height);
    AclLiteError JpegD(ImageData& destYuv, ImageData& srcJpeg);
    AclLiteError JpegE(ImageData& destJpeg, ImageData& srcYuv);
    AclLiteError PngD(ImageData& dest, ImageData& src);
    AclLiteError Crop(ImageData& dest, ImageData& src,
                      uint32_t ltHorz, uint32_t ltVert,
                      uint32_t rbHorz, uint32_t rbVert);
    AclLiteError CropPaste(ImageData& dest, ImageData& src,
                           uint32_t width, uint32_t height,
                           uint32_t ltHorz, uint32_t ltVert,
                           uint32_t rbHorz, uint32_t rbVert);
    AclLiteError ProportionPaste(ImageData& dest, ImageData& src,
                                 uint32_t ltHorz, uint32_t ltVert,
                                 uint32_t rbHorz, uint32_t rbVert);
    AclLiteError ProportionPasteCenter(ImageData& dest, ImageData& src,
                                               uint32_t ltHorz, uint32_t ltVert,
                                               uint32_t rbHorz, uint32_t rbVert);
    void DestroyResource();

protected:
    bool isReleased_;
    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;
    int isInitOk_;
};
#endif