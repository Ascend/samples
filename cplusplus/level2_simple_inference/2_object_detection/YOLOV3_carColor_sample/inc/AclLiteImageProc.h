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

#ifndef YOLOV3_CARCOLOR_SAMPLE_INC_ACLLITEIMAGEPROC_H
#define YOLOV3_CARCOLOR_SAMPLE_INC_ACLLITEIMAGEPROC_H

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
    AclLiteError Crop(ImageData& dest, ImageData& src,
                      uint32_t ltHorz, uint32_t ltVert,
                      uint32_t rbHorz, uint32_t rbVert);
    AclLiteError CropPaste(ImageData& dest, ImageData& src,
                           uint32_t width, uint32_t height,
                           uint32_t ltHorz, uint32_t ltVert,
                           uint32_t rbHorz, uint32_t rbVert);
    void DestroyResource();

protected:
    bool g_isReleased_;
    aclrtStream g_stream_;
    acldvppChannelDesc *g_dvppChannelDesc_;
    int g_isInitOk_;
};

#endif