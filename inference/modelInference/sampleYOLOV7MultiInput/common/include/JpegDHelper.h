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
* Description: handle dvpp process
*/
#ifndef JPEGD_HELPER_H
#define JPEGD_HELPER_H
#pragma once
#include <cstdint>
#include <string.h>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "AclLiteUtils.h"

class JpegDHelper {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    JpegDHelper(aclrtStream &stream,  acldvppChannelDesc *dvppChannelDesc);

    /**
    * @brief Destructor
    */
    ~JpegDHelper();

    /**
    * @brief dvpp global init
    * @return result
    */
    AclLiteError InitResource();
    AclLiteError InitDecodeOutputDesc(ImageData& inputImage);
    AclLiteError Process(ImageData& dest, ImageData& src);

private:
    void DestroyDecodeResource();

private:
    aclrtStream stream_;
    void* decodeOutBufferDev_; // decode output buffer
    acldvppPicDesc *decodeOutputDesc_; // decode output desc
    acldvppChannelDesc *dvppChannelDesc_;
    uint32_t decodeOutWidth_;
    uint32_t decodeOutHeight_;
    uint32_t decodeOutWidthStride_;
    uint32_t decodeOutHeightStride_;
    uint32_t decodeOutBufferSize_;
};
#endif