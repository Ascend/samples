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
#ifndef JPEGE_HELPER_H
#define JPEGE_HELPER_H
#pragma once
#include <cstdint>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "AclLiteUtils.h"
#include "AclLiteImageProc.h"

class JpegEHelper {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    JpegEHelper(aclrtStream &stream, acldvppChannelDesc* dvppChannelDesc);

    /**
    * @brief Destructor
    */
    ~JpegEHelper();

    /**
    * @brief process encode
    * @return result
    */
    AclLiteError Process(ImageData& destJpegImage, ImageData& srcYuvImage);

   /**
    * @brief release encode resource
    */
    void DestroyEncodeResource();

private:
    AclLiteError InitJpegEResource(ImageData& inputImage);
    AclLiteError InitEncodeInputDesc(ImageData& inputImage);
    void DestroyResource();
    void DestroyOutputPara();

    aclrtStream stream_;
    void* encodeOutBufferDev_; // encode output buffer
    acldvppJpegeConfig* jpegeConfig_;
    acldvppPicDesc* encodeInputDesc_; // encode input desc
    acldvppChannelDesc* dvppChannelDesc_;
    uint32_t encodeOutBufferSize_;
};
#endif