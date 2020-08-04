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
#include "dvpp_process.h"

/**
 * DvppProcess
 */
class DvppJpegE{
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    DvppJpegE(aclrtStream &stream, acldvppChannelDesc* dvppChannelDesc);

    /**
    * @brief Destructor
    */
    ~DvppJpegE();

    /**
    * @brief process encode
    * @return result
    */
    Result Process(ImageData& destJpegImage, ImageData& srcYuvImage);

   /**
    * @brief release encode resource
    */
    void DestroyEncodeResource();

private:
    Result InitJpegEResource(ImageData& inputImage);
    Result InitEncodeInputDesc(ImageData& inputImage);
    void DestroyResource();
    void DestroyOutputPara();

    aclrtStream stream_;
    acldvppChannelDesc* dvppChannelDesc_;

    acldvppJpegeConfig* jpegeConfig_;

    uint32_t encodeOutBufferSize_;
    void* encodeOutBufferDev_; // encode output buffer
    acldvppPicDesc* encodeInputDesc_; //encode input desc
};

