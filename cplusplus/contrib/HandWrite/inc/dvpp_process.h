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

/**
 * DvppProcess
 */
class DvppProcess {
public:
    DvppProcess();

    ~DvppProcess();

    Result Resize(ImageData& src, ImageData& dest,
                  uint32_t width, uint32_t height);
    Result CvtYuv420spToJpeg(ImageData& dest, ImageData& src);
    Result InitResource(aclrtStream& stream);

protected:
    int isInitOk_;
    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;
    bool isGlobalContext_;

private:
    void DestroyResource();
};

