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

#ifndef YOLOV3_COCO_DETECTION_DYNAMIC_AIPP_INC_DVPP_PROCESS_H
#define YOLOV3_COCO_DETECTION_DYNAMIC_AIPP_INC_DVPP_PROCESS_H

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
    Result CvtJpegToYuv420sp(ImageData& src, ImageData& dest);
    Result InitResource(aclrtStream& stream);
    void DestroyResource();

protected:
    aclrtStream g_stream_;
    acldvppChannelDesc *g_dvppChannelDesc_;
};

#endif