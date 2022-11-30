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
#ifndef RESNET50_IMAGENET_DYNAMIC_DIMS_INC_SAMPLE_PROCESS_H
#define RESNET50_IMAGENET_DYNAMIC_DIMS_INC_SAMPLE_PROCESS_H

#include <iostream>
#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"

class SampleProcess {
public:
    SampleProcess();
    virtual ~SampleProcess();

    Result InitResource();
    Result Process();
    uint32_t ReadOneBatch(std::string inputImageDir, void *&inputBuff, int batchSize);
    uint32_t ReadOneBatchPicHwc(std::string inputImageDir, void *&inputBuff, int batchSize, int resizeWidth, int resizeHeight);

private:
    void DestroyResource();
    bool isDevice_;
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    aclrtRunMode runMode_;
    ModelProcess modelProcess_;
};
#endif
