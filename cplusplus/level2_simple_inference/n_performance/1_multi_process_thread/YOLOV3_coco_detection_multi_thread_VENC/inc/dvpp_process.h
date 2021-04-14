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

    Result InitResource();
    Result VencProcess(cv::Mat& srcImage, FILE *outFileFp);
    void DestroyResource();

    protected:
    //aclrtStream stream_;
    aclvencChannelDesc *vencChannelDesc_;   //指定通道描述信息
    acldvppPicDesc *vpcInputDesc_;          //输入图片描述信息
    aclvencFrameConfig *vencFrameConfig_;   //单帧配置数据
    acldvppStreamDesc *outputStreamDesc;     //VENC编码后的输出帧码流描述信息

    void *codeInputBufferDev_;
    acldvppPixelFormat format_;
    //uint8_t *inDevBuffer_;
    //bool runFlag_;
    int32_t enType_;
    aclrtRunMode runMode;
    uint32_t inputBufferSize;
    pthread_t threadId_;
    //aclvencChannelDesc *vencChannelDesc_;
};

