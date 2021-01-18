/**
* @file vdec_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef _VDEC_PROCESS_H_
#define _VDEC_PROCESS_H_

#include <cstdint>
#include <iostream>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

class VdecProcess {
public:
    VdecProcess(int channel, uint32_t width, uint32_t height, 
                int type, aclvdecCallback callback,
                uint32_t outFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    ~VdecProcess();

    static void* SubscribeReportThreadFunc(void *arg);

    AtlasError Init();
    void DestroyResource();
    AtlasError Process(std::shared_ptr<FrameData> frameData, void* userData);
    AtlasError SetFormat(uint32_t format);
    AtlasError VideoParamCheck();
    bool IsExit() { return isExit_; }

private:
    AtlasError CreateVdecChannelDesc();
    AtlasError CreateInputStreamDesc(std::shared_ptr<FrameData> frame);
    AtlasError CreateOutputPicDesc(size_t size);
    void UnsubscribReportThread();

private:
    int channelId_;

    /* 1：YUV420 semi-planner（nv12）
       2：YVU420 semi-planner（nv21）
    */
    uint32_t format_; 

    /* 0：H265 main level
     * 1：H264 baseline level
     * 2：H264 main level
     * 3：H264 high level
     */
    uint32_t enType_;

    uint32_t frameWidth_;
    uint32_t frameHeight_;
    uint32_t alignWidth_;
    uint32_t alignHeight_;
    uint32_t outputPicSize_;
    void *outputPicBuf_;
    aclvdecCallback callback_;
    
    aclrtStream stream_;

    aclvdecChannelDesc *vdecChannelDesc_;
    acldvppStreamDesc *inputStreamDesc_;
    acldvppPicDesc *outputPicDesc_;

    pthread_t subscribeThreadId_;

    bool isExit_;
    bool isReleased_;
    
};

#endif
