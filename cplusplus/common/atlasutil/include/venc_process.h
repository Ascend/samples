/**
* @file venc_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include <cstdint>
#include <iostream>
#include <thread>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#include "atlas_utils.h"
#include "thread_safe_queue.h"

enum VencStatus {
    STATUS_VENC_INIT = 0,
    STATUS_VENC_WORK,
    STATUS_VENC_FINISH,
    STATUS_VENC_EXIT,
    STATUS_VENC_ERROR,
};

struct VencConfig {
    uint32_t maxWidth = 0;
    uint32_t maxHeight = 0;
    std::string   outFile;
    acldvppPixelFormat format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    acldvppStreamFormat enType = H264_MAIN_LEVEL;
    aclrtContext context = nullptr;
    aclrtRunMode runMode = ACL_HOST;
};

class DvppVenc {
public:
    DvppVenc(VencConfig& vencConfig);
    ~DvppVenc();

    AtlasError Init();
    AtlasError Process(ImageData& image);
    void Finish();

private:
    AtlasError InitResource();
    AtlasError CreateVencChannel();
    AtlasError CreateInputPicDesc(ImageData& image);
    AtlasError CreateFrameConfig();
    AtlasError SetFrameConfig(uint8_t eos, uint8_t forceIFrame);
    AtlasError SaveVencFile(void* vencData, uint32_t size);
    void DestroyResource();

    static void Callback(acldvppPicDesc *input,
                         acldvppStreamDesc *output, void *userData);
    static void* SubscribleThreadFunc(void *arg);
private:
    VencConfig vencInfo_;

    pthread_t threadId_;
    aclvencChannelDesc *vencChannelDesc_;
    aclvencFrameConfig *vencFrameConfig_;
    acldvppPicDesc *inputPicDesc_;
    aclrtStream vencStream_;

    FILE *outFp_;
    bool isFinished_;
};


class VencProcess {
    public:
    VencProcess(VencConfig& vencConfig);
    ~VencProcess();

    AtlasError Init();
    AtlasError Process(ImageData& image);

    void SetStatus(VencStatus status) { status_ = status; }
    VencStatus GetStatus() { return status_; }
    
private:
    static void AsyncVencThreadEntry(void* arg);
    std::shared_ptr<ImageData> GetEncodeImage();

private:
    VencConfig vencInfo_;

    VencStatus status_;
    DvppVenc* vencProc_;
    ThreadSafeQueue<std::shared_ptr<ImageData>> frameImageQueue_;
};

