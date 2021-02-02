/**
* @file venc_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <cstdint>
#include <iostream>
#include <errno.h>
#include <string.h>

#include "venc_process.h"

using namespace std;
namespace {
    uint32_t kVencQueueSize = 256;
    uint32_t kImageEnQueueRetryTimes = 3;
    uint32_t kEnqueueWait = 10000;
    uint32_t kOutqueueWait = 10000;
    uint32_t kAsyncWait = 10000;
    bool g_RunFlag = true;
}

VencProcess::VencProcess(VencConfig& vencInfo):
vencInfo_(vencInfo),
status_(STATUS_VENC_INIT),
vencProc_(nullptr),
frameImageQueue_(kVencQueueSize){    
}

AtlasError VencProcess::Init() {
    if (status_ != STATUS_VENC_INIT) {
        return ATLAS_ERROR;
    }

    if (vencInfo_.context == nullptr) {
        aclError ret = aclrtGetCurrentContext(&(vencInfo_.context));
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Get current context failed");
            return ATLAS_ERROR_GET_ACL_CONTEXT;
        }
    }
    
    thread asyncVencTh = thread(VencProcess::AsyncVencThreadEntry, (void*)this);
    asyncVencTh.detach();

    while(status_ == STATUS_VENC_INIT) {
        usleep(kAsyncWait);
    }

    return (status_ == STATUS_VENC_WORK)? ATLAS_OK : ATLAS_ERROR_VENC_STATUS;
}

void VencProcess::AsyncVencThreadEntry(void* arg) {
    VencProcess* thisPtr =  (VencProcess*)arg;
    DvppVenc venc(thisPtr->vencInfo_);

    AtlasError ret = venc.Init();
    if (ret != ATLAS_OK) {
        thisPtr->SetStatus(STATUS_VENC_ERROR);
        ATLAS_LOG_ERROR("Dvpp venc init acl resource failed, error %d", ret);
        return;
    }

    thisPtr->SetStatus(STATUS_VENC_WORK);
    while(thisPtr->GetStatus() == STATUS_VENC_WORK) {
        shared_ptr<ImageData> image = thisPtr->GetEncodeImage();        
        if (image == nullptr) {
            usleep(kOutqueueWait);
            continue;
        }

        ret = venc.Process(*image.get());
        if (ret != ATLAS_OK) {
            thisPtr->SetStatus(STATUS_VENC_ERROR);
            ATLAS_LOG_ERROR("Dvpp venc image failed, error %d", ret);
            break;
        }
    }

    venc.Finish();
    thisPtr->SetStatus(STATUS_VENC_EXIT);
}

AtlasError VencProcess::Process(ImageData& image) {
    if (status_ != STATUS_VENC_WORK) {
        ATLAS_LOG_ERROR("The venc(status %d) is not working", status_);
        return ATLAS_ERROR_VENC_STATUS;
    }

    shared_ptr<ImageData> imagePtr = make_shared<ImageData>();    
    imagePtr->format = image.format;
    imagePtr->width = image.width;
    imagePtr->height = image.height;
    imagePtr->alignWidth = image.alignWidth;
    imagePtr->alignHeight = image.alignHeight;
    imagePtr->size = image.size;
    imagePtr->data = image.data;

    for (uint32_t count = 0; count < kImageEnQueueRetryTimes; count++) {
        if (frameImageQueue_.Push(imagePtr)) {
            return ATLAS_OK;
        }
        usleep(kEnqueueWait); 
    }
    ATLAS_LOG_ERROR("Venc(%s) lost image for queue full", vencInfo_.outFile.c_str());

    return ATLAS_ERROR_VENC_QUEUE_FULL;
}

shared_ptr<ImageData> VencProcess::GetEncodeImage() {
    shared_ptr<ImageData> image = frameImageQueue_.Pop();
    return image;
}

DvppVenc::DvppVenc(VencConfig& vencInfo):
vencInfo_(vencInfo), threadId_(0),
vencChannelDesc_(nullptr), vencFrameConfig_(nullptr),
inputPicDesc_(nullptr), vencStream_(nullptr), 
outFp_(nullptr), isFinished_(false){
}

DvppVenc::~DvppVenc(){
    DestroyResource();
}

void DvppVenc::Callback(acldvppPicDesc *input,
                        acldvppStreamDesc *output, void *userData) {
    void* data = acldvppGetStreamDescData(output);
    uint32_t retCode = acldvppGetStreamDescRetCode(output);
    if (retCode == 0) {
        //encode success, then process output pic
        uint32_t size = acldvppGetStreamDescSize(output);

        DvppVenc* venc = (DvppVenc*)userData;  
        AtlasError ret = venc->SaveVencFile(data, size);     
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Save venc file failed, error %d", ret);
        } else {
            ATLAS_LOG_INFO("success to callback, stream size:%u", size);
        }
    } else {
        ATLAS_LOG_ERROR("venc encode frame failed, ret = %u.", retCode);
    }

    acldvppDestroyPicDesc(input);
}

AtlasError DvppVenc::SaveVencFile(void* vencData, uint32_t size) {   
    AtlasError atlRet = ATLAS_OK;

    void* data = vencData;
    if (vencInfo_.runMode == ACL_HOST) {
        data = CopyDataToHost(vencData, size, vencInfo_.runMode, MEMORY_NORMAL);
    }
    
    size_t ret = fwrite(data, 1, size, outFp_);
    if (ret != size) {
        ATLAS_LOG_ERROR("Save venc file %s failed, need write %u bytes, "
                        "but only write %zu bytes, error: %s",
                        vencInfo_.outFile.c_str(), size, ret, strerror(errno));
        atlRet = ATLAS_ERROR_WRITE_FILE;
    } else {
        fflush(outFp_); 
    }

    if (vencInfo_.runMode == ACL_HOST) {
        delete[]((uint8_t *)data);   
    }

    return atlRet;
}

AtlasError DvppVenc::Init() {
       // create process callback thread
    int ret = pthread_create(&threadId_, nullptr, 
                             &DvppVenc::SubscribleThreadFunc, nullptr);                             
    if (ret != 0) {
        ATLAS_LOG_ERROR("Create venc subscrible thread failed, error %d", ret);
        return ATLAS_ERROR_CREATE_THREAD;
    }

    outFp_ = fopen(vencInfo_.outFile.c_str(), "wb+");
    if (outFp_ == nullptr) {
        ATLAS_LOG_ERROR("Open file %s failed, error %s", 
                        vencInfo_.outFile.c_str(), strerror(errno));
        return ATLAS_ERROR_OPEN_FILE;
    } 

    return InitResource();
}

void* DvppVenc::SubscribleThreadFunc(void *arg)
{
     // Notice: create context for this thread
    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, 0);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Create context failed, error %d.", ret);
        return ((void*)(-1));
    }

    while (g_RunFlag) {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
    }

    ret = aclrtDestroyContext(context);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("aclrtDestroyContext failed, ret=%d.", ret);
    }

    return (void*)0;
}

AtlasError DvppVenc::InitResource()
{
    aclError aclRet = aclrtSetCurrentContext(vencInfo_.context);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set context for dvpp venc failed, error %d", aclRet);
        return ATLAS_ERROR_SET_ACL_CONTEXT;
    }

    AtlasError ret = CreateVencChannel();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create venc channel failed, error %d", ret);
        return ret; 
    }

    aclRet = aclrtCreateStream(&vencStream_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Create venc stream failed, error %d", aclRet);
        return ATLAS_ERROR_CREATE_STREAM;
    }

    aclRet = aclrtSubscribeReport(threadId_, vencStream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Venc subscrible report failed, error %d", aclRet);
        return ATLAS_ERROR_SUBSCRIBE_REPORT;
    }      

    ret = CreateFrameConfig();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create venc frame config failed, error %d", ret);
        return ret;
    }

    ATLAS_LOG_INFO("venc init resource success");
    return ATLAS_OK;
}

AtlasError DvppVenc::CreateVencChannel() {
    // create vdec channelDesc
    vencChannelDesc_ = aclvencCreateChannelDesc();
    if (vencChannelDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create venc channel desc failed");
        return ATLAS_ERROR_CREATE_VENC_CHAN_DESC;
    }

    aclvencSetChannelDescThreadId(vencChannelDesc_, threadId_);
    aclvencSetChannelDescCallback(vencChannelDesc_, &DvppVenc::Callback);
    aclvencSetChannelDescEnType(vencChannelDesc_, vencInfo_.enType);
    aclvencSetChannelDescPicFormat(vencChannelDesc_, vencInfo_.format);
    aclvencSetChannelDescPicWidth(vencChannelDesc_, vencInfo_.maxWidth);
    aclvencSetChannelDescPicHeight(vencChannelDesc_, vencInfo_.maxHeight);
    aclvencSetChannelDescKeyFrameInterval(vencChannelDesc_, 16);

    aclError ret = aclvencCreateChannel(vencChannelDesc_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("fail to create venc channel");
        return ATLAS_ERROR_CREATE_VENC_CHAN;
    }
    return ATLAS_OK;
}

AtlasError DvppVenc::CreateFrameConfig() {
    vencFrameConfig_ = aclvencCreateFrameConfig();
    if (vencFrameConfig_ == nullptr) {
        ATLAS_LOG_ERROR("Create frame config");
        return ATLAS_ERROR_VENC_CREATE_FRAME_CONFIG;
    }

    AtlasError ret = SetFrameConfig(0, 1);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Set frame config failed, error %d", ret);
        return ret;
    }

    return ATLAS_OK;
}

AtlasError DvppVenc::SetFrameConfig(uint8_t eos, uint8_t forceIFrame)
{
    // set eos
    aclError ret = aclvencSetFrameConfigEos(vencFrameConfig_, eos);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("fail to set eos, ret = %d", ret);
        return ATLAS_ERROR_VENC_SET_EOS;
    }

    ret = aclvencSetFrameConfigForceIFrame(vencFrameConfig_, forceIFrame);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("fail to set venc ForceIFrame");
        return ATLAS_ERROR_VENC_SET_IF_FRAME;
    }

    return ATLAS_OK;
}

AtlasError DvppVenc::Process(ImageData& image)
{
    // create picture desc
    AtlasError ret = CreateInputPicDesc(image);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("fail to create picture description");
        return ret;
    }

    // send frame
    acldvppStreamDesc *outputStreamDesc = nullptr;

    ret = aclvencSendFrame(vencChannelDesc_, inputPicDesc_,
        static_cast<void *>(outputStreamDesc), vencFrameConfig_, (void *)this);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("send venc frame failed, error %d", ret);
        return ATLAS_ERROR_VENC_SEND_FRAME;
    }

    return ATLAS_OK;
}

AtlasError DvppVenc::CreateInputPicDesc(ImageData& image)
{
    inputPicDesc_ = acldvppCreatePicDesc();
    if (inputPicDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create input pic desc failed");
        return ATLAS_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescFormat(inputPicDesc_, vencInfo_.format);
    acldvppSetPicDescWidth(inputPicDesc_, image.width);
    acldvppSetPicDescHeight(inputPicDesc_, image.height);
    acldvppSetPicDescWidthStride(inputPicDesc_, ALIGN_UP16(image.width));
    acldvppSetPicDescHeightStride(inputPicDesc_, ALIGN_UP2(image.height));  
    acldvppSetPicDescData(inputPicDesc_, image.data.get());
    acldvppSetPicDescSize(inputPicDesc_, image.size);

    return ATLAS_OK;
}

void DvppVenc::Finish() {
    if (isFinished_) {
        return;
    }

    // set frame config, eos frame
    AtlasError ret = SetFrameConfig(1, 0);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Set eos frame config failed, error %d", ret);
        return;
    }

    // send eos frame
    ret = aclvencSendFrame(vencChannelDesc_, nullptr,
                           nullptr, vencFrameConfig_, nullptr);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("fail to send eos frame, ret=%u", ret);
        return;
    }

    fclose(outFp_);
    outFp_ = nullptr;
    isFinished_ = true;
    ATLAS_LOG_INFO("venc process success");

    return;
}

void DvppVenc::DestroyResource()
{
    Finish();

    if (vencChannelDesc_ != nullptr) {
        (void)aclvencDestroyChannel(vencChannelDesc_);
        (void)aclvencDestroyChannelDesc(vencChannelDesc_);
        vencChannelDesc_ = nullptr;
    }

    if (inputPicDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(inputPicDesc_);
        inputPicDesc_ = nullptr;
    }

    if (vencStream_ != nullptr) {
        aclError ret = aclrtDestroyStream(vencStream_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Vdec destroy stream failed, error %d", ret);
        }
        vencStream_ = nullptr;
    }

    if (vencFrameConfig_ != nullptr) {
        (void)aclvencDestroyFrameConfig(vencFrameConfig_);
        vencFrameConfig_ = nullptr;
    }
}
