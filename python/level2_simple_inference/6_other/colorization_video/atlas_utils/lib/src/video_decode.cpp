/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <fstream>
#include <memory>
#include <thread>
#include <cstring>
#include <iostream>

#include "atlas_utils.h"
#include "channel_id_generator.h"
#include "video_decode.h"

using namespace std;

namespace {
    const int64_t kUsec = 1000000;
}

namespace {
    const uint32_t kDecodeFrameQueueSize = 256;
    const int kDecodeQueueOpWait = 10000; //每次等待10毫秒
    const int kFrameEnQueueRetryTimes = 1000;//为了防止丢帧,ffmpeg解码得到的h26x入队最多等待 100秒
    const int kQueueOpRetryTimes = 100;
    const int kWaitFrameInterval = 10;
    const int kInvalidTpye = -1;
}

VideoDecode::VideoDecode(const std::string& videoName, 
                         uint32_t channelId, aclrtContext context)
 : context_(context)
,channelId_(channelId)
,frameId_(0)
,finFrameCnt_(0)
,status_(DECODE_UNINIT)
,streamName_(videoName)
,streamFormat_(0)
,lastDecodeTime_(0)
,fpsInterval_(0)
,frameDecoder_(nullptr)
,dvppVdec_(nullptr)
,frameH26xQueue_(kDecodeFrameQueueSize)
,frameImageQueue_(kDecodeFrameQueueSize){
     Init();
 }


VideoDecode::~VideoDecode() {
    delete frameDecoder_;
    delete dvppVdec_;
}

int VideoDecode::Init() {
    //防止多次初始化
    if (status_ != DECODE_UNINIT) 
        return status_;
    
    //如果有传入acl context参数,设置解码器主线程context
    aclError aclRet = SetAclContext();
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set frame decoder context failed, errorno:%d", aclRet);
        return status_;
    }
    
    //创建视频解码为H26X的解码器实例
    frameDecoder_ = new VideoFrameDecoder(streamName_);
    if (kInvalidTpye == GetVdecType()) {
        ASC_LOG_ERROR("Video %s type is invalid", streamName_.c_str());
        delete frameDecoder_;
        status_ = DECODE_ERROR;
        return status_;
    } 
    //启动使用DVPP VDEC将H26X帧解码为YUV图片的线程.因为dvpp vdec初始化时间较长,
    //而ffmpeg将视频解码成h26x帧的时间很短,如果dvpp vdec没有初始化好,将会有大量h26x帧积压
    if (!StartupVdecProcess()) {
        ASC_LOG_ERROR("Vdec process init failed.");
        status_ = DECODE_ERROR;
        return status_;
    }
    //h26x裸流文件没有帧率参数
    int fps =  GetFps();
    if (fps == 0) {
        fps = 10;
        ASC_LOG_INFO("Video %s fps is 0, change to 10", streamName_.c_str());
    }
    fpsInterval_ = kUsec / GetFps();

    status_ = DECODE_READY;
    ASC_LOG_INFO("Video %s decode init ok", streamName_.c_str());
    
    return status_;
}

int VideoDecode::GetVdecType() {
    //VDEC支持　H265 main level，264 baseline level，main level，high level
    //等４种格式的视频解码.根据ffmpeg的解码确定视频文件属于哪一种.同样的,如果时h26x裸流,
    //ffmpeg是无法解析出格式信息的
    int type = frameDecoder_->GetVideoType();
    int profile = frameDecoder_->GetProfile();
    if (type == AV_CODEC_ID_HEVC) {
        streamFormat_ = H265_MAIN_LEVEL;         
    } else if (type == AV_CODEC_ID_H264) {
        switch(profile) {
            case FF_PROFILE_H264_BASELINE:
                streamFormat_ = H264_BASELINE_LEVEL;
                break;
            case FF_PROFILE_H264_MAIN:
                streamFormat_ = H264_MAIN_LEVEL;
                break;
            case FF_PROFILE_H264_HIGH:
            case FF_PROFILE_H264_HIGH_10:
            case FF_PROFILE_H264_HIGH_10_INTRA:
            case FF_PROFILE_H264_MULTIVIEW_HIGH:
            case FF_PROFILE_H264_HIGH_422:
            case FF_PROFILE_H264_HIGH_422_INTRA:
            case FF_PROFILE_H264_STEREO_HIGH:
            case FF_PROFILE_H264_HIGH_444:
            case FF_PROFILE_H264_HIGH_444_PREDICTIVE:
            case FF_PROFILE_H264_HIGH_444_INTRA:
                streamFormat_ = H264_HIGH_LEVEL;
                break;
            default:
                ASC_LOG_INFO("Not support h264 profile %d, use as mp", profile);
                streamFormat_ = H264_MAIN_LEVEL; 
                break;
        }
    } else {
        streamFormat_ = kInvalidTpye;
        ASC_LOG_ERROR("Not support stream, type %d,  profile %d", type, profile);
    }

    return streamFormat_;
}

bool VideoDecode::StartupVdecProcess() {
    //实例化vdec解码器,将输入的h26x帧解码为图片
    dvppVdec_ = new VdecProcess(channelId_, GetFrameWidth(), 
                               GetFrameHeight(), streamFormat_,
                               VideoDecode::DvppVdecCallback);
    //启动vdec解码线程
    dvppVdecThread_ = thread(&VideoDecode::DvppVdecThreadFunc, this);
    dvppVdecThread_.detach();
    
    return IsVdecProcessReady();
}

//等待dvpp vdec初始化完成
bool VideoDecode::IsVdecProcessReady() {
    int waitCnt = 10;

    while(waitCnt > 0) {        
        if (dvppVdec_->is_init_ok()) {
            ASC_LOG_INFO("Vdec process startup ok");
            return true;
        }
        sleep(1);
        waitCnt--;
    }

    return false;
}

void VideoDecode::DvppVdecThreadFunc(void* decoderSelf) {
    VideoDecode* thisPtr = (VideoDecode*)decoderSelf;
    //设置线程的acl context,解码器主线程,ffmpeg子线程和vdec子线程使用同一context，
    //以使用同一context下的dvpp内存   
    aclError aclRet = thisPtr->SetAclContext();
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set dvpp vdec decoder context failed, errorno:%d",
                      aclRet);
        return;
    }
    //初始化vdec解码器    
    if (SUCCESS != thisPtr->InitDvppVdec()) {
        ASC_LOG_ERROR("dvpp init failed");
        return;    
    }
    //dvpp vdec的初始化是异步的,需要等待
    while(thisPtr->status_ < DECODE_START) {
        usleep(kWaitFrameInterval);
    }
    //将ffmpeg解码器解码的每一帧h26x数据送到vdec解码器解码
    while(thisPtr->status_ < DECODE_DVPP_FINISHED) {
        //如果dvpp vdec解码完最后一帧数据并且处理了eos帧,则当前线程结束
        if (thisPtr->IsVdecExitReady()) {
            thisPtr->set_status(DECODE_DVPP_FINISHED);
            break;
        }
        //从h26x队列获取一帧数据使用dvpp vdec解码
        if (SUCCESS != thisPtr->DecodeH26xFrame()) {
            usleep(kWaitFrameInterval);
            continue;
        }
        usleep(0);
    }
    ASC_LOG_INFO("dvpp vdec thread exit, status %d", thisPtr->status_);
}

Result VideoDecode::DecodeH26xFrame() {
    //从h26x帧队列中读取一帧数据
    shared_ptr<FrameData> frame = FrameH26xOutQueue();
    if (frame == nullptr) {
        return FAILED;
    }

    //使用dvpp vdec解码h26x帧数据 
    return dvppVdec_->Process(frame, (void*)this);
}

shared_ptr<FrameData> VideoDecode::FrameH26xOutQueue() {
    shared_ptr<FrameData> frameData = nullptr;

    for (int count = 0; count < kQueueOpRetryTimes; count++) {
        frameData = frameH26xQueue_.Pop();
        if (frameData != nullptr)
            return frameData;

        usleep(kDecodeQueueOpWait);
    }

    return nullptr;
}

//dvpp vdec　callback函数
void VideoDecode::DvppVdecCallback(acldvppStreamDesc *input, 
                                   acldvppPicDesc *output, void *userData)
{
    VideoDecode* decoder = (VideoDecode*)userData;
    //获取dvpp vdec解码后的yuv图片数据
    shared_ptr<ImageData> image = make_shared<ImageData>();
    image->format = acldvppGetPicDescFormat(output);
    image->width = acldvppGetPicDescWidth(output);
    image->height = acldvppGetPicDescHeight(output);
    image->alignWidth = acldvppGetPicDescWidthStride(output);
    image->alignHeight = acldvppGetPicDescHeightStride(output);
    image->size = acldvppGetPicDescSize(output);
    image->data = (uint8_t *)acldvppGetPicDescData(output);

    //将解码后的图片放入队列等待读取
    decoder->ProcessDecodedImage(image);

    //释放dvpp vdec的输入输出desc.为了减少内存申请和拷贝,输入和输出数据内存都是
    //和上下游操作复用的,并且解码时异步的,内存无法复用, 所以需要对每一帧重新创建输入输出desc
    aclError ret = acldvppDestroyPicDesc(output);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("fail to destroy pic desc, error %d", ret);
    }

    if (input != nullptr) {
        void* inputBuf = acldvppGetStreamDescData(input);
        if (inputBuf != nullptr) {
            acldvppFree(inputBuf);
        }
        
        aclError ret = acldvppDestroyStreamDesc(input);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("fail to destroy input stream desc");
        }
    }
}

void VideoDecode::ProcessDecodedImage(shared_ptr<ImageData> frameData) {
    finFrameCnt_++;
    if ((status_ == DECODE_FFMPEG_FINISHED) && (finFrameCnt_ >= frameId_)) {
        ASC_LOG_INFO("Last frame decoded by dvpp, notify dvpp vdec finish");
        dvppVdec_->notify_finish();
    }
    FrameImageEnQueue(frameData);
}

Result VideoDecode::FrameImageEnQueue(shared_ptr<ImageData> frameData) {
    for (int count = 0; count < kFrameEnQueueRetryTimes; count++) {
        if (frameImageQueue_.Push(frameData)) 
            return SUCCESS;
        usleep(kDecodeQueueOpWait); 
    }
    ASC_LOG_ERROR("Video %s lost decoded image for queue full", 
	           streamName_.c_str());

    return FAILED;
}

//启动ffmpeg解码器,将视频文件解码为h26x帧
void VideoDecode::StartFrameDecoder() {
    if (status_ == DECODE_READY) {

        decodeThread_ = thread(FrameDecodeThreadFunction, (void*)this);
        decodeThread_.detach();

        status_ = DECODE_START;
    }
}

//ffmpeg解码器线程入口
void VideoDecode::FrameDecodeThreadFunction(void* decoderSelf) {
    VideoDecode* thisPtr =  (VideoDecode*)decoderSelf;

    //将ffmpeg解码器线程的acl context设置为跟主线程和vdec解码线程一致
    aclError aclRet = thisPtr->SetAclContext();
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set frame decoder context failed, errorno:%d", aclRet);
        return;
    }
    //调用ffmpeg解码器开始解码视频
    thisPtr->FFmpegDecode();
     
    //ffmpeg解码完视频后vdec解码器发一个结束帧
    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();    
    videoFrame->isFinished = true;
    videoFrame->data = nullptr;
    videoFrame->size = 0;

    thisPtr->FrameH26xEnQueue(videoFrame);
    thisPtr->set_status(DECODE_FFMPEG_FINISHED);
}


//ffmpeg解码回调
void VideoDecode::FrameDecodeCallback(void* decoder, void* frameData, 
                                      int frameSize) {
    if ((frameData == NULL) || (frameSize == 0)) {
        ASC_LOG_ERROR("Frame data is null");
        return;
    }

    //将ffmpeg解码得到的h26x数据拷贝到dvpp内存
    VideoDecode* videoDecoder = (VideoDecode*)decoder;

    void* buffer = copy_data_local_to_device(frameData, frameSize,
                                         videoDecoder->runModel_, MEMORY_DVPP);
                                                
    if (buffer == nullptr) {
        ASC_LOG_ERROR("Copy frame h26x data to dvpp failed");
        return;
    }

    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();
    videoDecoder->frameId_++;
    videoFrame->frameId = videoDecoder->frameId_;
    videoFrame->data = buffer;
    videoFrame->size = frameSize;
    //将h26x数据放入队列等待dvpp vdec读取
    videoDecoder->FrameH26xEnQueue(videoFrame);
    //根据视频帧率等待下一帧解码时间到来
    videoDecoder->SleeptoNextFrameTime();

}

Result VideoDecode::FrameH26xEnQueue(shared_ptr<FrameData> frameData) {
	for (int count = 0; count < kQueueOpRetryTimes; count++) {
        if (frameH26xQueue_.Push(frameData)) 
            return SUCCESS;
        usleep(kDecodeQueueOpWait); 
    }

    ASC_LOG_ERROR("Video %s lost %dth frame h26x data for queue full", 
	              streamName_.c_str(), frameData->frameId);

    return FAILED;
}

void VideoDecode::SleeptoNextFrameTime() {
    //获取当前时间
    timeval tv;
    gettimeofday(&tv, 0);
    int64_t now = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;

    if (lastDecodeTime_ == 0) {
        lastDecodeTime_ = now;
        return;
    }
    //计算到解码一帧后还剩余的时间
    int64_t lastInterval = (now - lastDecodeTime_);
    int64_t sleepTime = (lastInterval < fpsInterval_)?(fpsInterval_-lastInterval):0;
    //耗完一帧解码时间
    usleep(sleepTime);
    //记录下一帧解码的开始时间
    gettimeofday(&tv, 0);
    lastDecodeTime_ = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;

    return;
}

//当前解码器是否准备好
bool VideoDecode::IsOpened() { 
    ASC_LOG_INFO("Video %s decode status %d", streamName_.c_str(), status_);
    return (status_ == DECODE_READY) || (status_ == DECODE_START);
}

//设置RTSP视频流传输方式
Result VideoDecode::Set(StreamProperty key, std::string& value) {
    if (key == VIDEO_TRANSPORT) {
        if (!strcmp(value.c_str(), RTSP_TRANSPORT_UDP) || 
            !strcmp(value.c_str(), RTSP_TRANSPORT_TCP)) {
            frameDecoder_->SetTransport(value);
            return SUCCESS;
        }       
    }
    return FAILED;
}

/*读取一帧解码后的yuv图像*/
shared_ptr<ImageData> VideoDecode::Read() {
    //如果当前解码器异常或者解码结束,则直接返回nullptr
    if ((status_ == DECODE_ERROR) || (status_ == DECODE_FINISHED)) {
        ASC_LOG_ERROR("read exit 1"); 
        return nullptr;
    }
    //如果当前只是准备好,但是还未开始解码.Read的调用触发解码开始
    if (status_ == DECODE_READY) {
        if (dvppVdec_->video_param_check()) {
            ASC_LOG_ERROR("Read frame failed for video param valid");
            return nullptr;
        }
        StartFrameDecoder();
        usleep(kDecodeQueueOpWait);
    }
    //从解码后图片存放队列读取一帧图片
    shared_ptr<ImageData> frame = FrameImageOutQueue();
    if ((status_ == DECODE_DVPP_FINISHED) && (frame == nullptr)) {
        status_ = DECODE_FINISHED;
    }

    return frame;
}

shared_ptr<ImageData> VideoDecode::FrameImageOutQueue() {
    shared_ptr<ImageData> image = nullptr;

    for (int count = 0; count < kQueueOpRetryTimes; count++) {
        image = frameImageQueue_.Pop();
        if (image != nullptr)
            return image;

        usleep(kDecodeQueueOpWait);
    }

    return nullptr;
}

Result VideoDecode::Set(StreamProperty key, int value) {
    Result ret = FAILED;
    //ffmpeg无法从h26x裸流文件中读取帧分辨率和图像格式信息,需要指定
    switch(key) {
        case FRAME_WIDTH:
            ret = dvppVdec_->set_width(value);
            break;
        case FRAME_HEIGHT:
            ret = dvppVdec_->set_hight(value);
            break;
        case IMAGE_ENTYPE:
            ret = dvppVdec_->set_en_type(value);
            break;
        case IMAGE_FORMAT:
            ret = dvppVdec_->set_format(value);
            break;
        default:
            break;
    }

    return ret;
}

aclError VideoDecode::SetAclContext() {
    if (context_ != nullptr) 
        return aclrtSetCurrentContext(context_);
    else
        return ACL_ERROR_NONE;   
}