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

* File main.cpp
* Description: dvpp sample main func
*/
#include <cstdint>
#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include <memory>
#include <vector>
#include <thread>
#include <string>
#include "ThreadSafeQueue.h"
#include "utils.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

class DecodeProcess {
public:
    DecodeProcess(aclrtStream& stream, aclrtRunMode& runMode, std::string streamName, aclrtContext& context);
    ~DecodeProcess();
    Result ReadFrame(PicDesc& image);
    Result InitResource();
    Result DestroyResource();
    int GetFrameWidth() { return frameWidth_; }
    int GetFrameHeight() { return frameHeight_; }
    static void Decode(void* callbackParam);

private:
    Result SetAclContext();
    Result FrameImageEnQueue(std::shared_ptr<PicDesc> frameData);
    int GetVideoIndex(AVFormatContext* avFormatContext);
    void SetDictForRtsp(AVDictionary*& avdic);
    bool OpenVideo(AVFormatContext*& avFormatContext);
    void GetVideoInfo();
    void FFmpegDecoder();
    int GetVdecType();
    Result FrameDecodeCallback(void* userData, void* frameData, int frameSize);
    void InitVideoStreamFilter(const AVBitStreamFilter*& videoFilter);
    bool InitVideoParams(int videoIndex, 
                        AVFormatContext* avFormatContext,
                        AVBSFContext*& bsfCtx);
    Result VdecDecoder();
    std::shared_ptr<PicDesc> FrameImageOutQueue(bool noWait);
    static void callback(acldvppStreamDesc *input, acldvppPicDesc *output, void *userdata);
    static void* ThreadFunc(void *arg);
    void SleeptoNextFrameTime();
private:
    aclrtContext context_;
    aclrtStream stream_;
    pthread_t threadId_;
    int32_t format_; 
    int32_t enType_;
    aclvdecChannelDesc *vdecChannelDesc_;
    acldvppStreamDesc *streamInputDesc_;
    acldvppPicDesc *picOutputDesc_;
    void *picOutBufferDev_;
    void *inBufferDev_;
    int frameWidth_;
    int frameHeight_;
    int videoType_;
    int profile_;
    int fps_;
    int streamFormat_;
    std::string streamName_;  
    std::string rtspTransport_;
    aclrtRunMode runMode_;
    std::thread decodeThread_;
    bool isStop_;
    int fpsInterval_;
    int64_t lastDecodeTime_ ;
    ThreadSafeQueue<std::shared_ptr<PicDesc>> frameImageQueue_;
};