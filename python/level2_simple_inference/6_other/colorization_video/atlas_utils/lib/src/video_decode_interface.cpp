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
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>

#include "atlas_utils.h"
#include "channel_id_generator.h"
#include "video_decode.h"
#include "video_decode_interface.h"

using namespace std;

namespace {
ChannelIdGenerator chanIdGenerator;
VideoDecode* g_VideoDecoderList[VIDEO_CHANNEL_MAX] = {NULL, };
}

extern "C" {

int open_video(const char* videoName) {
    int channelId = chanIdGenerator.generate_channel_id();
    if (channelId == INVALID_CHANNEL_ID) {
        ASC_LOG_ERROR("Create video decoder failed for "
                      "video number excessive limit");
        return INVALID_CHANNEL_ID;
    }    
    ASC_LOG_INFO("openvideo channel id %d", channelId);

    aclrtContext aclCtx;
    aclError ret = aclrtGetCurrentContext(&aclCtx);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("get current context failed");
        return -1;
    }

    VideoDecode* decoder = new VideoDecode(videoName, channelId, aclCtx);
    g_VideoDecoderList[channelId] = decoder;

    return channelId;
}

bool IsOpened(int channelId) {
    if ((channelId < 0) || (channelId >= VIDEO_CHANNEL_MAX)) {
        ASC_LOG_ERROR("Channel id %d invalid", channelId);
        return false;
    }

    return g_VideoDecoderList[channelId]->IsOpened();
}

int ReadDecodedFrame(int channelId, ImageData& frame) {
    if ((channelId < 0) || (channelId >= VIDEO_CHANNEL_MAX)) {
        ASC_LOG_ERROR("Read frame error for channel id %d invalid", channelId);
        return READ_ERROR;
    }

    VideoDecode* decoder = g_VideoDecoderList[channelId];
    if (decoder->IsFinished()) {
        ASC_LOG_ERROR("Read (channel id %d) frame failed for decoded stoped", channelId);
        return READ_FINISHED;
    }

    shared_ptr<ImageData> decodedFrame = decoder->Read();    
    if (decodedFrame == NULL) {
        ASC_LOG_ERROR("Read (channel id %d) frame failed for no decoded frame", channelId);
        return READ_NOFRAME;    
    }
    
    frame.format = decodedFrame->format;
    frame.width = decodedFrame->width;
    frame.height = decodedFrame->height;
    frame.alignWidth = decodedFrame->alignWidth;
    frame.alignHeight = decodedFrame->alignHeight;
    frame.size = decodedFrame->size;
    frame.data = decodedFrame->data;

    decodedFrame->data = nullptr;

    return SUCCESS;
}

int CloseVideo(int channelId) {
    if ((channelId < 0) || (channelId >= VIDEO_CHANNEL_MAX)) {
        ASC_LOG_ERROR("Close video failed for channel id %d invalid", channelId);
        return FAILED;
    }

    chanIdGenerator.release_channle_id(channelId);
    delete g_VideoDecoderList[channelId];
    g_VideoDecoderList[channelId] = NULL;

    return SUCCESS;
}

}