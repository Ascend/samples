/**
 *  Copyright [2021] Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */

#ifndef SAMPLE_API_H
#define SAMPLE_API_H

#ifdef HMEV_PLATFORM_SDK

#include <stdint.h>
#include <pthread.h>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <time.h>
#include <sys/epoll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/eventfd.h>
#include <map>
#include <list>
#include <memory>
#include <sys/time.h>
#include <vector>
#include <string>
#include <queue>
#include <thread>
#include <future>
#include <functional>
#include <stdexcept>
#include <algorithm>
#include <deque>
#include <sstream>
#include <fstream>

#include "hi_dvpp.h"

#include <stddef.h>
#include "acl.h"
#include "acl_rt.h"

#define HMEV_SUCCESS (0)
#define HMEV_FAILURE (1)

typedef void* IHWCODEC_HANDLE;

typedef struct {
    void (*encDateOutProcess)(uint32_t channelId, void* buffer);
} VencCallback;

enum CodecState {
    GHOST,
    WORKING,
    FADED,
    DIED,
};

typedef enum {
    HMEV_RC_CBR = 0,
    HMEV_RC_VBR,
    HMEV_RC_AVBR,
    HMEV_RC_QVBR,
    HMEV_RC_CVBR,
    HMEV_RC_QPMAP,
    HMEV_RC_FIXQP
} RcMode;

typedef struct {
    uint32_t width;
    uint32_t height;
} FrameSize;

typedef enum {
    VENC_CODEC_TYPE_H264 = 0,
    VENC_CODEC_TYPE_H265,
    VENC_CODEC_TYPE_MAX,
} VencCodecType;

typedef enum {
    MODE_DEVICE,
    MODE_HOST
} RunMode;

typedef struct {
    VencCodecType codecType;
    uint32_t channelId;
    VencCallback encCallback;
    FrameSize frameSize;
    RcMode rcMode;
    uint32_t fixQp;
    uint32_t maxQp; // RW Range:(MinQp, 51];the max QP value
    uint32_t minQp; // RW Range:[0, 51], the min QP value
    uint32_t maxIQp; // RW Range:(MinQp, 51], max qp for i frame
    uint32_t minIQp; // RW Range:[0, 51], min qp for i frame
    uint32_t profile;
    uint32_t bitRate;
    uint32_t frameRate;
    uint32_t frameNum;
    uint32_t inputFrameLen;
    uint32_t frameGop;
    uint32_t highPriority; // high priority encode
} VencParam;

extern int32_t venc_mng_create(IHWCODEC_HANDLE* encHandle, VencParam* encParam);
extern int32_t venc_mng_process_buffer(IHWCODEC_HANDLE encHandle, void* buffer);
extern int32_t venc_mng_delete(IHWCODEC_HANDLE encHandle);

#endif // HMEV_PLATFORM_SDK
#endif // SAMPLE_API_H
