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

#ifndef __SAMPLE_ENCODER__
#define __SAMPLE_ENCODER__

#include "sample_api.h"
#include "sample_comm.h"
#include "sample_looper.h"

#ifdef HMEV_PLATFORM_SDK

#define HI_MAXINUM_LIMIT 100000
#define MAX_ALIGN        1024
#define SEG_CMP_LENGTH   256
#define DIV_UP(x, a) (((x) + ((a) - 1)) / a)
#define ALIGN_DOWN(x, a) (((x) / (a)) * (a))

typedef struct SampleCalConfigTmp {
    hi_u32 vbSize;
    hi_u32 headStride;
    hi_u32 headSize;
    hi_u32 headYSize;
    hi_u32 mainStride;
    hi_u32 mainSize;
    hi_u32 mainYSize;
    hi_u32 extStride;
    hi_u32 extYSize;
} SampleCalConfig;

hi_void get_pic_buffer_config(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
    hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align, SampleCalConfig* calConfig);

hi_u32 get_pic_buffer_size(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
    hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align);

class OnEncStreamHandler;

class HmevEncoder {
    friend OnEncStreamHandler;
public:
    HmevEncoder(VencParam* encParam);
    ~HmevEncoder();
    int32_t codec_init();
    int32_t start_receive_frame();
    int32_t stop_receive_frame();
    int32_t process_frame(hi_video_frame_info* buffer);
    int32_t cancel_frame(hi_video_frame_info* buffer);
    hi_venc_stream* find_match_stream_cache(uint32_t packCount);
    void return_stream_cache(hi_venc_stream* vencStream);
    int32_t dequeue_input_buffer(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
        hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align, hi_video_frame_info** inputFrame);
    uint32_t get_channel_id()
    {
        std::lock_guard<std::recursive_mutex> guardLock(m_lock);
        return m_encParam.channelId;
    }

private:
    int32_t on_encode_stream();
    int32_t do_creat_channel();
    int32_t alloc_input_buffer(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
        hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align, hi_video_frame_info* inputFrame);

    mutable std::recursive_mutex m_lock;
    mutable std::condition_variable_any m_cv;
    VencParam m_encParam;
    uint32_t m_sendFrame = 0;
    uint32_t m_getFrame = 0;
    hi_venc_chn_attr m_stChnAttr;
    hi_venc_gop_attr m_stGopAttr;
    CodecState m_codecState = GHOST;
    std::map<hi_video_frame_info*, uint32_t> m_inBufferSizeMap;
    std::map<uint64_t, hi_video_frame_info*> m_ptsMap;
    std::list<hi_video_frame_info*> m_freeInputBufQueue;
    std::list<hi_venc_stream*> m_freeStreamBuffer;
    const uint32_t FREE_FRAME_QUEUE_LEN = 10;
    HmevLooper* m_encLooper{nullptr};
};

class OnEncStreamHandler : public MsgHandler {
public:
    OnEncStreamHandler(HmevEncoder* encoder) : MsgHandler()
    {
        penc = encoder;
    }

    ~OnEncStreamHandler()
    {
    }

    virtual int32_t handle_message()
    {
        CHECK_NULL_PTR(penc);
        return penc->on_encode_stream();
    }

private:
    HmevEncoder* penc;
};

#endif

#endif
