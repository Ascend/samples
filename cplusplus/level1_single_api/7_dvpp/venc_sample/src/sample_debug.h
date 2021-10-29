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


#ifndef ____SAMPLE_DEBUG_____
#define ____SAMPLE_DEBUG_____

#include "sample_api.h"
#include "sample_comm.h"

#ifdef HMEV_PLATFORM_SDK

typedef struct {
    char* pcData;
    uint64_t dataLen;
} VencOutStream;

extern void venc_stream_out(uint32_t channelId, void* buffer);
extern int32_t show_encode_state();
extern void venc_wait_finish(IHWCODEC_HANDLE* encHandle, VencSendParams* vencThreadParam);
extern int32_t start_encode();
hi_s32 venc_save_stream(FILE* fp, VencOutStream* outStream, uint64_t packCount);
int32_t venc_read_yuv_file(FILE* fpYuv, hi_video_frame_info* videoFrameInfo, hi_s64* fileOffset);
int32_t venc_do_init(IHWCODEC_HANDLE* encHandle, VencParam* encParam);

#endif

#endif
