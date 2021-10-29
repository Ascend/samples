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

#include "sample_encoder_manage.h"

#ifdef HMEV_PLATFORM_SDK

std::mutex g_enc_mutex;
EncoderHandle g_encoders[MAX_ENC_CHANNEL_NUM];

int32_t venc_mng_create(IHWCODEC_HANDLE* encHandle, VencParam* encParam)
{
    int32_t ret;
    HMEV_HISDK_CHECK_PTR_RET(encHandle, HMEV_FAILURE);
    HMEV_HISDK_CHECK_PTR_RET(encParam, HMEV_FAILURE);

    if (encParam->channelId >= MAX_ENC_CHANNEL_NUM) {
        HMEV_HISDK_PRT(ERROR, "%d <= channelId(%u)!", MAX_ENC_CHANNEL_NUM, encParam->channelId);
        return HMEV_FAILURE;
    }

    std::lock_guard<std::mutex> guardLock(g_enc_mutex);

    std::shared_ptr <HmevEncoder> enc(new HmevEncoder(encParam));
    HMEV_HISDK_CHECK_PTR_RET(enc, HMEV_FAILURE);

    ret = enc->codec_init();
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "codec_init fail");
        return HMEV_FAILURE;
    }

    ret = enc->start_receive_frame();
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "start_receive_frame fail");
        return HMEV_FAILURE;
    }

    *encHandle = (IHWCODEC_HANDLE)&g_encoders[encParam->channelId];
    g_encoders[encParam->channelId].encoder = enc;

    memcpy(&g_encoders[encParam->channelId].encParam, encParam, sizeof(VencParam));
    return HMEV_SUCCESS;
}

int32_t venc_mng_process_buffer(IHWCODEC_HANDLE encHandle, void* buffer)
{
    int32_t i;
    int32_t ret;
    HMEV_HISDK_CHECK_PTR_RET(encHandle, HMEV_FAILURE);
    HMEV_HISDK_CHECK_PTR_RET(buffer, HMEV_FAILURE);
    EncoderHandle* handle = (EncoderHandle*)encHandle;
    hi_video_frame_info* frameInfo = (hi_video_frame_info*)buffer;

    for (i = 0; i < MAX_ENC_CHANNEL_NUM; ++i) {
        if (handle == &g_encoders[i]) {
            HMEV_HISDK_PRT(DEBUG, "before process_frame");
            std::shared_ptr <HmevEncoder> enc = g_encoders[i].encoder;
            HMEV_HISDK_CHECK_PTR_RET(enc, HMEV_FAILURE);
            ret = enc->process_frame(frameInfo);
            HMEV_HISDK_PRT(DEBUG, "after process_frame");
            return ret;
        }
    }
    return HMEV_SUCCESS;
}

int32_t venc_mng_delete(IHWCODEC_HANDLE encHandle)
{
    uint32_t i;
    HMEV_HISDK_CHECK_PTR_RET(encHandle, HMEV_FAILURE);
    EncoderHandle* handle = (EncoderHandle*)encHandle;

    for (i = 0; i < MAX_ENC_CHANNEL_NUM; ++i) {
        if (handle == &g_encoders[i]) {
            std::shared_ptr <HmevEncoder> enc = handle->encoder;
            HMEV_HISDK_CHECK_PTR_RET(enc, HMEV_FAILURE);
            HMEV_HISDK_PRT(INFO, "delete encoder channel %d", enc->get_channel_id());
            enc->stop_receive_frame();
            handle->encoder = NULL;
            g_encoders[i].encoder = NULL;
            enc = NULL;
            return HMEV_SUCCESS;
        }
    }
    return HMEV_FAILURE;
}

EncoderHandle* venc_mng_get_handle(uint32_t channelId)
{
    if (channelId >= MAX_ENC_CHANNEL_NUM) {
        HMEV_HISDK_PRT(ERROR, "channelId  exceeds");
        return NULL;
    }
    return &g_encoders[channelId];
}

#endif