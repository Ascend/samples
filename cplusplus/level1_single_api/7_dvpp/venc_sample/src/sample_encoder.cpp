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

#include "sample_encoder.h"

#ifdef HMEV_PLATFORM_SDK

extern std::map<hi_u64, hi_void*> g_mbuf_map;
extern pthread_mutex_t g_mbuf_mutex;
extern int32_t g_one_stream_mode;
extern int32_t g_perf_test;
extern hi_u64 g_start_send_time[MAX_ENC_CHANNEL_NUM];

// calculate the size of different YUV format, resolution, bit width
hi_void get_pic_buffer_config(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
    hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align, SampleCalConfig* calConfig)
{
    hi_u32 bitWidthTmp = 0;
    hi_u32 headStride = 0;
    hi_u32 vbSize = 0;
    hi_u32 headSize = 0;
    hi_u32 alignHeight = 0;
    hi_u32 mainStride = 0;
    hi_u32 mainSize = 0;
    hi_u32 extStride = 0;
    hi_u32 extYSize = 0;
    hi_u32 headYSize = 0;
    hi_u32 ySize = 0;

    if ((width > HI_MAXINUM_LIMIT) || (height > HI_MAXINUM_LIMIT)) {
        calConfig->vbSize = 0;
    }

    // u32Align: 0 is automatic mode, alignment size following system. Non-0 for specified alignment size
    if (align == 0) {
        align = DEFAULT_ALIGN;
    } else if (align > MAX_ALIGN) {
        align = MAX_ALIGN;
    } else {
        align = (ALIGN_UP(align, DEFAULT_ALIGN));
    }

    switch (bitWidth) {
        case HI_DATA_BIT_WIDTH_8: {
            bitWidthTmp = 8; // 8 bitwidth
            break;
        }
        case HI_DATA_BIT_WIDTH_10: {
            bitWidthTmp = 10; // 10 bitwidth
            break;
        }
        case HI_DATA_BIT_WIDTH_12: {
            bitWidthTmp = 12; // 12 bitwidth
            break;
        }
        case HI_DATA_BIT_WIDTH_14: {
            bitWidthTmp = 14; // 14 bitwidth
            break;
        }
        case HI_DATA_BIT_WIDTH_16: {
            bitWidthTmp = 16; // 16 bitwidth
            break;
        }
        default: {
            bitWidthTmp = 0;
            break;
        }
    }

    alignHeight = ALIGN_UP(height, 2); // height align up with 2

    if (cmpMode == HI_COMPRESS_MODE_NONE) {
        mainStride = ALIGN_UP((width * bitWidthTmp + 7) >> 3, align); // +7 >>3 calculate the byte
        if ((pixelFormat >= HI_PIXEL_FORMAT_YUYV_PACKED_422) && (pixelFormat <= HI_PIXEL_FORMAT_VYUY_PACKED_422)) {
            mainStride = ALIGN_UP(((width * bitWidthTmp + 7) >> 3) * 2, align); // +7 >>3 *2 calculate the byte of 422
        }
        ySize = mainStride * alignHeight;

        if ((pixelFormat == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420) ||
            (pixelFormat == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420)) {
            mainSize = (mainStride * alignHeight * 3) >> 1; // 3:calculate the size of 420
        } else if (pixelFormat == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422 ||
                   pixelFormat == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422) {
            mainSize = mainStride * alignHeight * 2; // 2:calculate the size of 422
        } else if (pixelFormat == HI_PIXEL_FORMAT_YUV_400) {
            mainSize = mainStride * alignHeight;
        } else if ((pixelFormat >= HI_PIXEL_FORMAT_YUYV_PACKED_422) &&
                   (pixelFormat <= HI_PIXEL_FORMAT_VYUY_PACKED_422)) {
            mainSize = mainStride * alignHeight;
        } else {
            mainSize = mainStride * alignHeight * 3; // 3:calculate the size of 444
        }
        vbSize = mainSize;
    }

    calConfig->vbSize = vbSize;
    calConfig->headYSize = headYSize;
    calConfig->headSize = headSize;
    calConfig->headStride = headStride;
    calConfig->mainStride = mainStride;
    calConfig->mainYSize = ySize;
    calConfig->mainSize = mainSize;
    calConfig->extStride = extStride;
    calConfig->extYSize = extYSize;

    return;
}

hi_u32 get_pic_buffer_size(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
    hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align)
{
    SampleCalConfig calConfig;
    get_pic_buffer_config(width, height, pixelFormat, bitWidth, cmpMode, align, &calConfig);
    return calConfig.vbSize;
}

HmevEncoder::HmevEncoder(VencParam* encParam)
    : m_encParam(*encParam)
{
}

HmevEncoder::~HmevEncoder()
{
    if (m_codecState != DIED) {
        stop_receive_frame();
    }
}

int32_t HmevEncoder::codec_init()
{
    hi_s32 ret;
    std::lock_guard<std::recursive_mutex> guardLock(m_lock);
    HMEV_HISDK_CHECK_RET(m_codecState != GHOST, HMEV_FAILURE, "encoder has been deleted");

    m_stGopAttr.gop_mode = HI_VENC_GOP_MODE_NORMAL_P;
    m_stGopAttr.normal_p.ip_qp_delta = 3; // 3:value of IPQpDelta

    ret = do_creat_channel();
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "create encode channel fail");
        return HMEV_FAILURE;
    }

    // init input frame buffer
    if (m_encParam.inputFrameLen <= 0) {
        HMEV_HISDK_PRT(ERROR, "input frame len is invalid");
        return HMEV_FAILURE;
    }

    for (uint32_t i = 0; i < m_encParam.inputFrameLen; i++) {
        hi_video_frame_info* temp = new hi_video_frame_info;
        HMEV_HISDK_CHECK_PTR_RET(temp, HMEV_FAILURE);
        memset(temp, 0, sizeof(hi_video_frame_info));
        m_inBufferSizeMap[temp] = 0;
        m_freeInputBufQueue.push_back(temp);
    }

    std::string name = "encoder-listener-" + std::to_string(m_encParam.channelId);
    m_encLooper = new HmevLooper(name.c_str(), 1, 1000); // timeout 1000 ms
    if (m_encLooper->m_thread == 0) {
        HMEV_HISDK_PRT(ERROR, "m_thread create failed");
        return HMEV_FAILURE;
    }
    OnEncStreamHandler* handler = new OnEncStreamHandler(this);
    HMEV_HISDK_CHECK_PTR_RET(handler, HMEV_FAILURE);
    m_encLooper->add_fd(hi_mpi_venc_get_fd(m_encParam.channelId), handler);
    HMEV_HISDK_PRT(DEBUG, "channelId:%u fd:%x",
        m_encParam.channelId, hi_mpi_venc_get_fd(m_encParam.channelId));

    m_codecState = WORKING;
    return HMEV_SUCCESS;
}

// alloc input buffer and set parameter
int32_t HmevEncoder::alloc_input_buffer(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
    hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align, hi_video_frame_info* inputFrame)
{
    int32_t ret;
    SampleCalConfig calConfig;

    CHECK_NULL_PTR(inputFrame);

    std::lock_guard<std::recursive_mutex> guardLock(m_lock);

    auto iter = m_inBufferSizeMap.find(inputFrame);
    if (iter == m_inBufferSizeMap.end()) {
        HMEV_HISDK_PRT(ERROR, "pstInputFrame not belong to this encoder!");
        return HMEV_FAILURE;
    }
    get_pic_buffer_config(width, height, pixelFormat, bitWidth, cmpMode, align, &calConfig);
    if (m_inBufferSizeMap[inputFrame] != 0) {
        HMEV_HISDK_PRT(ERROR, "stInputFrame has been alloc!");
        return HMEV_FAILURE;
    }

    hi_void* inputBuf = NULL;
    ret = hi_mpi_dvpp_malloc(0, &inputBuf, calConfig.vbSize);
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_dvpp_malloc fail, size: %u", calConfig.vbSize);
        return HMEV_FAILURE;
    }
    inputFrame->pool_id = 0;
    inputFrame->mod_id = HI_ID_VENC;
    inputFrame->v_frame.width = width;
    inputFrame->v_frame.height = height;
    inputFrame->v_frame.dynamic_range = HI_DYNAMIC_RANGE_SDR8;
    inputFrame->v_frame.compress_mode = cmpMode;
    inputFrame->v_frame.pixel_format = pixelFormat;
    inputFrame->v_frame.video_format = HI_VIDEO_FORMAT_LINEAR;
    inputFrame->v_frame.field = HI_VIDEO_FIELD_FRAME;
    inputFrame->v_frame.color_gamut = HI_COLOR_GAMUT_BT709;

    inputFrame->v_frame.width_stride[0] = calConfig.mainStride;
    inputFrame->v_frame.width_stride[1] = calConfig.mainStride;

    inputFrame->v_frame.virt_addr[0] = inputBuf;
    inputFrame->v_frame.virt_addr[1] = (hi_void*)((uintptr_t)inputFrame->v_frame.virt_addr[0] + calConfig.mainYSize);

    pthread_mutex_lock(&g_mbuf_mutex);
    g_mbuf_map[(hi_u64)inputBuf] = inputBuf;
    pthread_mutex_unlock(&g_mbuf_mutex);

    m_inBufferSizeMap[inputFrame] = calConfig.vbSize;

    HMEV_HISDK_PRT(DEBUG, "alloc_input_buffer pstInputFrame->v_frame width %u, height %u, width_stride %u",
        inputFrame->v_frame.width, inputFrame->v_frame.height, inputFrame->v_frame.width_stride[0]);
    return HMEV_SUCCESS;
}

// take a node from the free queue to store the input data
int32_t HmevEncoder::dequeue_input_buffer(hi_u32 width, hi_u32 height, hi_pixel_format pixelFormat,
    hi_data_bit_width bitWidth, hi_compress_mode cmpMode, hi_u32 align, hi_video_frame_info** inputFrame)
{
    int32_t ret;
    SampleCalConfig calConfig;
    CHECK_NULL_PTR(inputFrame);

    get_pic_buffer_config(width, height, pixelFormat, bitWidth, cmpMode, align, &calConfig);
    std::lock_guard<std::recursive_mutex> guardLock(m_lock);

    // find the matchable existed buffer
    HMEV_HISDK_PRT(DEBUG, "encoder[%u] dequeue_input_buffer before m_freeInputBufQueue size %zu",
        m_encParam.channelId, m_freeInputBufQueue.size());
    for (auto iter = m_freeInputBufQueue.begin(); iter != m_freeInputBufQueue.end(); ++iter) {
        hi_video_frame &videoFrame = (*iter)->v_frame;
        if (videoFrame.width == width && videoFrame.height == height && videoFrame.compress_mode == cmpMode &&
            videoFrame.pixel_format == pixelFormat && videoFrame.header_stride[0] == calConfig.headStride &&
            videoFrame.width_stride[0] == calConfig.mainStride) {
            *inputFrame = *iter;
            m_freeInputBufQueue.erase(iter);
            HMEV_HISDK_PRT(DEBUG, "dequeue_input_buffer after m_freeInputBufQueue size %zu",
                m_freeInputBufQueue.size());
            return HMEV_SUCCESS;
        }
    }
    if (m_freeInputBufQueue.empty()) {
        HMEV_HISDK_PRT(WARN, "free input buffer is empty");
        return HMEV_FAILURE;
    }

    // release old frame buffer
    hi_video_frame_info* temp = m_freeInputBufQueue.front();

    // realloc new frame buffer
    ret = alloc_input_buffer(width, height, pixelFormat, bitWidth, cmpMode, align, temp);
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "alloc_input_buffer fail");
        return HMEV_FAILURE;
    }
    m_freeInputBufQueue.pop_front();
    *inputFrame = temp;

    HMEV_HISDK_PRT(DEBUG, "dequeue_input_buffer after m_freeInputBufQueue size %zu", m_freeInputBufQueue.size());
    return HMEV_SUCCESS;
}

// set the receive state of encoder and start encoding
int32_t HmevEncoder::start_receive_frame()
{
    hi_s32 ret;
    hi_venc_start_param recvParam;

    std::lock_guard<std::recursive_mutex> guardLock(m_lock);

    HMEV_HISDK_CHECK_RET(m_codecState != WORKING, HMEV_FAILURE, "encoder has been deleted");
    recvParam.recv_pic_num = -1;
    ret = hi_mpi_venc_start_chn(m_encParam.channelId, &recvParam);
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_start_chn faild with%#x! \n", ret);
        return HMEV_FAILURE;
    }

    return HMEV_SUCCESS;
}

// stop the encoding process and destroy the channel
int32_t HmevEncoder::stop_receive_frame()
{
    hi_s32 ret;

    if (m_encLooper != NULL) {
        m_encLooper->remove_fd(hi_mpi_venc_get_fd(m_encParam.channelId));
        m_encLooper->quit();
        delete m_encLooper;
        m_encLooper = NULL;
    }

    std::unique_lock <std::recursive_mutex> ulock(m_lock);
    HMEV_HISDK_CHECK_RET(m_codecState != WORKING, HMEV_FAILURE, "encoder has been deleted");
    m_codecState = FADED;

    while (!m_freeStreamBuffer.empty()) {
        hi_venc_stream *vencStream = m_freeStreamBuffer.front();
        if (vencStream->pack) {
            delete[] vencStream->pack;
            vencStream->pack = NULL;
        }
        delete vencStream;
        m_freeStreamBuffer.pop_front();
    }

    m_inBufferSizeMap.clear();
    while (!m_freeInputBufQueue.empty()) {
        hi_video_frame_info* frameInfo = m_freeInputBufQueue.front();
        m_freeInputBufQueue.pop_front();
        delete frameInfo;
    }

    ret = hi_mpi_venc_stop_chn(m_encParam.channelId);
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_stop_chn vechn[%u] failed with %#x!", m_encParam.channelId, ret);
        return HMEV_FAILURE;
    }
    ret = hi_mpi_venc_destroy_chn(m_encParam.channelId);
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_destroy_chn channel %u fail, error code 0x%x", m_encParam.channelId, ret);
    }
    m_codecState = DIED;
    return HMEV_SUCCESS;
}

// processing function of sending frame
int32_t HmevEncoder::process_frame(hi_video_frame_info* buffer)
{
    hi_s32 ret;
    std::lock_guard<std::recursive_mutex> guardLock(m_lock);
    HMEV_HISDK_CHECK_RET(m_codecState != WORKING, HMEV_FAILURE, "encoder has been deleted");

    ++m_sendFrame;
    hi_u64 sendTimeOut = 0;
    HMEV_GET_TIMESTAMP_US(buffer->v_frame.pts);
    if (m_sendFrame == 1) {
        g_start_send_time[m_encParam.channelId] = buffer->v_frame.pts;
    }
    HMEV_HISDK_PRT(INFO, "channelId:%u m_sendFrame:%u u64SendTime in:%lld",
        m_encParam.channelId, m_sendFrame, buffer->v_frame.pts);

    ret = hi_mpi_venc_send_frame(m_encParam.channelId, buffer, 0);
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_send_frame failed, s32Ret:0x%x\n", ret);
        return HMEV_FAILURE;
    }
    HMEV_GET_TIMESTAMP_US(sendTimeOut);
    HMEV_HISDK_PRT(INFO, "channelId:%u m_sendFrame:%u sendTimeOut:%lld, cost:%lld",
        m_encParam.channelId, m_sendFrame, sendTimeOut, sendTimeOut - buffer->v_frame.pts);

    m_ptsMap[buffer->v_frame.pts] = buffer;
    return HMEV_SUCCESS;
}

// put the node back to the free queue
int32_t HmevEncoder::cancel_frame(hi_video_frame_info* buffer)
{
    std::lock_guard<std::recursive_mutex> guardLock(m_lock);

    HMEV_HISDK_CHECK_RET(m_codecState != WORKING, HMEV_FAILURE, "encoder has been deleted");
    auto iter = m_inBufferSizeMap.find(buffer);
    if (iter == m_inBufferSizeMap.end()) {
        HMEV_HISDK_PRT(ERROR, "pstInputFrame not belong to this encoder!");
        return HMEV_FAILURE;
    }

    for (auto liter = m_freeInputBufQueue.begin(); liter != m_freeInputBufQueue.end(); ++liter) {
        if (*liter == buffer) {
            if (!g_perf_test) {
                HMEV_HISDK_PRT(ERROR, "this buffer is free, cant process");
            }
            return HMEV_FAILURE;
        }
    }

    m_freeInputBufQueue.push_back(buffer);
    return HMEV_SUCCESS;
}

hi_venc_stream *HmevEncoder::find_match_stream_cache(uint32_t packCount)
{
    hi_venc_stream* vencStream = NULL;
    if (!m_freeStreamBuffer.empty()) {
        // first we find the matchable stream object
        for (auto iter = m_freeStreamBuffer.begin(); iter != m_freeStreamBuffer.end(); ++iter) {
            if ((*iter)->pack != NULL && (*iter)->pack_cnt == packCount) {
                vencStream = (*iter);
                m_freeStreamBuffer.erase(iter);
                return vencStream;
            }
        }
        // no matchable stream and size exceed limit, then we free one slot
        if (m_freeStreamBuffer.size() >= FREE_FRAME_QUEUE_LEN) {
            vencStream = m_freeStreamBuffer.front();
            m_freeStreamBuffer.pop_front();
            if (vencStream->pack != NULL) {
                delete[] vencStream->pack;
            }
            delete vencStream;
            vencStream = NULL;
        }
    }

    vencStream = new hi_venc_stream;
    if (vencStream == NULL) {
        return NULL;
    }
    memset(vencStream, 0, sizeof(hi_venc_stream));
    vencStream->pack_cnt = packCount;
    vencStream->pack = new hi_venc_pack[packCount];
    if (vencStream->pack == NULL) {
        delete vencStream;
        return NULL;
    }

    return vencStream;
}

void HmevEncoder::return_stream_cache(hi_venc_stream* vencStream)
{
    hi_venc_pack* packTemp = vencStream->pack;
    hi_u32 packCountTemp = vencStream->pack_cnt;
    memset(packTemp, 0, sizeof(hi_venc_pack) * packCountTemp);
    memset(vencStream, 0, sizeof(hi_venc_stream));
    vencStream->pack = packTemp;
    vencStream->pack_cnt = packCountTemp;
    m_freeStreamBuffer.push_back(vencStream);
}

// encode post-processing, query status, get encoded stream
int32_t HmevEncoder::on_encode_stream()
{
    hi_s32 ret = HMEV_SUCCESS;
    uint64_t lastTime = 0;
    uint64_t curTime = 0;
    uint64_t timeDiff = 0;
    hi_venc_chn_status stStat;
    hi_venc_stream* vencStream = nullptr;

    std::lock_guard<std::recursive_mutex> guardLock(m_lock);

    HMEV_HISDK_CHECK_RET(m_codecState != WORKING, HMEV_FAILURE, "encoder has been deleted");
    ret = hi_mpi_venc_query_status(m_encParam.channelId, &stStat);
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_query_status chn[%u] failed with 0x%x!", m_encParam.channelId, ret);
        return HMEV_FAILURE;
    }

    if (stStat.cur_packs == 0) {
        HMEV_HISDK_PRT(ERROR, "NOTE: Current frame packs is 0");
        return HMEV_FAILURE;
    }

    vencStream = find_match_stream_cache(stStat.cur_packs);

    hi_u64 getTimeIn = 0;
    hi_u64 getTimeOut = 0;

    ++m_getFrame;
    HMEV_GET_TIMESTAMP_US(getTimeIn);
    HMEV_HISDK_PRT(INFO, "channelId:%u m_getFrame:%u u64GetTime in:%lld",
        m_encParam.channelId, m_getFrame, getTimeIn);
    ret = hi_mpi_venc_get_stream(m_encParam.channelId, vencStream, 1000);
    if (ret != HMEV_SUCCESS) {
        m_freeStreamBuffer.push_back(vencStream);
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_get_stream failed with 0x%x!", ret);
        return HMEV_FAILURE;
    }

    HMEV_GET_TIMESTAMP_US(getTimeOut);
    HMEV_HISDK_PRT(INFO, "channelId:%u m_getFrame:%u u64GetTime out:%lld cost:%lld encTime:%u", m_encParam.channelId,
        m_getFrame, getTimeOut, getTimeOut - getTimeIn, (hi_u32)(getTimeOut - vencStream->pack[0].pts));

    HMEV_HISDK_PRT(INFO, "u64InputAddr:%llx", vencStream->pack[0].input_addr);

    auto iter = m_ptsMap.find(vencStream->pack[0].pts);
    HMEV_HISDK_PRT(INFO, "pts: %llx", vencStream->pack[0].pts);
    if (iter != m_ptsMap.end()) {
        cancel_frame(iter->second);
        m_ptsMap.erase(iter);
    }

    CHECK_NULL_PTR(m_encParam.encCallback.encDateOutProcess);
    uint64_t curTimeMs = 0;

    m_encParam.encCallback.encDateOutProcess(m_encParam.channelId, vencStream);

    ret = hi_mpi_venc_release_stream(m_encParam.channelId, vencStream);
    if (ret != HMEV_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_release_stream failed with 0x%x!", ret);
        return HMEV_FAILURE;
    }
    // return cache stream
    return_stream_cache(vencStream);
    return HMEV_SUCCESS;
}

// create encode channel, mode setting
int32_t HmevEncoder::do_creat_channel()
{
    hi_s32 ret = 0;

    if (m_encParam.codecType == VENC_CODEC_TYPE_H264) {
        m_stChnAttr.venc_attr.type = HI_PT_H264;
        m_stChnAttr.venc_attr.profile = m_encParam.profile;
        m_stChnAttr.venc_attr.buf_size = 1024 * 1024 * 4; // stream buffer size 4M:1024 * 1024 * 4
        HMEV_HISDK_PRT(DEBUG, "profile is: %d\n", m_stChnAttr.venc_attr.profile);
    } else if (m_encParam.codecType == VENC_CODEC_TYPE_H265) {
        m_stChnAttr.venc_attr.type = HI_PT_H265;
        m_stChnAttr.venc_attr.buf_size = 1024 * 1024 * 2; // stream buffer size 2M:1024 * 1024 * 2
        m_stChnAttr.venc_attr.profile = m_encParam.profile; // Main Profile
    }

    m_stChnAttr.venc_attr.max_pic_width = m_encParam.frameSize.width;
    m_stChnAttr.venc_attr.max_pic_height = m_encParam.frameSize.height;
    m_stChnAttr.venc_attr.pic_width = m_encParam.frameSize.width;   // the picture width
    m_stChnAttr.venc_attr.pic_height = m_encParam.frameSize.height; // the picture height
    m_stChnAttr.venc_attr.is_by_frame = HI_TRUE;

    if (m_encParam.codecType == VENC_CODEC_TYPE_H264) {
        m_stChnAttr.venc_attr.h264_attr.rcn_ref_share_buf_en = HI_FALSE;
        if (m_encParam.rcMode == HMEV_RC_VBR) {
            m_stChnAttr.rc_attr.rc_mode = HI_VENC_RC_MODE_H264_VBR;
            m_stChnAttr.rc_attr.h264_vbr.gop = m_encParam.frameGop;
            m_stChnAttr.rc_attr.h264_vbr.stats_time = 1;
            m_stChnAttr.rc_attr.h264_vbr.src_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h264_vbr.dst_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h264_vbr.max_bit_rate = m_encParam.bitRate;
        } else {
            m_stChnAttr.rc_attr.rc_mode = HI_VENC_RC_MODE_H264_CBR;
            m_stChnAttr.rc_attr.h264_cbr.gop = m_encParam.frameGop;
            m_stChnAttr.rc_attr.h264_cbr.stats_time = 1;
            m_stChnAttr.rc_attr.h264_cbr.src_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h264_cbr.dst_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h264_cbr.bit_rate = m_encParam.bitRate;
        }
    } else if (m_encParam.codecType == VENC_CODEC_TYPE_H265) {
        m_stChnAttr.venc_attr.h265_attr.rcn_ref_share_buf_en = HI_FALSE;
        if (m_encParam.rcMode == HMEV_RC_VBR) {
            m_stChnAttr.rc_attr.rc_mode = HI_VENC_RC_MODE_H265_VBR;
            m_stChnAttr.rc_attr.h265_vbr.gop = m_encParam.frameGop;
            m_stChnAttr.rc_attr.h265_vbr.stats_time = 1;
            m_stChnAttr.rc_attr.h265_vbr.src_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h265_vbr.dst_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h265_vbr.max_bit_rate = m_encParam.bitRate;
        } else {
            m_stChnAttr.rc_attr.rc_mode = HI_VENC_RC_MODE_H265_CBR;
            m_stChnAttr.rc_attr.h265_cbr.gop = m_encParam.frameGop;
            m_stChnAttr.rc_attr.h265_cbr.stats_time = 1;
            m_stChnAttr.rc_attr.h265_cbr.src_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h265_cbr.dst_frame_rate = m_encParam.frameRate;
            m_stChnAttr.rc_attr.h265_cbr.bit_rate = m_encParam.bitRate;
        }
    }

    m_stChnAttr.gop_attr.gop_mode = m_stGopAttr.gop_mode;
    m_stChnAttr.gop_attr.normal_p.ip_qp_delta = m_stGopAttr.normal_p.ip_qp_delta;

    ret = hi_mpi_venc_create_chn(m_encParam.channelId, &m_stChnAttr);
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_create_chn [%u] faild with 0x%x!\n", m_encParam.channelId, ret);
        return ret;
    }

    // change the scene mode when you need
    hi_venc_scene_mode sceneMode = HI_VENC_SCENE_0;
    ret = hi_mpi_venc_set_scene_mode(m_encParam.channelId, sceneMode);
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_set_scene_mode [%u] failed with 0x%x!\n", m_encParam.channelId, ret);
        return ret;
    }

    hi_venc_rc_param rcParam;
    hi_mpi_venc_get_rc_param(m_encParam.channelId, &rcParam);
    // change the param in rcParam when you need
    ret = hi_mpi_venc_set_rc_param(m_encParam.channelId, &rcParam);
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_set_rc_param [%u] failed with 0x%x!\n", m_encParam.channelId, ret);
        return ret;
    }

    if (m_encParam.highPriority == 1) {
        hi_venc_chn_param chnParam;
        chnParam.priority = 1;

        ret = hi_mpi_venc_set_chn_param(m_encParam.channelId, &chnParam);
        if (ret != HI_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "hi_mpi_venc_set_chn_param [%u] failed\n", m_encParam.channelId);
            return ret;
        }
    }

    return HI_SUCCESS;
}

#endif
