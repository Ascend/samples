#include "pictortsp.h"
#include "AclLiteApp.h"
using namespace cv;
using namespace std;
namespace {
    const string g_avFormat = "rtsp";
    const uint32_t g_bitRate = 2000000;
    const uint32_t g_gopSize = 12;
    const uint16_t g_frameRate = 15;
}
PicToRtsp::PicToRtsp()
{
    this->g_bgrToRtspFlag = false;
    this->g_yuvToRtspFlag = false;
    this->g_avStream = NULL;
    this->g_codec = NULL;
    this->g_codecCtx = NULL;
    this->g_fmtCtx = NULL;
    this->g_pkt  = NULL;
    this->g_imgCtx = NULL;
    this->g_yuvSize = 0;
	this->g_rgbSize = 0;
}

PicToRtsp::~PicToRtsp()
{
    av_packet_free(&g_pkt);
    avcodec_close(g_codecCtx);
    if (g_fmtCtx) {
        avio_close(g_fmtCtx->pb);
        avformat_free_context(g_fmtCtx);
    }
}

int PicToRtsp::AvInit(int picWidth, int picHeight, std::string g_outFile)
{
    avformat_network_init();
    if (avformat_alloc_output_context2(&g_fmtCtx, NULL, g_avFormat.c_str(), g_outFile.c_str()) < 0) {
        ACLLITE_LOG_ERROR("Cannot alloc output file context");
        return ACLLITE_ERROR;
    }
    av_opt_set(g_fmtCtx->priv_data, "rtsp_transport", "tcp", 0);
    av_opt_set(g_fmtCtx->priv_data, "tune", "zerolatency", 0);
    av_opt_set(g_fmtCtx->priv_data, "preset", "superfast", 0);

    g_codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (g_codec == NULL) {
        ACLLITE_LOG_ERROR("Cannot find any endcoder");
        return ACLLITE_ERROR;
    }

    g_codecCtx = avcodec_alloc_context3(g_codec);
    if (g_codecCtx == NULL) {
        ACLLITE_LOG_ERROR("Cannot alloc context");
        return ACLLITE_ERROR;
    }
    
    g_avStream = avformat_new_stream(g_fmtCtx, g_codec);
    if (g_avStream == NULL) {
        ACLLITE_LOG_ERROR("failed create new video stream");
        return ACLLITE_ERROR;
    }

    g_avStream->time_base = AVRational{1, g_frameRate};

    AVCodecParameters* param = g_fmtCtx->streams[g_avStream->index]->codecpar;
    param->codec_type = AVMEDIA_TYPE_VIDEO;
    param->width = picWidth;
    param->height = picHeight;

    avcodec_parameters_to_context(g_codecCtx, param);

    g_codecCtx->pix_fmt = AV_PIX_FMT_NV12;
    g_codecCtx->time_base = AVRational{1, g_frameRate};
    g_codecCtx->bit_rate = g_bitRate;
    g_codecCtx->gop_size = g_gopSize;
    g_codecCtx->max_b_frames = 0;

    if (g_codecCtx->codec_id == AV_CODEC_ID_H264) {
        g_codecCtx->qmin = 10;
        g_codecCtx->qmax = 51;
        g_codecCtx->qcompress = (float)0.6;
    }

    if (g_codecCtx->codec_id == AV_CODEC_ID_MPEG1VIDEO)
        g_codecCtx->mb_decision = 2;

    if (avcodec_open2(g_codecCtx, g_codec, NULL) < 0) {
        ACLLITE_LOG_ERROR("Open encoder failed");
        return ACLLITE_ERROR;
    }

    avcodec_parameters_from_context(g_avStream->codecpar, g_codecCtx);
    av_dump_format(g_fmtCtx, 0, g_outFile.c_str(), 1);

    int ret = avformat_write_header(g_fmtCtx, NULL);
    if (ret != AVSTREAM_INIT_IN_WRITE_HEADER) {
        ACLLITE_LOG_ERROR("Write file header fail");
        return ACLLITE_ERROR;
    }
    g_pkt = av_packet_alloc();

    return ACLLITE_OK;
}

int PicToRtsp::FlushEncoder()
{
    int ret;
    int vStreamIndex = g_avStream->index;
    AVPacket* pkt = av_packet_alloc();
    pkt->data = NULL;
    pkt->size = 0;

    if (!(g_codecCtx->codec->capabilities & AV_CODEC_CAP_DELAY)) {
        av_packet_free(&pkt);
        return ACLLITE_ERROR;
    }

    ACLLITE_LOG_INFO("Flushing stream %d encoder", vStreamIndex);

    if ((ret = avcodec_send_frame(g_codecCtx, 0)) >= 0) {
        while (avcodec_receive_packet(g_codecCtx, pkt) >= 0) {
            pkt->stream_index = vStreamIndex;
            av_packet_rescale_ts(pkt, g_codecCtx->time_base,
                g_fmtCtx->streams[vStreamIndex]->time_base);
            ret = av_interleaved_write_frame(g_fmtCtx, pkt);
            if (ret < 0) {
                ACLLITE_LOG_ERROR("error is: %d", ret);
                break;
            }
        }
    }
    av_packet_free(&pkt);
    av_write_trailer(g_fmtCtx);

    if (this->g_bgrToRtspFlag == true) {
        av_free(g_brgBuf);
        av_free(g_yuvBuf);
        sws_freeContext(g_imgCtx);
        if (g_rgbFrame)
            av_frame_free(&g_rgbFrame);
        if (g_yuvFrame)
            av_frame_free(&g_yuvFrame);
    }
    if (this->g_yuvToRtspFlag == true) {
        av_free(g_yuvBuf);
        if (g_yuvFrame)
            av_frame_free(&g_yuvFrame);
    }
    return ret;
}

void PicToRtsp::YuvDataInit()
{
    if (this->g_yuvToRtspFlag == false) {
        g_yuvFrame = av_frame_alloc();
        g_yuvFrame->width = g_codecCtx->width;
        g_yuvFrame->height = g_codecCtx->height;
        g_yuvFrame->format = g_codecCtx->pix_fmt;

        g_yuvSize = av_image_get_buffer_size(g_codecCtx->pix_fmt, g_codecCtx->width, g_codecCtx->height, 1);
        
        g_yuvBuf = (uint8_t*)av_malloc(g_yuvSize);

        int ret = av_image_fill_arrays(g_yuvFrame->data, g_yuvFrame->linesize,
        g_yuvBuf, g_codecCtx->pix_fmt,
        g_codecCtx->width, g_codecCtx->height, 1);
        this->g_yuvToRtspFlag = true;
    }
}

int PicToRtsp::YuvDataToRtsp(void *dataBuf, uint32_t size, uint32_t seq)
{
    memcpy(g_yuvBuf, dataBuf, size);
    g_yuvFrame->pts = seq;
    if (avcodec_send_frame(g_codecCtx, g_yuvFrame) >= 0) {
        while (avcodec_receive_packet(g_codecCtx, g_pkt) >= 0) {
            g_pkt->stream_index = g_avStream->index;
            av_packet_rescale_ts(g_pkt, g_codecCtx->time_base, g_avStream->time_base);
            g_pkt->pos = -1;
            int ret = av_interleaved_write_frame(g_fmtCtx, g_pkt);
            if (ret < 0) {
                ACLLITE_LOG_ERROR("error is: %d", ret);
            }
        }
    }
	return ACLLITE_OK;
}

void PicToRtsp::BgrDataInint()
{
    if (this->g_bgrToRtspFlag == false) {
        g_rgbFrame = av_frame_alloc();
        g_yuvFrame = av_frame_alloc();
        g_rgbFrame->width = g_codecCtx->width;
        g_yuvFrame->width = g_codecCtx->width;
        g_rgbFrame->height = g_codecCtx->height;
        g_yuvFrame->height = g_codecCtx->height;
        g_rgbFrame->format = AV_PIX_FMT_BGR24;
        g_yuvFrame->format = g_codecCtx->pix_fmt;

        g_rgbSize = av_image_get_buffer_size(AV_PIX_FMT_BGR24, g_codecCtx->width, g_codecCtx->height, 1);
        g_yuvSize = av_image_get_buffer_size(g_codecCtx->pix_fmt, g_codecCtx->width, g_codecCtx->height, 1);

        g_brgBuf = (uint8_t*)av_malloc(g_rgbSize);
        g_yuvBuf = (uint8_t*)av_malloc(g_yuvSize);

        int ret = av_image_fill_arrays(g_rgbFrame->data, g_rgbFrame->linesize,
            g_brgBuf, AV_PIX_FMT_BGR24,
            g_codecCtx->width, g_codecCtx->height, 1);

        ret = av_image_fill_arrays(g_yuvFrame->data, g_yuvFrame->linesize,
            g_yuvBuf, g_codecCtx->pix_fmt,
            g_codecCtx->width, g_codecCtx->height, 1);
        g_imgCtx = sws_getContext(
            g_codecCtx->width, g_codecCtx->height, AV_PIX_FMT_BGR24,
            g_codecCtx->width, g_codecCtx->height, g_codecCtx->pix_fmt,
            SWS_BILINEAR, NULL, NULL, NULL);
        this->g_bgrToRtspFlag = true;
    }
}

int PicToRtsp::BgrDataToRtsp(void *dataBuf, uint32_t size, uint32_t seq)
{
    if (g_rgbSize != size) {
        ACLLITE_LOG_ERROR("bgr data size error, The data size should be %d, but the actual size is %d", g_rgbSize, size);
        return ACLLITE_ERROR;
    }
    memcpy(g_brgBuf, dataBuf, g_rgbSize);
    sws_scale(g_imgCtx,
        g_rgbFrame->data,
        g_rgbFrame->linesize,
        0,
        g_codecCtx->height,
        g_yuvFrame->data,
        g_yuvFrame->linesize);
    g_yuvFrame->pts = seq;
    if (avcodec_send_frame(g_codecCtx, g_yuvFrame) >= 0) {
        while (avcodec_receive_packet(g_codecCtx, g_pkt) >= 0) {
            g_pkt->stream_index = g_avStream->index;
            av_packet_rescale_ts(g_pkt, g_codecCtx->time_base, g_avStream->time_base);
            g_pkt->pos = -1;
            int ret = av_interleaved_write_frame(g_fmtCtx, g_pkt);
            if (ret < 0) {
                ACLLITE_LOG_ERROR("error is: %d", ret);
            }
        }
    }
	return ACLLITE_OK;
}