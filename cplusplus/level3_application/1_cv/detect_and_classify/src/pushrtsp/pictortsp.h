#pragma once
#include "common.h"
#include "AclLiteUtils.h"

class PicToRtsp
{
public:
	PicToRtsp();
	~PicToRtsp();

	int AvInit(int picWidth, int picHeight, std::string g_outFile);
	
	void YuvDataInit();
	void BgrDataInint();

	int YuvDataToRtsp(void *dataBuf, uint32_t size, uint32_t seq);
	int BgrDataToRtsp(void *dataBuf, uint32_t size, uint32_t seq);
	int FlushEncoder();

private:
    AVFormatContext* g_fmtCtx;
    AVCodecContext* g_codecCtx;
    AVStream* g_avStream;
    AVCodec* g_codec;
    AVPacket* g_pkt;
	AVFrame* g_yuvFrame;
	uint8_t* g_yuvBuf;
	AVFrame* g_rgbFrame;
	uint8_t* g_brgBuf;
	int g_yuvSize;
	int g_rgbSize;
	struct SwsContext* g_imgCtx;
	bool g_bgrToRtspFlag;
	bool g_yuvToRtspFlag;
};
