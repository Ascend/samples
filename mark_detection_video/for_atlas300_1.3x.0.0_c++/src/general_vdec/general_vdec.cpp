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



#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "hiaiengine/ai_memory.h"
#include "hiaiengine/log.h"
#include <sys/stat.h>
#include <unistd.h>
#include <hiaiengine/api.h>
#include "dvpp/idvppapi.h"
#include "dvpp/Vpc.h"
#include "tool_api.h"
#include "general_vdec.h"


using Stat = struct stat;

const static int NUM_TWO = 2;
const static int NUM_THREE = 3;
const static int BIT_DEPTH8 = 8;

const static int H264 = 0;
const static int H265 = 1;




HIAI_StatusT GeneralVdec::Init(const hiai::AIConfig& config,
	const std::vector<hiai::AIModelDescription>& model_desc)
{
	HIAI_ENGINE_LOG("[VDecEngine] init start.");
	if (pVdecHandle == NULL) {
		int ret = CreateVdecApi(pVdecHandle, 0);
		if (ret != 0 || pVdecHandle == NULL) {
			HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "pVdecHandle is null.");
			return HIAI_ERROR;
		}
	}
	if (pDvppHandle == NULL) {
		int ret = CreateDvppApi(pDvppHandle);
		if (ret != 0 || pDvppHandle == NULL) {
			HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "pDvppHandle is null.");
		}
	}
	vdecInMsg.call_back = GeneralVdec::FrameCallback;
	HIAI_ENGINE_LOG("[VDecEngine] init is finished.");
	return HIAI_OK;
}

GeneralVdec::~GeneralVdec()
{
	if (pVdecHandle != NULL) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "destroy dvpp api!");
		DestroyVdecApi(pVdecHandle, 0);
		pVdecHandle = NULL;
	}
	if (pDvppHandle != NULL) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "destroy dvpp api!");
		DestroyDvppApi(pDvppHandle);
		pDvppHandle = NULL;
	}
}

HIAI_StatusT GeneralVdec::Hfbc2YuvNew(FRAME* frame, uint8_t* outputBuffer)
{
	if (pDvppHandle == NULL) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "[VDecEngine::Hfbc2YuvNew] pDvppHandle is NULL\n");
		return HIAI_ERROR;
	}
	INFO_LOG("frame w %d, h %d\n", frame->width, frame->height);
	std::shared_ptr<VpcUserImageConfigure> userImage(new VpcUserImageConfigure);
	userImage->bareDataAddr = nullptr;
	userImage->bareDataBufferSize = 0;
	userImage->widthStride = frame->width;
	userImage->heightStride = frame->height;
	string imageFormat(frame->image_format);
	if (frame->bitdepth == BIT_DEPTH8) {
		if (imageFormat == "nv12") {
			userImage->inputFormat = INPUT_YUV420_SEMI_PLANNER_UV;
		}
		else {
			userImage->inputFormat = INPUT_YUV420_SEMI_PLANNER_VU;
		}
	}
	else {
		if (imageFormat == "nv12") {
			userImage->inputFormat = INPUT_YUV420_SEMI_PLANNER_UV_10BIT;
		}
		else {
			userImage->inputFormat = INPUT_YUV420_SEMI_PLANNER_VU_10BIT;
		}
	}
	userImage->outputFormat = OUTPUT_YUV420SP_UV;
	userImage->isCompressData = true;
	VpcCompressDataConfigure* compressDataConfigure = &userImage->compressDataConfigure;
	uintptr_t baseAddr = (uintptr_t)frame->buffer;
	compressDataConfigure->lumaHeadAddr = baseAddr + frame->offset_head_y;
	compressDataConfigure->chromaHeadAddr = baseAddr + frame->offset_head_c;
	compressDataConfigure->lumaPayloadAddr = baseAddr + frame->offset_payload_y;
	compressDataConfigure->chromaPayloadAddr = baseAddr + frame->offset_payload_c;
	compressDataConfigure->lumaHeadStride = frame->stride_head;
	compressDataConfigure->chromaHeadStride = frame->stride_head;
	compressDataConfigure->lumaPayloadStride = frame->stride_payload;
	compressDataConfigure->chromaPayloadStride = frame->stride_payload;
	userImage->yuvSumEnable = false;
	userImage->cmdListBufferAddr = nullptr;
	userImage->cmdListBufferSize = 0;
	std::shared_ptr<VpcUserRoiConfigure> roiConfigure(new VpcUserRoiConfigure);
	roiConfigure->next = nullptr;
	userImage->roiConfigure = roiConfigure.get();
	VpcUserRoiInputConfigure* roiInput = &roiConfigure->inputConfigure;
	roiInput->cropArea.leftOffset = 0;
	roiInput->cropArea.rightOffset = (frame->width % NUM_TWO) ? frame->width : (frame->width - 1);
	roiInput->cropArea.upOffset = 0;
	roiInput->cropArea.downOffset = (frame->height % NUM_TWO) ? frame->height : (frame->height - 1);
	VpcUserRoiOutputConfigure* roiOutput = &roiConfigure->outputConfigure;
	roiOutput->outputArea.leftOffset = 0;
	roiOutput->outputArea.rightOffset = (frame->width % NUM_TWO) ? frame->width : (frame->width - 1);
	roiOutput->outputArea.upOffset = 0;
	roiOutput->outputArea.downOffset = (frame->height % NUM_TWO) ? frame->height : (frame->height - 1);
	roiOutput->widthStride = ALIGN_UP(frame->width, DVPP_STRIDE_WIDTH);
	roiOutput->heightStride = ALIGN_UP(frame->height, DVPP_STRIDE_HEIGHT);
	roiOutput->bufferSize = roiOutput->widthStride * roiOutput->heightStride * NUM_THREE / NUM_TWO;
	roiOutput->addr = outputBuffer;
	roiInput->cropArea = roiConfigure->inputConfigure.cropArea;
	dvppapi_ctl_msg dvppApiCtlMsg;
	dvppApiCtlMsg.in = static_cast<void*>(userImage.get());
	dvppApiCtlMsg.in_size = sizeof(VpcUserImageConfigure);
	int ret = DvppCtl(pDvppHandle, DVPP_CTL_VPC_PROC, &dvppApiCtlMsg);
	if (ret != 0) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "[VDecEngine::Hfbc2YuvNew] call dvppctl fail\n");
		hiai::HIAIMemory::HIAI_DVPP_DFree(roiOutput->addr);
		DestroyDvppApi(pDvppHandle);
		return HIAI_ERROR;
	}
	return HIAI_OK;
}

void GeneralVdec::FrameCallback(FRAME* frame, void* hiaiData)
{
	GeneralVdec* vedcPtr = NULL;
	if (hiaiData != NULL) {
		vedcPtr = static_cast<GeneralVdec*>(hiaiData);
	}
	else {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "hiaiData is NULL");
		return;
	}

	uint8_t* outputBuffer = nullptr;
    uint32_t widthStride = ALIGN_UP(frame->width, DVPP_STRIDE_WIDTH);
    uint32_t heightStride = ALIGN_UP(frame->height, DVPP_STRIDE_HEIGHT);
    uint32_t bufferSize = widthStride * heightStride * NUM_THREE / NUM_TWO;

	HIAI_StatusT ret = hiai::HIAIMemory::HIAI_DVPP_DMalloc(bufferSize, (void*&)outputBuffer);

	// call vpc interface to decompress hfbc
	vedcPtr->Hfbc2YuvNew(frame, outputBuffer);
	std::shared_ptr<EngineTrans> image_handle = std::make_shared<EngineTrans>();
	image_handle->image_info.width = frame->realWidth;
	image_handle->image_info.height = frame->realHeight;
	image_handle->image_info.size = bufferSize;
	image_handle->is_finished = false;
	// 指定析构器
	image_handle->image_info.data = std::shared_ptr<uint8_t>(
		     reinterpret_cast<uint8_t*>(outputBuffer), hiai::HIAIMemory::HIAI_DVPP_DFree);

	ret = vedcPtr->SendData(0, "EngineTrans", std::static_pointer_cast<void>(image_handle));
	if (ret != HIAI_OK) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "send_data failed! ret = %d", ret);
		return;
	}
}

HIAI_IMPL_ENGINE_PROCESS("general_vdec", GeneralVdec, INPUT_SIZE)
{
	HIAI_ENGINE_LOG("general_vdec process start!");
	std::shared_ptr<EngineTrans> inputArg = std::static_pointer_cast<EngineTrans>(arg0);
	// if use hiai
	vdecInMsg.hiai_data = this;
	vdecInMsg.channelId = inputArg->channel_id;
	if (inputArg->is_finished) {
		vdecInMsg.isEOS = 1;
	}
	else {
		if (inputArg->format == kH264)
			strncpy_s(vdecInMsg.video_format, sizeof(vdecInMsg.video_format), "h264", 4);
		else
			strncpy_s(vdecInMsg.video_format, sizeof(vdecInMsg.video_format), "h265", 4);

		vdecInMsg.in_buffer_size = inputArg->image_info.size;
		vdecInMsg.in_buffer = reinterpret_cast<char*>(inputArg->image_info.data.get());
		vdecInMsg.isEOS = 0;

		HIAI_ENGINE_LOG("xxxxx image format %d, channel %d, size %d", 
			inputArg->format, vdecInMsg.channelId, vdecInMsg.in_buffer_size);
	}
	dvppapi_ctl_msg MSG;
	MSG.in_size = sizeof(vdec_in_msg);
	MSG.in = reinterpret_cast<void*>(&vdecInMsg);
	int ret = VdecCtl(pVdecHandle, DVPP_CTL_VDEC_PROC, &MSG, 0);
	if (ret != 0) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "[VDecEngine] DVPP_CTL_VDEC_PROC failed! ret = %d", ret);
		return HIAI_ERROR;
	}
	// VdecCtl will be blocked if eos is 1
	// just send end data here
	if (inputArg->is_finished) {
		std::shared_ptr<EngineTrans> eos = std::make_shared<EngineTrans>();
		eos->is_finished = true;
		SendData(0, "EngineTrans", std::static_pointer_cast<void>(eos));
	}
	HIAI_ENGINE_LOG("[general_vdec] general_vdec process end!");
	return HIAI_OK;
}


