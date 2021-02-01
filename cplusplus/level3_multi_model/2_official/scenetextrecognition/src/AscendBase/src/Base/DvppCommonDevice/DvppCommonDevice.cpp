/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "DvppCommonDevice/DvppCommonDevice.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"

APP_ERROR DvppCommonDevice::init()
{
    iDvppApi_ = nullptr;
    uint32_t ret = CreateDvppApi(iDvppApi_);
    if (ret != APP_ERR_OK) {
        LogError << "create DVPP API fail, ret=" << ret;
        return APP_ERR_COMM_INIT_FAIL;
    }
    return APP_ERR_OK;
}

APP_ERROR DvppCommonDevice::deinit()
{
    if (iDvppApi_ == nullptr) {
        LogWarn << "DVPP objet is null";
        return APP_ERR_COMM_INVALID_POINTER;
    }
    uint32_t ret = DestroyDvppApi(iDvppApi_);
    if (ret != APP_ERR_OK) {
        LogError << "destroy DVPP API fail, ret=" << ret;
        return APP_ERR_COMM_FAILURE;
    }
    return APP_ERR_OK;
}

APP_ERROR DvppCommonDevice::VpcResize(DvppResizeInputMsg input)
{
    if (input.outBuf == nullptr || input.imgBuf == nullptr) {
        LogError << "input buffer or output buffer is null, input.imgBuf=" << (void *)input.imgBuf
                 << ", input.outBuf=" << (void *)input.outBuf;
        return APP_ERR_COMM_INVALID_POINTER;
    }
    uint32_t inWidthStride = DVPP_ALIGN_UP(input.inW, VPC_WIDTH_ALIGN);
    uint32_t inHeightStride = DVPP_ALIGN_UP(input.inH, VPC_HEIGHT_ALIGN);
    SetImageConfigure(imageConfigure_, inWidthStride, inHeightStride, input.imgBuf);
    auto roiConfigure = std::make_shared<struct VpcUserRoiConfigure>();
    roiConfigure->next = nullptr;
    struct VpcUserRoiInputConfigure *inputConfigure = &(roiConfigure->inputConfigure);
    SetCropConfigure(inputConfigure->cropArea, 0, inWidthStride - 1, 0, inHeightStride - 1);

    uint32_t outWidthStride = DVPP_ALIGN_UP(input.outW, VPC_WIDTH_ALIGN);
    uint32_t outHeightStride = DVPP_ALIGN_UP(input.outH, VPC_HEIGHT_ALIGN);
    struct VpcUserRoiOutputConfigure *outputConfigure = &(roiConfigure->outputConfigure);
    SetRoiOutputConfigure(outputConfigure, outWidthStride, outHeightStride, input.outBuf);
    SetCropConfigure(outputConfigure->outputArea, 0, outWidthStride - 1, 0, outHeightStride - 1);

    imageConfigure_.roiConfigure = roiConfigure.get();

    dvppApiCtrlMsg_.in = static_cast<void *>(&imageConfigure_);
    dvppApiCtrlMsg_.in_size = sizeof(VpcUserImageConfigure);
    int ret = DvppCtl(iDvppApi_, DVPP_CTL_VPC_PROC, &dvppApiCtrlMsg_);
    if (ret != APP_ERR_OK) {
        LogError << "resize fail, ret=" << ret;
        return APP_ERR_COMM_FAILURE;
    }
    return APP_ERR_OK;
}

APP_ERROR DvppCommonDevice::VpcResizeWithPadding(DvppResizeInputMsg input)
{
    if (input.outBuf == nullptr || input.imgBuf == nullptr) {
        LogError << "input buffer or output buffer is null, input.imgBuf=" << (void *)input.imgBuf
                 << ", input.outBuf=" << (void *)input.outBuf;
        return APP_ERR_COMM_INVALID_POINTER;
    }
    uint32_t inWidthStride = DVPP_ALIGN_UP(input.inW, VPC_HEIGHT_ALIGN);
    uint32_t inHeightStride = DVPP_ALIGN_UP(input.inH, VPC_HEIGHT_ALIGN);
    uint32_t outWidthStride = DVPP_ALIGN_UP(input.outW, VPC_WIDTH_ALIGN);
    uint32_t outHeightStride = DVPP_ALIGN_UP(input.outH, VPC_HEIGHT_ALIGN);
    SetImageConfigure(imageConfigure_, inWidthStride, inHeightStride, input.imgBuf);
    auto roiConfigure = std::make_shared<struct VpcUserRoiConfigure>();
    roiConfigure->next = nullptr;
    struct VpcUserRoiInputConfigure *inputConfigure = &(roiConfigure->inputConfigure);
    uint32_t upOffset = 0;
    uint32_t downOffset = inHeightStride - 1;
    uint32_t leftOffset = 0;
    uint32_t rightOffset = inWidthStride - 1;
    SetCropConfigure(inputConfigure->cropArea, leftOffset, rightOffset, upOffset, downOffset);

    struct VpcUserRoiOutputConfigure *outputConfigure = &(roiConfigure->outputConfigure);
    SetRoiOutputConfigure(outputConfigure, outWidthStride, outHeightStride, input.outBuf);
    if ((float(inWidthStride) / float(outWidthStride)) >= (float(inHeightStride) / float(outHeightStride))) {
        float scale = ((float)outWidthStride) / ((float)inWidthStride);
        downOffset = DVPP_ALIGN_UP(uint32_t(scale * inHeightStride), VPC_OFFSET_ALIGN) - 1;
        rightOffset = outWidthStride - 1;
    } else {
        float scale = ((float)outHeightStride) / ((float)inHeightStride);
        rightOffset = DVPP_ALIGN_UP(uint32_t(scale * inWidthStride), VPC_OFFSET_ALIGN) - 1;
        downOffset = outHeightStride - 1;
    }
    SetCropConfigure(outputConfigure->outputArea, leftOffset, rightOffset, upOffset, downOffset);

    imageConfigure_.roiConfigure = roiConfigure.get();

    dvppApiCtrlMsg_.in = static_cast<void *>(&imageConfigure_);
    dvppApiCtrlMsg_.in_size = sizeof(VpcUserImageConfigure);
    int ret = DvppCtl(iDvppApi_, DVPP_CTL_VPC_PROC, &dvppApiCtrlMsg_);
    if (ret != APP_ERR_OK) {
        LogError << "resize fail, ret=" << ret;
        return APP_ERR_COMM_FAILURE;
    }
    return APP_ERR_OK;
}

void DvppCommonDevice::SetImageConfigure(VpcUserImageConfigure &imageConfigure, const uint32_t inWidthStride,
    const uint32_t inHeightStride, uint8_t *&inBuffer)
{
    imageConfigure.bareDataAddr = inBuffer;
    imageConfigure.bareDataBufferSize =
        inWidthStride * inHeightStride * YUV_BGR_SIZE_CONVERT_3 / YUV_BGR_SIZE_CONVERT_2;
    imageConfigure.isCompressData = false;
    imageConfigure.widthStride = inWidthStride;
    imageConfigure.heightStride = inHeightStride;
    imageConfigure.inputFormat = INPUT_YUV420_SEMI_PLANNER_UV;
    imageConfigure.outputFormat = OUTPUT_YUV420SP_UV;
    imageConfigure.yuvSumEnable = false;
    imageConfigure.cmdListBufferAddr = nullptr;
    imageConfigure.cmdListBufferSize = 0;

    return;
}

void DvppCommonDevice::SetCropConfigure(VpcUserCropConfigure &cropArea, const uint32_t leftOffset,
    const uint32_t rightOffset, const uint32_t upOffset, const uint32_t downOffset)
{
    cropArea.leftOffset = leftOffset;
    cropArea.rightOffset = rightOffset;
    cropArea.upOffset = upOffset;
    cropArea.downOffset = downOffset;
    return;
}

void DvppCommonDevice::SetRoiOutputConfigure(VpcUserRoiOutputConfigure *&outputConfigure, const uint32_t outWidthStride,
    const uint32_t outHeightStride, uint8_t *&outBuffer)
{
    outputConfigure->addr = outBuffer;
    outputConfigure->bufferSize = outWidthStride * outHeightStride * YUV_BGR_SIZE_CONVERT_3 / YUV_BGR_SIZE_CONVERT_2;
    outputConfigure->widthStride = outWidthStride;
    outputConfigure->heightStride = outHeightStride;
    return;
}

APP_ERROR DvppCommonDevice::VpcCrop(DvppCropInputMsg input)
{
    uint32_t inWidthStride = DVPP_ALIGN_UP(input.inW, VPC_WIDTH_ALIGN);
    uint32_t inHeightStride = DVPP_ALIGN_UP(input.inH, VPC_HEIGHT_ALIGN);
    SetImageConfigure(imageConfigure_, inWidthStride, inHeightStride, input.imgBuf);

    auto roiConfigure = std::make_shared<struct VpcUserRoiConfigure>();
    roiConfigure->next = nullptr;
    struct VpcUserRoiInputConfigure *inputConfigure = &(roiConfigure->inputConfigure);
    uint32_t leftOffset = input.roi.left / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN;
    uint32_t rightOffset = input.roi.right / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN - 1;
    uint32_t upOffset = input.roi.up / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN;
    uint32_t downOffset = input.roi.down / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN - 1;
    SetCropConfigure(inputConfigure->cropArea, leftOffset, rightOffset, upOffset, downOffset);

    uint32_t outWidthStride = DVPP_ALIGN_UP(input.outW, VPC_WIDTH_ALIGN);
    uint32_t outHeightStride = DVPP_ALIGN_UP(input.outH, VPC_HEIGHT_ALIGN);
    struct VpcUserRoiOutputConfigure *outputConfigure = &(roiConfigure->outputConfigure);
    SetRoiOutputConfigure(outputConfigure, outWidthStride, outHeightStride, input.outBuf);
    SetCropConfigure(outputConfigure->outputArea, 0, outWidthStride - 1, 0, outHeightStride - 1);

    imageConfigure_.roiConfigure = roiConfigure.get();

    dvppApiCtrlMsg_.in = static_cast<void *>(&imageConfigure_);
    dvppApiCtrlMsg_.in_size = sizeof(VpcUserImageConfigure);
    int ret = DvppCtl(iDvppApi_, DVPP_CTL_VPC_PROC, &dvppApiCtrlMsg_);
    if (ret != APP_ERR_OK) {
        LogError << "resize fail, ret=" << ret;
        return APP_ERR_COMM_FAILURE;
    }
    return APP_ERR_OK;
}

APP_ERROR DvppCommonDevice::VpcMultiCrop(std::vector<DvppCropInputMsg> inputs)
{
    LogDebug << "VpcCrop Count: " << inputs.size();
    VpcUserRoiConfigure *lastRoi = nullptr;
    const uint32_t zero = 0;
    for (unsigned int i = 0; i < inputs.size(); i++) {
        DvppCropInputMsg input = inputs[i];
        uint32_t inWidthStride = DVPP_ALIGN_UP(input.inW, VPC_WIDTH_ALIGN);
        uint32_t inHeightStride = DVPP_ALIGN_UP(input.inH, VPC_HEIGHT_ALIGN);
        SetImageConfigure(imageConfigure_, inWidthStride, inHeightStride, input.imgBuf);

        // std::shared_ptr<struct VpcUserRoiConfigure> roiConfigure(new VpcUserRoiConfigure);
        struct VpcUserRoiConfigure *roiConfigure = new VpcUserRoiConfigure();
        roiConfigure->next = lastRoi;
        struct VpcUserRoiInputConfigure *inputConfigure = &(roiConfigure->inputConfigure);
        uint32_t leftOffset = input.roi.left / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN;
        uint32_t rightOffset = input.roi.right / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN - 1;
        uint32_t upOffset = input.roi.up / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN;
        uint32_t downOffset = input.roi.down / VPC_OFFSET_ALIGN * VPC_OFFSET_ALIGN - 1;
        SetCropConfigure(inputConfigure->cropArea, leftOffset, rightOffset, upOffset, downOffset);

        uint32_t outWidthStride = DVPP_ALIGN_UP(input.outW, VPC_WIDTH_ALIGN);
        uint32_t outHeightStride = DVPP_ALIGN_UP(input.outH, VPC_HEIGHT_ALIGN);
        struct VpcUserRoiOutputConfigure *outputConfigure = &(roiConfigure->outputConfigure);
        SetRoiOutputConfigure(outputConfigure, outWidthStride, outHeightStride, input.outBuf);
        SetCropConfigure(outputConfigure->outputArea, zero, outWidthStride - 1, zero, outHeightStride - 1);

        lastRoi = roiConfigure;
    }

    imageConfigure_.roiConfigure = lastRoi;

    dvppApiCtrlMsg_.in = static_cast<void *>(&imageConfigure_);
    dvppApiCtrlMsg_.in_size = sizeof(VpcUserImageConfigure);
    int ret = DvppCtl(iDvppApi_, DVPP_CTL_VPC_PROC, &dvppApiCtrlMsg_);

    while (lastRoi != nullptr) {
        struct VpcUserRoiConfigure *tmp = lastRoi->next;
        delete lastRoi;
        lastRoi = tmp;
    }

    if (ret != APP_ERR_OK) {
        LogError << "resize fail, ret=" << ret;
        return APP_ERR_COMM_FAILURE;
    }
    return APP_ERR_OK;
}

APP_ERROR DvppCommonDevice::DvppJpegDecode(DvppJpegDecodeInputMsg &input)
{
    // input setting
    jpegdIn_.jpegData = input.rawBuf;
    jpegdIn_.isYUV420Need = true;
    jpegdIn_.jpegDataSize = input.rawBufByteLength + DVPP_JPEG_OFFSET;
    jpegdIn_.isVBeforeU = false;
    // output setting
    uint32_t W_Aligned = DVPP_ALIGN_UP(input.jpegWidth, JPEG_WIDTH_ALIGN);
    uint32_t H_Aligned = DVPP_ALIGN_UP(input.jpegHeight, JPEG_HEIGHT_ALIGN);
    uint32_t outputBuffSize = W_Aligned * H_Aligned * YUV_BGR_SIZE_CONVERT_3 / YUV_BGR_SIZE_CONVERT_2;
    jpegdOut_.yuvData = input.decodedBuf;
    jpegdOut_.yuvDataSize = outputBuffSize;

    dvppApiCtrlMsg_.in = static_cast<void *>(&jpegdIn_);
    dvppApiCtrlMsg_.in_size = sizeof(JpegdIn);
    dvppApiCtrlMsg_.out = static_cast<void *>(&jpegdOut_);
    dvppApiCtrlMsg_.out_size = sizeof(JpegdOut);
    uint32_t ret = DvppCtl(iDvppApi_, DVPP_CTL_JPEGD_PROC, &dvppApiCtrlMsg_);
    if (ret != APP_ERR_OK) {
        LogError << "decode jpeg fail, ret=" << ret;
        return APP_ERR_COMM_FAILURE;
    }
    return APP_ERR_OK;
}

uint32_t DvppCommonDevice::GetBufferSize(uint32_t w, uint32_t h)
{
    uint32_t widthStride = DVPP_ALIGN_UP(w, VPC_WIDTH_ALIGN);
    uint32_t heightStride = DVPP_ALIGN_UP(h, VPC_HEIGHT_ALIGN);
    return widthStride * heightStride * YUV_BGR_SIZE_CONVERT_3 / YUV_BGR_SIZE_CONVERT_2;
}