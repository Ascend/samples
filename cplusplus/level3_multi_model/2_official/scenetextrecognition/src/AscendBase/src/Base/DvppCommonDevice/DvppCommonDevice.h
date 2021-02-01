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

#ifndef DVPP_COMMON_DEVICE_H
#define DVPP_COMMON_DEVICE_H

#include "Dvpp.h"
#include "CommonDataType/CommonDataType.h"
#include "ErrorCode/ErrorCode.h"

#include "acl/ops/acl_dvpp.h"

struct DvppPicDescData {
    uint32_t width;
    uint32_t height;
    uint32_t widthStride;
    uint32_t heightStride;
    uint32_t bufferSize;
};

struct DvppResizeInputMsg {
    uint32_t inW;
    uint32_t inH;
    uint32_t outH;
    uint32_t outW;
    uint8_t *imgBuf;
    uint8_t *outBuf;
};

struct DvppCropInputMsg {
    uint32_t inW;
    uint32_t inH;
    uint32_t outH;
    uint32_t outW;
    uint8_t *imgBuf;
    uint8_t *outBuf;
    CropRoiConfig roi;
};

struct DvppJpegDecodeInputMsg {
    uint32_t rawBufByteLength;
    uint8_t *rawBuf;
    uint8_t *decodedBuf;
    uint32_t jpegWidth;
    uint32_t jpegHeight;
};
class DvppCommonDevice {
public:
    DvppCommonDevice() {};
    ~DvppCommonDevice() {};

    APP_ERROR init();
    APP_ERROR deinit();
    APP_ERROR VpcResize(DvppResizeInputMsg input);
    APP_ERROR VpcResizeWithPadding(DvppResizeInputMsg input);
    APP_ERROR VpcCrop(DvppCropInputMsg input);
    APP_ERROR VpcMultiCrop(std::vector<DvppCropInputMsg> inputs);
    APP_ERROR DvppJpegDecode(DvppJpegDecodeInputMsg &input);
    static uint32_t GetBufferSize(uint32_t w, uint32_t h);

private:
    IDVPPAPI *iDvppApi_;
    struct JpegdIn jpegdIn_;
    struct JpegdOut jpegdOut_;
    struct dvppapi_ctl_msg dvppApiCtrlMsg_;
    struct VpcUserImageConfigure imageConfigure_;

    void SetImageConfigure(VpcUserImageConfigure &imageConfigure, const uint32_t inWidthStride,
        const uint32_t inHeightStride, uint8_t *&inBuffer);
    void SetCropConfigure(VpcUserCropConfigure &cropArea, const uint32_t leftOffset, const uint32_t rightOffset,
        const uint32_t upOffset, const uint32_t downOffset);
    void SetRoiOutputConfigure(VpcUserRoiOutputConfigure *&outputConfigure, const uint32_t outWidthStride,
        const uint32_t outHeightStride, uint8_t *&outBuffer);
};
#endif