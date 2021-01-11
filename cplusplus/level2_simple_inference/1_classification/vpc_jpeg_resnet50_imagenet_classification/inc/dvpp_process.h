/**
* @file dvpp_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include <cstdint>
#include "utils.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

class DvppProcess {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    explicit DvppProcess(aclrtStream &stream);

    /**
    * @brief Destructor
    */
    virtual ~DvppProcess();

    /**
    * @brief dvpp init
    * @return result
    */
    Result InitResource();

    /**
    * @brief init dvpp output para
    * @param [in] modelInputWidth: model input width
    * @param [in] modelInputHeight: model input height
    * @return result
    */
    Result InitDvppOutputPara(int modelInputWidth, int modelInputHeight);

    /**
    * @brief set jpegd input
    * @param [in] inDevBuffer: device buffer of input pic
    * @param [in] inDevBufferSize: device buffer size of input pic
    * @param [in] picDesc:picture description
    */
    void SetInput4JpegD(char *inDevBuffer, uint32_t inDevBufferSize, const PicDesc &picDesc);

    /**
    * @brief set jpege input
    * @param [in] inDevBuffer: device buffer of input yuv file
    * @param [in] inDevBufferSize: device input pic buffer size after align
    * @param [in] inputWidth:width of pic after encode
    * @param [in] inputHeight:height of pic after encode
    */
    void SetInput4JpegE(char *inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight);

    /**
    * @brief get dvpp output
    * @param [in] outputBuffer: pointer which points to dvpp output buffer
    * @param [out] outputSize: output size
    */
    void GetDvppOutput(void **outputBuffer, int &outputSize);

    /**
    * @brief dvpp process
    * @return result
    */
    Result Process();

    /**
    * @brief set dvpp type after JpegD(vpcResize/vpcCrop/vpcCropAndPaste)
    * @return result
    */
    void SetDvppType(DvppType dvppType);

    /**
    * @brief compute encode input pic desc size
    * @return input pic desc size
    */
    uint32_t ComputeEncodeInputSize(int inputWidth, int inputHeight);

    /**
    * @brief process encode
    * @return result
    */
    Result ProcessJpegE();

    /**
    * @brief process 8k resize
    * @return result
    */
    Result Process8kResize();

private:
    Result InitDecodeOutputDesc();
    Result ProcessDecode();
    void DestroyDecodeResource();

    Result InitResizeInputDesc();
    Result Init8kResizeInputDesc();
    Result InitResizeOutputDesc();
    Result Init8kResizeOutputDesc();
    Result ProcessResize();
    void DestroyResizeResource();

    Result InitCropInputDesc();
    Result InitCropOutputDesc();
    Result ProcessCrop();
    void DestroyCropResource();

    Result InitCropAndPasteInputDesc();
    Result InitCropAndPasteOutputDesc();
    Result ProcessCropAndPaste();
    void DestroyCropAndPasteResource();

    Result InitEncodeResource();
    void DestroyEncodeResource();

    void DestroyResource();
    void DestroyDvppOutputPara();
    void DestroyDecodeOutBuff();

    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;

    DvppType dvppType_;
    acldvppRoiConfig *cropArea_;
    acldvppRoiConfig *pasteArea_;
    acldvppJpegeConfig *jpegeConfig_;
    acldvppResizeConfig *resizeConfig_;

    void* decodeOutBufferDev_; // decode output buffer
    acldvppPicDesc *decodeOutputDesc_; //decode output desc

    void* encodeOutBufferDev_; // encode output buffer
    uint32_t encodeOutBufferSize_; // encode output buffer size
    acldvppPicDesc *encodeInputDesc_; //encode input desc

    acldvppPicDesc *vpcInputDesc_; // vpc input desc
    acldvppPicDesc *vpcOutputDesc_; // vpc output desc

    char *inDevBuffer_;  // input pic dev buffer
    uint32_t inDevBufferSizeD_; // input pic size for decode
    uint32_t inDevBufferSizeE_; // input pic size for encode
    uint32_t jpegDecodeOutputSize_; // jpeg decode output size

    uint32_t decodeOutputWidth_; // decode output width
    uint32_t decodeOutputWidthStride_; // decode output width aligned
    uint32_t decodeOutputHeight_; // decode output height

    void *vpcInBufferDev_; // vpc input buffer
    void *vpcOutBufferDev_; // vpc output buffer
    uint32_t vpcOutBufferSize_;  // vpc output size

    uint32_t modelInputWidth_; // model input width
    uint32_t modelInputHeight_; // model input height

    uint32_t jpegeInputWidth_; // encode input width
    uint32_t jpegeInputHeight_; // encode input height
};

