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
#include <stack>
#include "utils.h"

typedef struct CropAndPaste {
    uint32_t cropWidth = 0;
    uint32_t cropHeight = 0;
    uint32_t pasteWidth = 0;
    uint32_t pasteHeight = 0;
} CropAndPaste;

typedef enum WidthExtendDrection {
    TO_RIGHT,
    TO_LEFT,
    TO_LEFT_AND_RIGHT
} WidthExtendDrection;

typedef enum HeightExtendDrection {
    TO_BOTTOM,
    TO_TOP,
    TO_TOP_AND_BOTTOM
} HeightExtendDrection;

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
    * @param [in] inputWidth: input image width
    * @param [in] inputHeight: input image height
    * @param [in] inputFormat: input image format
    * @param [in] inFileName: input image file
    * @return result
    */
    void SetDvppInputPara(uint32_t inputWidth, uint32_t inputHeight, uint32_t inputFormat, std::string inFileName);

    /**
    * @brief init dvpp output para
    * @param [in] cropLeftOffset: crop left offset
    * @param [in] cropRightOffset: crop right offset
    * @param [in] cropTopOffset: crop top offset
    * @param [in] cropBottomOffset: crop bottom offset
    * @return result
    */
    void SetDvppCropPara(uint32_t cropLeftOffset, uint32_t cropRightOffset,
        uint32_t cropTopOffset, uint32_t cropBottomOffset);

    /**
    * @brief init dvpp output para
    * @param [in] outputWidth: output image width
    * @param [in] outputHeight: output image height
    * @param [in] outputFormat: output image format
    * @param [in] outFileName: output image file
    * @return result
    */
    void SetDvppOutputPara(uint32_t outputWidth, uint32_t outputHeight,
        uint32_t outputFormat, std::string outFileName);

    /**
    * @brief init dvpp output para
    * @param [in] pasteLeftOffset: paste left offset
    * @param [in] pasteRightOffset: paste right offset
    * @param [in] pasteTopOffset: paste top offset
    * @param [in] pasteBottomOffset: paste bottom offset
    * @return result
    */
    void SetDvppPastePara(uint32_t pasteLeftOffset, uint32_t pasteRightOffset,
        uint32_t pasteTopOffset, uint32_t pasteBottomOffset);

    /**
    * @brief check dvpp para
    * @return result
    */
    Result CheckParameter();

    /**
    * @brief dvpp process
    * @return result
    */
    Result Process();

private:

    bool CanProcessOnce(const CropAndPaste &cropAndPasteInfo);

    bool IsNeedSplit();

    void GetCropWidth(CropAndPaste &cropAndPasteInfo, uint32_t cropWidth, uint32_t pasteWidth);

    void GetCropHeight(CropAndPaste &cropAndPasteInfo, uint32_t cropHeight, uint32_t pasteHeight);

    void GetCropAndPasteInfo(CropAndPaste &cropAndPasteFirstInfo, CropAndPaste &cropAndPasteSecondInfo);

    void GetWidthExtendDirection(uint32_t extendedWidth);

    void GetHeightExtendDirection(uint32_t extendedHeight);

    void CalYuv400InputBufferSize(uint32_t inWidthStride, uint32_t inHeightStride, uint32_t &inBufferSize);

    Result BuildVpcParamterStack(std::stack<CropAndPaste> &cropAndPasteInfoStack);

    Result ModifyFirstCropRoi(CropAndPaste cropAndPasteFirstInfo);

    acldvppRoiConfig *GetSecondCropRoi(CropAndPaste cropAndPasteFirstInfo, CropAndPaste cropAndPasteSecondInfo);

    Result SplitProcessCropAndPaste();

    Result SplitProcessCropAndPasteFirst(CropAndPaste cropAndPasteFirstInfo);

    Result SplitProcessCropAndPasteSecond(CropAndPaste cropAndPasteFirstInfo,
        CropAndPaste cropAndPasteSecondInfo);

    Result ProcessCropAndPaste();

    uint32_t GetInputWidthStride();

    uint32_t GetInputBufferSize(uint32_t inWidthStride, uint32_t inHeightStride);

    Result InitCropAndPasteInputDesc();

    Result InitCropAndPasteOutputDesc();

    Result InitCropAndPasteResource();

    void DestroyCropAndPasteResource();

    void DestroyResource();

    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;

    std::string inFileName_; // input image name
    std::string outFileName_; // output image name
    uint32_t inWidth_; // input image width
    uint32_t inHeight_; // input image height
    uint32_t cropLeftOffset_; // crop left position, must even
    uint32_t cropRightOffset_; // crop right position, must odd
    uint32_t cropTopOffset_; // crop top position, must even
    uint32_t cropBottomOffset_; // crop bottom position, must odd
    uint32_t outWidth_; // output image width
    uint32_t outHeight_; // output image height
    uint32_t pasteLeftOffset_; // paste left position, must even and align to 16
    uint32_t pasteRightOffset_; // paste right position, must odd
    uint32_t pasteTopOffset_; // paste top position, must even
    uint32_t pasteBottomOffset_; // paste bottom position, must odd
    uint32_t inWidthStride_; // input width align to 16
    uint32_t inHeightStride_; // input height align to 2
    acldvppPixelFormat inFormat_; // input image format
    acldvppPixelFormat outFormat_; // output image format

    void *vpcInBufferDev_; // vpc input buffer
    void *vpcOutBufferDev_; // vpc output buffer

    acldvppPicDesc *vpcInputDesc_; // vpc input image desc
    acldvppPicDesc *vpcOutputDesc_; // vpc output image desc
    uint32_t vpcOutBufferSize_; // vpc output image size

    acldvppRoiConfig *cropArea_; // crop area
    acldvppRoiConfig *pasteArea_; // paste area

    uint32_t widthCoeffi_; // scaling factor for width
    uint32_t heightCoeffi_; // scaling factor for height
    WidthExtendDrection wExtendDirection_; // extend direction of width
    HeightExtendDrection hExtendDirection_; // extend direction of height

    void *vpcOutBufferDevFirst_; // output data of first vpc process
    acldvppPicDesc *vpcOutputDescFirst_; // output picture description of first vpc process
};
