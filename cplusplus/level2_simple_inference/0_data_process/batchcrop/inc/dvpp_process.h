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
#include <vector>
#include "utils.h"
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
    * @brief set crop picture width and height
    */
    void SetOutArea(int outputWidth, int outputHeight);

    /**
    * @brief set crop picture name
    */
    void SetOutputFileName(const char *outputFileName);

    /**
    * @brief input picture num and out crop num
    * @return result
    */
    Result SetBatchSize(int inBatchSize, int outBatchSize);

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource();

    /**
    * @brief set input picture size and name
    */
    void SetBatchInputCrop(const std::vector<InputArea> &batchInput);

    /**
    * @brief calculate input picture buffersize
    */
    uint32_t CalculateInBufferSize(uint32_t inputWidth, uint32_t inputHeight, acldvppPixelFormat inFormat);

    /**
    * @brief  Process Batch Crop
    * @return result
    */
    Result ProcessBatchCrop();

    /**
    * @brief  DestroyBatchCropResource
    */
    void DestroyBatchCropResource();

private:
    /**
    * @brief  set crop area list
    * @return void
    */
    void InitCropArea();
    /**
    * @brief  InitBatchCropInputDesc
    * @return Result
    */
    Result InitBatchCropInputDesc();

    /**
    * @brief  InitBatchCropOutputDesc
    * @return Result
    */
    Result InitBatchCropOutputDesc();

    /**
    * @brief  WriteOutputFile
    * @return Result
    */
    Result WriteOutputFile();

    void CalYuv400InputBufferSize(uint32_t inWidthStride, uint32_t inHeightStride, uint32_t &inBufferSize);

    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;
    acldvppResizeConfig *resizeConfig_;

    std::vector<void *> vecOutPtr_;
    std::vector<void *> vecInPtr_;

    acldvppBatchPicDesc *inputBatchPicDesc_;   // vpc input desc
    acldvppBatchPicDesc *outputBatchPicDesc_;  // vpc output desc

    uint32_t inputBatchSize_;
    uint32_t outputBatchSize_;

    const char *outputFileName_;

    uint32_t inputWidthStride_;
    uint32_t inputHeightStride_;

    uint32_t outputWidth_;   // output pic width
    uint32_t outputHeight_;  // output pic height

    std::vector<Area> batchCropArea_;
    std::vector<InputArea> batchInput_;
};
