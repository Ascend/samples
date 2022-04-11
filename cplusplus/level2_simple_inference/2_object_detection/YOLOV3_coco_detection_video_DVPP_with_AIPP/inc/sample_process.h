/**
* @file sample_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include "utils.h"
#include "acl/acl.h"

class SampleProcess {
public:
    /**
    * @brief Constructor
    */
    SampleProcess(std::string streamName);

    /**
    * @brief Destructor
    */
    virtual ~SampleProcess();

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource();

    /**
    * @brief sample process
    * @return result
    */
    Result Process();

    void DrawBoundBoxToImage(std::vector<BBox>& detectionResults, PicDesc &picDesc);

    void* GetInferenceOutputItem(uint32_t& itemDataSize, const aclmdlDataset* inferenceOutput, uint32_t idx);

    Result Postprocess(const aclmdlDataset* modelOutput, PicDesc &picDesc, int modelWidth, int modelHeight);

private:
    void DestroyResource();
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    std::string streamName_;
    aclrtRunMode runMode_;
    cv::VideoWriter outputVideo_;
};

