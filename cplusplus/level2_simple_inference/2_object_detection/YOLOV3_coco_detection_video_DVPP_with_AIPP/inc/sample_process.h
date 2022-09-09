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
#include "acl/acl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteError.h"
#include "acllite/AclLiteResource.h"
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteImageProc.h"
#include "AclLiteVideoProc.h"
#include "AclLiteVideoCapBase.h"
class SampleProcess {
public:
    SampleProcess(std::string streamName);
    virtual ~SampleProcess();

    AclLiteError InitResource();
    AclLiteError OpenVideoCapture();
    AclLiteError Process();

    void DrawBoundBoxToImage(std::vector<BBox>& detectionResults, cv::Mat& origImage);
    AclLiteError Postprocess(const std::vector<InferenceOutput>& modelOutput, cv::Mat& srcImg, int modelWidth, int modelHeight);

private:
    void DestroyResource();
    std::string streamName_;
    aclrtRunMode runMode_;
    AclLiteResource aclDev_;
    AclLiteImageProc dvpp_;
    AclLiteModel model_;
    uint32_t imageInfoSize_;
    void*    imageInfoBuf_;
    AclLiteVideoProc* cap_;
    cv::VideoWriter outputVideo_;
};

