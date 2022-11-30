/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef YOLOV3_COCO_DETECTION_VIDEO_DVPP_WITH_AIPP_INC_SAMPLE_PROCESS_H
#define YOLOV3_COCO_DETECTION_VIDEO_DVPP_WITH_AIPP_INC_SAMPLE_PROCESS_H

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
    AclLiteError Postprocess(const std::vector<InferenceOutput>& modelOutput,
                             cv::Mat& srcImg, int modelWidth, int modelHeight);

private:
    void DestroyResource();
    std::string g_streamName_;
    aclrtRunMode g_runMode_;
    AclLiteResource g_aclDev_;
    AclLiteImageProc g_dvpp_;
    AclLiteModel g_model_;
    uint32_t g_imageInfoSize_;
    void*    g_imageInfoBuf_;
    AclLiteVideoProc* g_cap_;
    cv::VideoWriter g_outputVideo_;
};

#endif