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

#include <iostream>
#include "acl/acl.h"
#include "object_detection.h"
#include "postprocess.h"
#include "AclLiteUtils.h"
#include "AclLiteApp.h"

using namespace std;

namespace {
    const vector<int> kColors_1 = {
        230, 150, 87, 34, 65, 92, 139, 210, 60, 189, 240, 21,
        40, 56, 77, 101, 168, 179, 199, 222};
    const vector<int> kColors_2 = {
        46, 235, 187, 234, 165, 222, 188, 0, 153, 43, 70, 221,
        184, 177, 217, 101, 32, 209, 199, 32};
    const vector<int> kColors_3 = {
        166, 50, 287, 134, 55, 192, 39, 210, 90, 109, 170, 161,
        40, 156, 177, 49, 48, 79, 89, 122};
}

PostprocessThread::PostprocessThread(uint32_t outputWidth, uint32_t outputHeight)
    : g_outputWidth_(outputWidth), g_outputHeight_(outputHeight)
{
}

PostprocessThread::~PostprocessThread()
{
}

AclLiteError PostprocessThread::Init()
{
    return ACLLITE_OK;
}

AclLiteError PostprocessThread::Process(int msgId, shared_ptr<void> data)
{
    struct timespec time5 = {0, 0};
    struct timespec time6 = {0, 0};
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<InferOutputMsg> inferMsg = static_pointer_cast<InferOutputMsg>(data);
    switch (msgId) {
        case MSG_INFER_OUTPUT:
            clock_gettime(CLOCK_REALTIME, &time5);
            InferOutputProcess(inferMsg);
            clock_gettime(CLOCK_REALTIME, &time6);
            cout << "postprocess time is: " << (time6.tv_sec - time5.tv_sec)*1000 + (time6.tv_nsec - time5.tv_nsec)/1000000 << "ms" << endl;
            break;
        case MSG_ENCODE_FINISH:
            SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
            break;
        default:
            ACLLITE_LOG_INFO("PostprocessThread thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError PostprocessThread::InferOutputProcess(shared_ptr<InferOutputMsg> inferMsg)
{
    float* data = (float *)inferMsg->inferData[0].data.get();
    if (data == nullptr) {
        ACLLITE_LOG_ERROR("inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t dataSize = inferMsg->inferData[0].size;
    uint32_t size = static_cast<uint32_t>(dataSize) / sizeof(float);

    // 读原图宽高
    cv::Mat originImage = cv::imread(inferMsg->imageFileName, CV_LOAD_IMAGE_COLOR);
    cv::Mat resizeImage;
    cv::resize(originImage, resizeImage, cv::Size(g_outputWidth_, g_outputHeight_));

    int pos = inferMsg->imageFileName.find_last_of("/");
    string filename(inferMsg->imageFileName.substr(pos + 1));
    cv::Mat mat_b_up, mat_g_up, mat_r_up;

    // 对每一张单通道图
    for (int i = 1; i < 21; i++) {
        // 保存为单通道图
        for (int j = i * (size / 21); j < (i + 1) * (size / 21); j++) {
            if (data[j] > 0.5) {
                // data[j] = kColors[i % kColors.size()];
                data[j] = 1;
            } else {
                data[j] = 0;
            }
        }
        cv::Mat mat_a(513, 513, CV_32FC1, const_cast<float*>((float*)data) + (size / 21)*i);
        int iVal255 = cv::countNonZero(mat_a);
        if (iVal255) {
            cv::Mat mat_a_up(g_outputHeight_, g_outputWidth_, CV_32FC1);
            cv::resize(mat_a, mat_a_up, cv::Size(g_outputWidth_, g_outputHeight_));        // 现在拿到一个原图掩码

            cv::multiply(mat_a_up, mat_a_up, mat_b_up, kColors_1[i % kColors_1.size()]);
            cv::multiply(mat_a_up, mat_a_up, mat_g_up, kColors_2[i % kColors_2.size()]);
            cv::multiply(mat_a_up, mat_a_up, mat_r_up, kColors_3[i % kColors_3.size()]);
            mat_b_up.convertTo(mat_b_up, CV_8U);
            mat_g_up.convertTo(mat_g_up, CV_8U);
            mat_r_up.convertTo(mat_r_up, CV_8U);
            cv::Mat newChannels[3] = { mat_b_up, mat_g_up, mat_r_up };
            cv::Mat resultImage;
            cv::merge(newChannels, 3, resultImage);
            cv::addWeighted(resizeImage, 1, resultImage, 1, 0, resizeImage);
        }
    }

    stringstream sstream;
    sstream.str("");
    sstream << "./out_" << filename;
    string outputPath = sstream.str();
    cv::imwrite(outputPath, resizeImage);

    return ACLLITE_OK;
}
