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
#include <stdlib.h>
#include <dirent.h>
#include <sys/time.h>
#include <regex>

#include "AclLiteResource.h"
#include "acllite/AclLiteApp.h"
#include "acllite/AclLiteThread.h"
#include "AclLiteType.h"
#include "VideoCapture.h"
#include "AclLiteUtils.h"
#include "object_detection.h"
#include "preprocess/preprocess.h"
#include "inference/inference.h"
#include "postprocess/postprocess.h"
#include "videoprocess/videoprocess.h"

#define NUM 1
using namespace std;
struct timespec time9 = {0, 0};
struct timespec time10 = {0, 0};

namespace {
uint32_t g_exitCount = 0;
uint32_t g_height1 = 256;
uint32_t g_width1 = 256;
uint32_t g_height2 = 512;
uint32_t g_width2 = 512;
uint32_t g_height3 = 1024;
uint32_t g_width3 = 1024;
uint32_t g_modelWidth;
uint32_t g_modelHeight;
const char* g_modelPath;
const char* g_configFile = "../scripts/AnimeGANv2_video_multi_device.conf";
const string g_regexData = "^data_addr+$";
const string g_regexPostNum = "^postprocess_num+$";
const string g_regexVideoHeight = "^video_height+$";
const string g_regexVideoWidth = "^video_width+$";
const string g_regexDeviceNum = "^device_num+$";
aclrtContext context;
aclrtRunMode runMode;
vector<aclrtContext> g_contextVector;
}

int MainThreadProcess(uint32_t msgId,
                      shared_ptr<void> msgData, void* userData)
{
    if (msgId == MSG_APP_EXIT) {
        AclLiteApp& app = GetAclLiteAppInstance();
        app.WaitEnd();
        ACLLITE_LOG_INFO("Receive exit message, exit now");
    }
    return ACLLITE_OK;
}

AclLiteError ParseConfig(string& dataAddr, uint32_t& postNum, uint32_t& deviceNum,
                         uint32_t& videoHeight, uint32_t& videoWidth)
{
    map<string, string> config;
    if (!ReadConfig(config, g_configFile)) {
        return ACLLITE_ERROR;
    }

    regex dataAddrRegex(g_regexData.c_str());
    regex postNumRegex(g_regexPostNum.c_str());
    regex videoHeightRegex(g_regexVideoHeight.c_str());
    regex videoWidthRegex(g_regexVideoWidth.c_str());
    regex deviceNumRegex(g_regexDeviceNum.c_str());
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        printf("config item: %s=%s\n", mIter->first.c_str(), mIter->second.c_str());
        if (regex_match(mIter->first, dataAddrRegex)) {
            dataAddr = mIter->second;
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s",
                             mIter->first.c_str(), mIter->second.c_str());
        } else if (regex_match(mIter->first, postNumRegex)) {
            postNum = stoi(mIter->second);
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s",
                             mIter->first.c_str(), mIter->second.c_str());
        } else if (regex_match(mIter->first, videoHeightRegex)) {
            videoHeight = stoi(mIter->second);
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s",
                             mIter->first.c_str(), mIter->second.c_str());
        } else if (regex_match(mIter->first, videoWidthRegex)) {
            videoWidth = stoi(mIter->second);
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s",
                             mIter->first.c_str(), mIter->second.c_str());
        } else if (regex_match(mIter->first, deviceNumRegex)) {
            deviceNum = stoi(mIter->second);
            ACLLITE_LOG_INFO("Data config item: %s=%s",
                             mIter->first.c_str(), mIter->second.c_str());
        }
    }

    return ACLLITE_OK;
}

void CreatePreprocessInstances(vector<AclLiteThreadParam>& threadTbl,
                               string dataAddr, uint32_t postNum, uint32_t deviceCount, AclLiteResource& aclDev)
{
    aclrtContext context = aclDev.GetContext();
    AclLiteThreadParam param;
    param.threadInst = new PreprocessThread(dataAddr, g_modelWidth,
                                      g_modelHeight, postNum, deviceCount, context);
    param.context = aclDev.GetContext();
    param.runMode = aclDev.GetRunMode();
    threadTbl.push_back(param);
}

void CreateInferenceInstance(vector<AclLiteThreadParam>& threadTbl, int32_t i,
                             aclrtContext& context, aclrtRunMode& runMode)
{
    AclLiteThreadParam param;

    param.threadInst = new InferenceThread(g_modelPath, g_modelWidth, 
                                            g_modelHeight, context);
    param.threadInstName.assign(g_inferName[i].c_str());
    param.context = context;
    param.runMode = runMode;
    threadTbl.push_back(param);
}

void CreatePostprocessInstances(vector<AclLiteThreadParam>& threadTbl,
                                uint32_t postNum, int32_t i,
                                aclrtContext& context, aclrtRunMode& runMode,
                                uint32_t videoHeight, uint32_t videoWidth)
{
    AclLiteThreadParam param;
    for (int j = 0; j < postNum; j++) {
        param.threadInst = new PostprocessThread(videoWidth, videoHeight);
        param.threadInstName.assign(g_postprocName[i*postNum+j].c_str());
        param.context = context;
        param.runMode = runMode;
        threadTbl.push_back(param);
    }
}

void CreateVideoprocessInstance(vector<AclLiteThreadParam>& threadTbl,
                                uint32_t videoHeight, uint32_t videoWidth, uint32_t postNum,
                                AclLiteResource& aclDev)
{
    AclLiteThreadParam param;

    param.threadInst = new VideoprocessThread(videoHeight, videoWidth, postNum);
    param.threadInstName.assign(g_videoprocName.c_str());
    param.context = aclDev.GetContext();
    param.runMode = aclDev.GetRunMode();
    threadTbl.push_back(param);
}

void CreateThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev)
{
    string dataAddr;
    uint32_t postNum;
    uint32_t videoHeight;
    uint32_t videoWidth;
    uint32_t deviceNum;

    runMode = aclDev.GetRunMode();

    AclLiteError ret = ParseConfig(dataAddr, postNum, deviceNum, videoHeight, videoWidth);
    if (ret != ACLLITE_OK) {
        return;
    }
    g_exitCount = postNum * deviceNum;

    CreatePreprocessInstances(threadTbl, dataAddr, postNum, deviceNum, aclDev);
    CreateVideoprocessInstance(threadTbl, videoHeight, videoWidth, g_exitCount, aclDev);

    for (int32_t i = 0; i < deviceNum; i++) {
        ret = aclrtSetDevice(i);
        if (ret != ACL_ERROR_NONE) {
            ACLLITE_LOG_ERROR("Acl open device %d failed", i);
            return;
        }

        ret = aclrtCreateContext(&context, i);
        if (ret != ACL_ERROR_NONE) {
            ACLLITE_LOG_ERROR("Create acl context failed, error:%d", ret);
            return;
        }

        g_contextVector.push_back(context);

        CreateInferenceInstance(threadTbl, i, context, runMode);
        CreatePostprocessInstances(threadTbl, postNum, i, context, runMode, videoHeight, videoWidth);
    }
}

void ExitApp(AclLiteApp& app, vector<AclLiteThreadParam>& threadTbl)
{
    for (int i = 0; i < threadTbl.size(); i++) {
        delete threadTbl[i].threadInst;
    }
    app.Exit();

    for (int i = 0; i < g_contextVector.size(); i++) {
        aclrtDestroyContext(g_contextVector[i]);
        if (i) {
            aclrtResetDevice(i);
        }
    }
}

void StartApp(AclLiteResource& aclDev)
{
    vector<AclLiteThreadParam> threadTbl;
    CreateThreadInstance(threadTbl, aclDev);

    AclLiteApp& app = CreateAclLiteAppInstance();
    AclLiteError ret = app.Start(threadTbl);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Start app failed, error %d", ret);
        ExitApp(app, threadTbl);
        return;
    }

    clock_gettime(CLOCK_REALTIME, &time9);
    for (int i = 0; i < threadTbl.size(); i++) {
        ret = SendMessage(threadTbl[i].threadInstId, MSG_APP_START, nullptr);
    }

    app.Wait(MainThreadProcess, nullptr);
    clock_gettime(CLOCK_REALTIME, &time10);
    cout << "videoprocess time is: " << (time10.tv_sec - time9.tv_sec)*1000 + (time10.tv_nsec - time9.tv_nsec)/1000000 << "ms" << endl;
    ExitApp(app, threadTbl);

    return;
}


int main(int argc, char *argv[])
{
    int argNum = 2;
    if ((argc < argNum)) {
        ACLLITE_LOG_ERROR("invalid parameter number, must input four parameters.");
        ACLLITE_LOG_ERROR("Please input: ./main size");
        return ACLLITE_ERROR;
    } else {
        uint64_t width = atoll(argv[1]);
        uint64_t height = atoll(argv[1]);
        if (!(((height == g_height1) && (width == g_width1)) ||
            ((height == g_height2) && (width == g_width2)) ||
            ((height == g_height3) && (width == g_width3)))) {
            ACLLITE_LOG_ERROR("invalid dynamic hw, should be 256*256,512*512,1024*1024.");
            return ACLLITE_ERROR;
            }
    }

    g_modelWidth = atoi(argv[1]);
    g_modelHeight = atoi(argv[1]);
    if (g_width1 == atoi(argv[1])) {
        g_modelPath = "../model/AnimeGANv2_256.om";
    } else if (g_width2 == atoi(argv[1])) { 
        g_modelPath = "../model/AnimeGANv2_512.om";
    } else {
        g_modelPath = "../model/AnimeGANv2_1024.om";
    }

    AclLiteResource aclDev = AclLiteResource();
    AclLiteError ret = aclDev.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init app failed");
    }
    StartApp(aclDev);
    return ACLLITE_OK;
}