/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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

* File main.cpp
* Description: dvpp sample main func
*/

#include <iostream>
#include <cstdlib>
#include <dirent.h>
#include <sys/time.h>
#include <regex>
#include <map>
#include "AclLiteResource.h"
#include "AclLiteApp.h"
#include "AclLiteThread.h"
#include "AclLiteType.h"
#include "VideoCapture.h"
#include "AclLiteUtils.h"
#include "CarParams.h"
#include "inference/inference.h"
#include "detectPreprocess/detectPreprocess.h"
#include "detectPostprocess/detectPostprocess.h"
#include "classifyPreprocess/classifyPreprocess.h"
#include "classifyPostprocess/classifyPostprocess.h"

#ifdef USE_PRESENT
#include "presentagentDisplay/presentagentDisplay.h"
#endif

#include "pushrtsp/pushrtspthread.h"

#define NUM (1)
#define INVALID (-1)
using namespace std;

#ifdef USE_PRESENT
using namespace ascend::presenter;
#endif

struct timespec time0 = {0, 0};
struct timespec time1 = {0, 0};

namespace {
uint32_t kExitCount = 0;
bool kDisplay = false;
const char* kConfigFile = "../scripts/params.conf";
const char* kPresentagentFile = "../scripts/present_start.conf";
const string kRegexDeviceNum = "^device_num+$";
const string kRegexRtspNumPerDevice = "^RtspNumPerDevice+$";
string kRtspUrl;

#ifdef USE_PRESENT
Channel* kPresenterChannel;
#endif

aclrtContext context;
aclrtRunMode runMode;
vector<aclrtContext> kContext;
}

int MainThreadProcess(uint32_t msgId,
                      shared_ptr<void> msgData, void* userData)
{
    if (msgId == MSG_APP_EXIT) {
        kExitCount--;
    }
    if (!kExitCount) {
        AclLiteApp& app = GetAclLiteAppInstance();
        app.WaitEnd();
        ACLLITE_LOG_INFO("Receive exit message, exit now");
    }
    return ACLLITE_OK;
}

#ifdef USE_PRESENT
AclLiteError InitPresentAgent()
{
    PresenterErrorCode ret = OpenChannelByConfig(kPresenterChannel, kPresentagentFile);
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Open channel failed, error %d", (int)ret);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Present Agent open success");
    return ACLLITE_OK;
}
#endif

AclLiteError ParseConfig(uint32_t& deviceNum, uint32_t& rtspNumPerDevice)
{
    map<string, string> config;
    if (!ReadConfig(config, kConfigFile)) {
        return ACLLITE_ERROR;
    }

    regex deviceNumRegex(kRegexDeviceNum.c_str());
    regex RtspNumPerDeviceRegex(kRegexRtspNumPerDevice.c_str());
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (regex_match(mIter->first, deviceNumRegex)) {
            deviceNum = stoi(mIter->second);
            ACLLITE_LOG_INFO("Data config item: %s=%s",
                             mIter->first.c_str(), mIter->second.c_str());
        } else if (regex_match(mIter->first, RtspNumPerDeviceRegex)) {
            rtspNumPerDevice = stoi(mIter->second);
            ACLLITE_LOG_INFO("Data config item: %s=%s",
                             mIter->first.c_str(), mIter->second.c_str());
        }
    }
    return ACLLITE_OK;
}

void SetDisplay()
{
    if (kDisplay) {
        return;
    }
    uint32_t deviceNum;
    uint32_t rtspNumPerDevice;
    AclLiteError ret = ParseConfig(deviceNum, rtspNumPerDevice);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Parse config fail in SetDisplay");
    }
    map<string, string> config;
    if (!ReadConfig(config, kConfigFile)) {
        ACLLITE_LOG_ERROR("read config fail in SetDisplay");
    }
    string outputTypeKey, outputType;
    uint32_t presentAgentNum = 0;
    for(int32_t i = 0; i < deviceNum; i++) {
        string outputTypeKey = "outputType_" + to_string(i);
        map<string, string>::const_iterator mIter = config.begin();
        for (; mIter != config.end(); ++mIter) {
            if (mIter->first == outputTypeKey) {
                outputType.assign(mIter->second.c_str());
            }
        }
        if (!outputType.empty() && outputType == "presentagent") {
            kDisplay = true;
            ACLLITE_LOG_INFO("Set kDisplay true");
        }
        ACLLITE_LOG_INFO("Set kDisplay false");
    }
}

bool readOutputTypeConfig(int channelId)
{
    map<string, string> config;
    if (!ReadConfig(config, kConfigFile)) {
        ACLLITE_LOG_ERROR("read config fail in SetDisplay");
    }
    string outputType;
    string outputTypeKey = "outputType_" + to_string(channelId);
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == "URL") {
            kRtspUrl.assign(mIter->second.c_str());
        }
        if (mIter->first == outputTypeKey) {
            outputType.assign(mIter->second.c_str());
            ACLLITE_LOG_INFO("channelId %d, outputType %s ", channelId, outputType.c_str());
            if (!outputType.empty() && outputType == "rtsp") {
                return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

void CreateThreadInstance(vector<AclLiteThreadParam>& threadTbl, int32_t deviceId,
                          aclrtContext& context, aclrtRunMode& runMode, int rtspNumPerDevice)
{
    AclLiteThreadParam detectPreThreadParam;
    for (int index = 0; index < rtspNumPerDevice; index++) {
        int channelId = deviceId*rtspNumPerDevice+index;
        if (kDisplay) {
            detectPreThreadParam.threadInst =
                new DetectPreprocessThread(kConfigFile, deviceId, channelId, runMode, true);
        } else {
            detectPreThreadParam.threadInst =
                new DetectPreprocessThread(kConfigFile, deviceId, channelId, runMode);
        }
        string DetectPreName = kDetectPreName + to_string(channelId);
        detectPreThreadParam.threadInstName.assign(DetectPreName.c_str());
        detectPreThreadParam.context = context;
        detectPreThreadParam.runMode = runMode;
        threadTbl.push_back(detectPreThreadParam);

        AclLiteThreadParam detectPostThreadParam;
        detectPostThreadParam.threadInst = new DetectPostprocessThread();
        string DetectPostName = kDetectPostName + to_string(channelId);
        detectPostThreadParam.threadInstName.assign(DetectPostName.c_str());
        detectPostThreadParam.context = context;
        detectPostThreadParam.runMode = runMode;
        threadTbl.push_back(detectPostThreadParam);

        AclLiteThreadParam classifyPreThreadParam;
        classifyPreThreadParam.threadInst = new ClassifyPreprocessThread(runMode);
        string ClassifyPreName = kClassifyPreName + to_string(channelId);
        classifyPreThreadParam.threadInstName.assign(ClassifyPreName.c_str());
        classifyPreThreadParam.context = context;
        classifyPreThreadParam.runMode = runMode;
        threadTbl.push_back(classifyPreThreadParam);

        AclLiteThreadParam classifyPostThreadParam;
        classifyPostThreadParam.threadInst = new ClassifyPostprocessThread(kConfigFile, channelId);
        string ClassifyPostName = kClassifyPostName + to_string(channelId);
        classifyPostThreadParam.threadInstName.assign(ClassifyPostName.c_str());
        classifyPostThreadParam.context = context;
        classifyPostThreadParam.runMode = runMode;
        threadTbl.push_back(classifyPostThreadParam);

        if (readOutputTypeConfig(channelId)) {
            AclLiteThreadParam rtspDisplayThreadParam;
            rtspDisplayThreadParam.threadInst = new PushRtspThread(kRtspUrl + to_string(channelId));
            string RtspDisplayName = kRtspDisplayName + to_string(channelId);
            rtspDisplayThreadParam.threadInstName.assign(RtspDisplayName.c_str());
            rtspDisplayThreadParam.context = context;
            rtspDisplayThreadParam.runMode = runMode;
            threadTbl.push_back(rtspDisplayThreadParam);
        }
    }

    AclLiteThreadParam InferParam;
    InferParam.threadInst = new InferenceThread(runMode);
    string InferName = kInferName + to_string(deviceId);
    InferParam.threadInstName.assign(InferName.c_str());
    InferParam.context = context;
    InferParam.runMode = runMode;
    threadTbl.push_back(InferParam);
}

void CreateALLThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev)
{
    uint32_t deviceNum;
    uint32_t rtspNumPerDevice;
    runMode = aclDev.GetRunMode();

    AclLiteError ret = ParseConfig(deviceNum, rtspNumPerDevice);
    if (ret != ACLLITE_OK) {
        return;
    }
    kExitCount = deviceNum * rtspNumPerDevice;

    for (int32_t i=0; i < deviceNum; i++) {
        context = aclDev.GetContextByDevice(i);
        if (context == nullptr) {
            ACLLITE_LOG_ERROR("Get acl context in device %d failed", i);
            return;
        }
        kContext.push_back(context);
        CreateThreadInstance(threadTbl, i, context, runMode, rtspNumPerDevice);
    }

    #ifdef USE_PRESENT
    if (kDisplay) {
        AclLiteThreadParam presentAgentDisplayThreadParam;
        presentAgentDisplayThreadParam.threadInst = new PresentAgentDisplayThread(kPresenterChannel);
        presentAgentDisplayThreadParam.threadInstName.assign(kPresentAgentDisplayName.c_str());
        presentAgentDisplayThreadParam.context = context;
        presentAgentDisplayThreadParam.runMode = runMode;
        threadTbl.push_back(presentAgentDisplayThreadParam);
    }
    #endif
}

void ExitApp(AclLiteApp& app, vector<AclLiteThreadParam>& threadTbl)
{
    for (int i = 0; i < threadTbl.size(); i++) {
        aclrtSetCurrentContext(threadTbl[i].context);
        delete threadTbl[i].threadInst;
    }
    app.Exit();

    for (int i = 0; i < kContext.size(); i++) {
        aclrtDestroyContext(kContext[i]);
        aclrtResetDevice(i);
    }
}

void StartApp(AclLiteResource& aclDev)
{
    vector<AclLiteThreadParam> threadTbl;
    CreateALLThreadInstance(threadTbl, aclDev);
    AclLiteApp& app = CreateAclLiteAppInstance();
    AclLiteError ret = app.Start(threadTbl);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Start app failed, error %d", ret);
        ExitApp(app, threadTbl);
        return;
    }

    // time print
    clock_gettime(CLOCK_REALTIME, &time0);
    for (int i = 0; i < threadTbl.size(); i++) {
        ret = SendMessage(threadTbl[i].threadInstId, MSG_APP_START, nullptr);
    }
    app.Wait(MainThreadProcess, nullptr);

    clock_gettime(CLOCK_REALTIME, &time1);
    cout<<"/*****************************/"<<endl;
    cout << "process time is: " << (time1.tv_sec - time0.tv_sec)*1000 +
        (time1.tv_nsec - time0.tv_nsec)/1000000 << "ms" << endl;
    cout<<"/*****************************/"<<endl;
    ExitApp(app, threadTbl);

    return;
}

int main()
{
    AclLiteResource aclDev = AclLiteResource();
    AclLiteError ret = aclDev.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init app failed");
    }
    SetDisplay();
    
    #ifdef USE_PRESENT
    if (kDisplay) {
        ret = InitPresentAgent();
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Init present agent failed");
        }
    }
    #endif

    StartApp(aclDev);
    return ACLLITE_OK;
}
