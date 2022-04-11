/**
* Copyright 2020 Huawei Technologies Co., Ltd
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
#include <stdlib.h>
#include <dirent.h>
#include <sys/time.h>
#include <regex>

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
#include "presentagentDisplay/presentagentDisplay.h"

#define NUM 1
#define INVALID -1
using namespace std;

namespace {
uint32_t kExitCount = 0;
bool kDisplay = false;
const char* kConfigFile = "../scripts/params.conf";
const char* kPresentagentFile = "../scripts/present_start.conf";
const string kRegexDeviceNum = "^device_num+$";
Channel* kPresenterChannel;
aclrtContext context;
aclrtRunMode runMode;
vector<aclrtContext> kContext;
}

int MainThreadProcess(uint32_t msgId, 
                      shared_ptr<void> msgData, void* userData) {
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

AclLiteError InitPresentAgent() {
    PresenterErrorCode ret = OpenChannelByConfig(kPresenterChannel, kPresentagentFile);
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Open channel failed, error %d", (int)ret);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Present Agent open success");
    return ACLLITE_OK;
}

AclLiteError ParseConfig(uint32_t& deviceNum) {
    map<string, string> config;
    if(!ReadConfig(config, kConfigFile)) {
        return ACLLITE_ERROR;
    }

    regex deviceNumRegex(kRegexDeviceNum.c_str());
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (regex_match(mIter->first, deviceNumRegex)) {
            deviceNum = stoi(mIter->second);
            ACLLITE_LOG_INFO("Data config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
    }

    return ACLLITE_OK;
}

void SetDisplay() {
    if (kDisplay){
        return;
    }
    uint32_t deviceNum;
    AclLiteError ret = ParseConfig(deviceNum);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Parse config fail in SetDisplay");
    }
    map<string, string> config;
    if(!ReadConfig(config, kConfigFile)) {
        ACLLITE_LOG_ERROR("read config fail in SetDisplay");
    }
    string outputTypeKey, outputType;
    uint32_t presentAgentNum = 0;
    for(int32_t i=0; i < deviceNum; i++){
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

void CreateInstances(vector<AclLiteThreadParam>& threadTbl, int32_t i,
                     aclrtContext& context, aclrtRunMode& runMode) {
    
    AclLiteThreadParam detectPreThreadParam;
    if (kDisplay) {
        detectPreThreadParam.threadInst = new DetectPreprocessThread(kConfigFile, i, runMode, true);
    }
    else {
        detectPreThreadParam.threadInst = new DetectPreprocessThread(kConfigFile, i, runMode);
    }
    detectPreThreadParam.threadInstName.assign(kDetectPreName[i].c_str());
    detectPreThreadParam.context = context;
    detectPreThreadParam.runMode = runMode;
    threadTbl.push_back(detectPreThreadParam);

    AclLiteThreadParam InferParam;
    InferParam.threadInst = new InferenceThread(runMode);
    InferParam.threadInstName.assign(kInferName[i].c_str());
    InferParam.context = context;
    InferParam.runMode = runMode;
    threadTbl.push_back(InferParam);

    AclLiteThreadParam detectPostThreadParam;
    detectPostThreadParam.threadInst = new DetectPostprocessThread();
    detectPostThreadParam.threadInstName.assign(kDetectPostName[i].c_str());
    detectPostThreadParam.context = context;
    detectPostThreadParam.runMode = runMode;
    threadTbl.push_back(detectPostThreadParam);

    AclLiteThreadParam classifyPreThreadParam;
    classifyPreThreadParam.threadInst = new ClassifyPreprocessThread(runMode);
    classifyPreThreadParam.threadInstName.assign(kClassifyPreName[i].c_str());
    classifyPreThreadParam.context = context;
    classifyPreThreadParam.runMode = runMode;
    threadTbl.push_back(classifyPreThreadParam);

    AclLiteThreadParam classifyPostThreadParam;
    classifyPostThreadParam.threadInst = new ClassifyPostprocessThread(kConfigFile, i);
    classifyPostThreadParam.threadInstName.assign(kClassifyPostName[i].c_str());
    classifyPostThreadParam.context = context;
    classifyPostThreadParam.runMode = runMode;
    threadTbl.push_back(classifyPostThreadParam);
}

void CreateThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev) {
    uint32_t deviceNum;
    runMode = aclDev.GetRunMode();

    AclLiteError ret = ParseConfig(deviceNum);
    if (ret != ACLLITE_OK) {
        return;
    }
    kExitCount = deviceNum;

    for(int32_t i=0; i < deviceNum; i++){
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
        kContext.push_back(context);
        CreateInstances(threadTbl, i, context, runMode);
    }

    if (kDisplay) {
        AclLiteThreadParam presentAgentDisplayThreadParam;
        presentAgentDisplayThreadParam.threadInst = new PresentAgentDisplayThread(kPresenterChannel);
        presentAgentDisplayThreadParam.threadInstName.assign(kPresentAgentDisplayName.c_str());
        presentAgentDisplayThreadParam.context = context;
        presentAgentDisplayThreadParam.runMode = runMode;
        threadTbl.push_back(presentAgentDisplayThreadParam);
    }
}

void ExitApp(AclLiteApp& app, vector<AclLiteThreadParam>& threadTbl) {
    for (int i = 0; i < threadTbl.size(); i++) {
        delete threadTbl[i].threadInst;
    }
    app.Exit();

    for(int i = 0; i < kContext.size(); i++) {
        aclrtDestroyContext(kContext[i]);
        aclrtResetDevice(i);
    }
}

void StartApp(AclLiteResource& aclDev) {
    vector<AclLiteThreadParam> threadTbl;
    CreateThreadInstance(threadTbl, aclDev);
    AclLiteApp& app = CreateAclLiteAppInstance();
    AclLiteError ret = app.Start(threadTbl);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Start app failed, error %d", ret);
        ExitApp(app, threadTbl);
        return;
    }

    for (int i = 0; i < threadTbl.size(); i++) {
        ret = SendMessage(threadTbl[i].threadInstId, MSG_APP_START, nullptr);
    }

    app.Wait(MainThreadProcess, nullptr);
    ExitApp(app, threadTbl);

    return;
}

int main(int argc, char *argv[]) {
    AclLiteResource aclDev = AclLiteResource();
    AclLiteError ret = aclDev.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init app failed");
    }
    SetDisplay();
    if (kDisplay) {
        ret = InitPresentAgent();
        if (ret != ACLLITE_OK){
            ACLLITE_LOG_ERROR("Init present agent failed");
        }
    }

    StartApp(aclDev);
    
    return ACLLITE_OK;
}
