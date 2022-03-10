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
#include "params.h"
#include "inference/inference.h"
#include "detectPreprocess/detectPreprocess.h"
#include "detectPostprocess/detectPostprocess.h"
#include "classifyPreprocess/classifyPreprocess.h"
#include "classifyPostprocess/classifyPostprocess.h"

#define NUM 1
using namespace std;
struct timespec time11 = {0, 0};
struct timespec time12 = {0, 0};

namespace {
uint32_t kExitCount = 0;
const char* kConfigFile = "../scripts/video_multi_models.conf";
const string kRegexDeviceNum = "^device_num+$";
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

void CreateInstances(vector<AclLiteThreadParam>& threadTbl, int32_t i,
                     aclrtContext& context, aclrtRunMode& runMode) {
    
    AclLiteThreadParam detectPreThreadParam;
    detectPreThreadParam.threadInst = new DetectPreprocessThread(kConfigFile, i, runMode);
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

    clock_gettime(CLOCK_REALTIME, &time11);
    for (int i = 0; i < threadTbl.size(); i++) {
        ret = SendMessage(threadTbl[i].threadInstId, MSG_APP_START, nullptr);
    }

    app.Wait(MainThreadProcess, nullptr);
    clock_gettime(CLOCK_REALTIME, &time12);
    //cout << "videoprocess time is: " << (time12.tv_sec - time11.tv_sec)*1000 + (time12.tv_nsec - time11.tv_nsec)/1000000 << "ms" << endl;
    ExitApp(app, threadTbl);

    return;
}

int main(int argc, char *argv[]) {
    AclLiteResource aclDev = AclLiteResource();
    AclLiteError ret = aclDev.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init app failed");
    }
    
    StartApp(aclDev);
    return ACLLITE_OK;
}
