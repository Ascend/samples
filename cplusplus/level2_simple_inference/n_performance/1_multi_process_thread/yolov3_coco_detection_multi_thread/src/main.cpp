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
#include "AclLiteVideoProc.h"
#include "AclLiteUtils.h"
#include "object_detection.h"
#include "preprocess/preprocess.h"
#include "inference/inference.h"
#include "postprocess/postprocess.h"

using namespace std;
namespace {
uint32_t kExitCount = 0;
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
const char* kModelPath = "../model/yolov3.om";
const char* kConfigFile = "../scripts/yolov3_coco_detection_multi_thread.conf";
const string kRegexRtsp = "^rtsp_[0-9]+$";
}

int MainThreadProcess(uint32_t msgId, 
                      shared_ptr<void> msgData, void* userData) {
    //获取msgId然后计数判断何时退出
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

AclLiteError ParseConfig(vector<string>& rtspList) {
    map<string, string> config;
    if(!ReadConfig(config, kConfigFile)) {
        return ACLLITE_ERROR;
    }

    regex rtspAddrRegex(kRegexRtsp.c_str());
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        printf("config item: %s=%s\n", mIter->first.c_str(), mIter->second.c_str());
        if (regex_match(mIter->first, rtspAddrRegex)) {
            rtspList.push_back(mIter->second);
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
    }

    kExitCount = rtspList.size();
    return ACLLITE_OK;
}

void CreatePreprocessInstances(vector<AclLiteThreadParam>& threadTbl,
                               vector<string>& rtspList) {
    AclLiteThreadParam param;
    for (int i = 0; i < rtspList.size(); i++) {
        param.threadInst = new Preprocess(rtspList[i], kModelWidth,
                                          kModelHeight, i);
        threadTbl.push_back(param);
    }    
}

void CreateInferenceInstance(vector<AclLiteThreadParam>& threadTbl) {
    AclLiteThreadParam param;

    param.threadInst = new Inference(kModelPath, kModelWidth, 
                                            kModelHeight);
    param.threadInstName.assign(kInferName.c_str());
    threadTbl.push_back(param);
}

void CreatePostprocessInstances(vector<AclLiteThreadParam>& threadTbl,
                               int rtspNum) {
    AclLiteThreadParam param;
    for (int i = 0; i < rtspNum; i++) {  
        param.threadInst = new Postprocess(kModelWidth, kModelHeight);
        param.threadInstName.assign(kPostprocName[i].c_str());    
        threadTbl.push_back(param);
    }    
}

void CreateThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev) {
    vector<string> rtspList;
    AclLiteError ret = ParseConfig(rtspList);
    if (ret != ACLLITE_OK) {
        return;
    }
    CreatePreprocessInstances(threadTbl, rtspList);
    CreateInferenceInstance(threadTbl);
    CreatePostprocessInstances(threadTbl, rtspList.size());

    for (int i = 0; i < threadTbl.size(); i++) {
        threadTbl[i].context = aclDev.GetContext();
        threadTbl[i].runMode = aclDev.GetRunMode();
    }
}

void ExitApp(AclLiteApp& app, vector<AclLiteThreadParam>& threadTbl) {
    for (int i = 0; i < threadTbl.size(); i++) {
        delete threadTbl[i].threadInst;
    }  
    app.Exit();
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
        return ACLLITE_ERROR;
    }    
 
    StartApp(aclDev);
    return ACLLITE_OK;
}
