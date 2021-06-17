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

#include "acl_device.h"
#include "atlas_app.h"
#include "atlas_thread.h"
#include "atlas_type.h"
#include "atlas_videocapture.h"
#include "parse_config.h"
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
        AtlasApp& app = GetAtlasAppInstance();
        app.WaitEnd();
        ATLAS_LOG_INFO("Receive exit message, exit now");
    }
    return ATLAS_OK;
}

AtlasError ParseConfig(vector<string>& rtspList) {
    map<string, string> config;
    if(!ReadConfig(config, kConfigFile)) {
        return ATLAS_ERROR;
    }

    regex rtspAddrRegex(kRegexRtsp.c_str());
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        printf("config item: %s=%s\n", mIter->first.c_str(), mIter->second.c_str());
        if (regex_match(mIter->first, rtspAddrRegex)) {
            rtspList.push_back(mIter->second);
            ATLAS_LOG_INFO("Rtsp config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
    }

    kExitCount = rtspList.size();
    return ATLAS_OK;
}

void CreatePreprocessInstances(vector<AtlasThreadParam>& threadTbl,
                               vector<string>& rtspList) {
    AtlasThreadParam param;
    for (int i = 0; i < rtspList.size(); i++) {
        param.threadInst = new Preprocess(rtspList[i], kModelWidth,
                                          kModelHeight, i);
        threadTbl.push_back(param);
    }    
}

void CreateInferenceInstance(vector<AtlasThreadParam>& threadTbl) {
    AtlasThreadParam param;

    param.threadInst = new Inference(kModelPath, kModelWidth, 
                                            kModelHeight);
    param.threadInstName.assign(kInferName.c_str());
    threadTbl.push_back(param);
}

void CreatePostprocessInstances(vector<AtlasThreadParam>& threadTbl,
                               int rtspNum) {
    AtlasThreadParam param;
    for (int i = 0; i < rtspNum; i++) {  
        param.threadInst = new Postprocess(kModelWidth, kModelHeight);
        param.threadInstName.assign(kPostprocName[i].c_str());    
        threadTbl.push_back(param);
    }    
}

void CreateThreadInstance(vector<AtlasThreadParam>& threadTbl, AclDevice& aclDev) {
    vector<string> rtspList;
    AtlasError ret = ParseConfig(rtspList);
    if (ret != ATLAS_OK) {
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

void ExitApp(AtlasApp& app, vector<AtlasThreadParam>& threadTbl) {
    for (int i = 0; i < threadTbl.size(); i++) {
        delete threadTbl[i].threadInst;
    }  
    app.Exit();
}

void StartApp(AclDevice& aclDev) {
    vector<AtlasThreadParam> threadTbl;
    CreateThreadInstance(threadTbl, aclDev);

    AtlasApp& app = CreateAtlasAppInstance(); 
    AtlasError ret = app.Start(threadTbl);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Start app failed, error %d", ret);
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
    AclDevice aclDev = AclDevice();
    AtlasError ret = aclDev.Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init app failed");
        return ATLAS_ERROR;
    }    
 
    StartApp(aclDev);
    return ATLAS_OK;
}
