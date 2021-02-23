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
#include "face_detection.h"
#include "preprocess/preprocess.h"
#include "inference/inference.h"
#include "postprocess/postprocess.h"

using namespace std;

namespace {
uint32_t kModelWidth = 304;
uint32_t kModelHeight = 300;
const char* kModelPath = "../model/face_detection.om";
const char* kConfigFile = "../scripts/multi_channels_rtsp.conf";
const string kRegexRtsp = "^rtsp_[0-9]+$";
}

int MainThreadProcess(uint32_t msgId, 
                      shared_ptr<void> msgData, void* userData) {
    if (msgId == MSG_APP_EXIT) {
        AtlasApp& app = GetAtlasAppInstance();
        app.WaitEnd();
        ATLAS_LOG_INFO("Receive exit message, exit now");
    }

    return ATLAS_OK;
}

AtlasError ParseConfig(vector<string>& rtspList, int& displayChannel) {
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
        } else if (mIter->first == "display_channel") {
            displayChannel = atoi(mIter->second.c_str());
            ATLAS_LOG_INFO("Display channel: %d", displayChannel);
        }
	}

    return ATLAS_OK;
}

void CreatePreprocessInstances(vector<AtlasThreadParam>& threadTbl,
                               vector<string>& rtspList,
                               int displayChannel) {
    AtlasThreadParam param;
    for (int i = 0; i < rtspList.size(); i++) {  
        bool display =  (displayChannel == i);
        param.threadInst = new Preprocess(rtspList[i], kModelWidth,
                                          kModelHeight, i, display);
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
                               int rtspNum,
                               int displayChannel) {
    bool display = (displayChannel >= 0) && (displayChannel < rtspNum);
    AtlasThreadParam param;
    param.threadInst = new Postprocess(kConfigFile, display);
    param.threadInstName.assign(kPostprocName.c_str());    
    threadTbl.push_back(param);

    if (rtspNum > 1) {
        param.threadInst = new Postprocess(kConfigFile, false);
        param.threadInstName.assign(kPostprocName2.c_str());
        threadTbl.push_back(param);
    }
}

void CreateThreadInstance(vector<AtlasThreadParam>& threadTbl, AclDevice& aclDev) {
    vector<string> rtspList;
    int displayChannel = -1;
    AtlasError ret = ParseConfig(rtspList, displayChannel);
    if (ret != ATLAS_OK) {
        return;
    }

    CreatePreprocessInstances(threadTbl, rtspList, displayChannel);
    CreateInferenceInstance(threadTbl);
    CreatePostprocessInstances(threadTbl, rtspList.size(), displayChannel);

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
