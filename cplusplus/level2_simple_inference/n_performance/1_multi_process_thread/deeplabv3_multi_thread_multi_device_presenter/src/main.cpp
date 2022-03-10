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
uint32_t kExitCount = 0;
uint32_t kModelWidth = 514;
uint32_t kModelHeight = 514;
const char* kModelPath = "../model/deeplab_quant.om";
const char* kConfigFile = "../scripts/deeplabv3_video_multi_thread.conf";
const string kRegexData = "^data_addr+$";
const string kRegexPostNum = "^postprocess_num+$";
const string kRegexOutputHeight = "^output_height+$";
const string kRegexOutputWidth = "^output_width+$";
const string kRegexDeviceNum = "^device_num+$";
aclrtContext context;
aclrtRunMode runMode;
vector<aclrtContext> kContext;
}

int MainThreadProcess(uint32_t msgId, 
                      shared_ptr<void> msgData, void* userData) {
    if (msgId == MSG_APP_EXIT) {
        AclLiteApp& app = GetAclLiteAppInstance();
        app.WaitEnd();
        ACLLITE_LOG_INFO("Receive exit message, exit now");
    }
    return ACLLITE_OK;
}

AclLiteError ParseConfig(string& dataAddr, uint32_t& postNum, uint32_t& deviceNum,
                       uint32_t& outputHeight, uint32_t& outputWidth) {
    map<string, string> config;
    if(!ReadConfig(config, kConfigFile)) {
        return ACLLITE_ERROR;
    }

    regex dataAddrRegex(kRegexData.c_str());
    regex postNumRegex(kRegexPostNum.c_str());
    regex outputHeightRegex(kRegexOutputHeight.c_str());
    regex outputWidthRegex(kRegexOutputWidth.c_str());
    regex deviceNumRegex(kRegexDeviceNum.c_str());
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        printf("config item: %s=%s\n", mIter->first.c_str(), mIter->second.c_str());
        if (regex_match(mIter->first, dataAddrRegex)) {
            dataAddr = mIter->second;
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
        else if (regex_match(mIter->first, postNumRegex)) {
            postNum = stoi(mIter->second);
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
        else if (regex_match(mIter->first, outputHeightRegex)) {
            outputHeight = stoi(mIter->second);
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
        else if (regex_match(mIter->first, outputWidthRegex)) {
            outputWidth = stoi(mIter->second);
            ACLLITE_LOG_INFO("Rtsp config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
        else if (regex_match(mIter->first, deviceNumRegex)) {
            deviceNum = stoi(mIter->second);
            ACLLITE_LOG_INFO("Data config item: %s=%s", 
                           mIter->first.c_str(), mIter->second.c_str());
        }
    }
    
    return ACLLITE_OK;
}

void CreatePreprocessInstances(vector<AclLiteThreadParam>& threadTbl,
                               string dataAddr, uint32_t postNum, uint32_t deviceCount, AclLiteResource& aclDev) {
    aclrtContext context = aclDev.GetContext();
    AclLiteThreadParam param;
    param.threadInst = new PreprocessThread(dataAddr, kModelWidth, kModelHeight, postNum, deviceCount, context);
    param.context = aclDev.GetContext();
    param.runMode = aclDev.GetRunMode();
    threadTbl.push_back(param);

}

void CreateInferenceInstance(vector<AclLiteThreadParam>& threadTbl, int32_t i, aclrtContext& context, aclrtRunMode& runMode) {
    AclLiteThreadParam param;

    param.threadInst = new InferenceThread(kModelPath, kModelWidth, 
                                           kModelHeight, context);
    param.threadInstName.assign(kInferName[i].c_str());
    param.context = context;
    param.runMode = runMode;
    threadTbl.push_back(param);
}

void CreatePostprocessInstances(vector<AclLiteThreadParam>& threadTbl,
                               uint32_t postNum, int32_t i,
                               aclrtContext& context, aclrtRunMode& runMode,
                               uint32_t outputHeight, uint32_t outputWidth) {
    AclLiteThreadParam param;
    for (int j = 0; j < postNum; j++) {  
        param.threadInst = new PostprocessThread(outputWidth, outputHeight);
        param.threadInstName.assign(kPostprocName[i*postNum+j].c_str());
        param.context = context;
        param.runMode = runMode;
        threadTbl.push_back(param);
    }
}

void CreateVideoprocessInstance(vector<AclLiteThreadParam>& threadTbl,
                                int outputHeight, int outputWidth, int postNum,
                                AclLiteResource& aclDev) {
    AclLiteThreadParam param;

    param.threadInst = new VideoprocessThread(outputHeight, outputWidth, postNum);
    param.threadInstName.assign(kVideoprocName.c_str());
    param.context = aclDev.GetContext();
    param.runMode = aclDev.GetRunMode();
    threadTbl.push_back(param);
}

void CreateThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev) {
    string dataAddr;
    uint32_t postNum;
    uint32_t outputHeight;
    uint32_t outputWidth;
    uint32_t deviceNum;

    runMode = aclDev.GetRunMode();

    AclLiteError ret = ParseConfig(dataAddr, postNum, deviceNum, outputHeight, outputWidth);
    if (ret != ACLLITE_OK) {
        return;
    }
    kExitCount = postNum * deviceNum;

    CreatePreprocessInstances(threadTbl, dataAddr, postNum, deviceNum, aclDev);
    CreateVideoprocessInstance(threadTbl, outputHeight, outputWidth, kExitCount, aclDev);
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

        CreateInferenceInstance(threadTbl, i, context, runMode);
        CreatePostprocessInstances(threadTbl, postNum, i, context, runMode, outputHeight, outputWidth);
    }
}

void ExitApp(AclLiteApp& app, vector<AclLiteThreadParam>& threadTbl) {
    for (int i = 0; i < threadTbl.size(); i++) {
        delete threadTbl[i].threadInst;
    }  
    app.Exit();

    for(int i = 0; i < kContext.size(); i++) {
        if(i){
            aclrtResetDevice(i);
        }
        aclrtDestroyContext(kContext[i]);
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


int main(int argc, char *argv[]) {
    AclLiteResource aclDev = AclLiteResource();
    AclLiteError ret = aclDev.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init app failed");
    }
    
    StartApp(aclDev);
    return ACLLITE_OK;
}
