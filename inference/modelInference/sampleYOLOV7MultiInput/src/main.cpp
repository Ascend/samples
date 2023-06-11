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
#include <regex>
#include <map>
#include <json/json.h>
#include "AclLiteResource.h"
#include "AclLiteApp.h"
#include "AclLiteThread.h"
#include "AclLiteType.h"
#include "AclLiteUtils.h"
#include "Params.h"
#include "dataInput/dataInput.h"
#include "dataOutput/dataOutput.h"
#include "detectInference/detectInference.h"
#include "detectPreprocess/detectPreprocess.h"
#include "detectPostprocess/detectPostprocess.h"
#include "pushrtsp/pushrtspthread.h"

using namespace std;
namespace {
uint32_t kExitCount = 0;
const string kJsonFile = "../scripts/test.json";
vector<aclrtContext> kContext;
uint32_t kBatch = 1;
int kPostNum = 1;
int kFramesPerSecond = 1000;
uint32_t kMsgQueueSize = 3;
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

void CreateALLThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev)
{
    aclrtRunMode runMode = aclDev.GetRunMode();
    Json::Reader reader;
    Json::Value root;
    ifstream srcFile(kJsonFile, ios::binary);
    if (!srcFile.is_open())
    {
        ACLLITE_LOG_ERROR("Fail to open test.json");
        return;
    }
    if (reader.parse(srcFile, root))
    {
        for (int i = 0; i < root["device_config"].size(); i++)
        {
            // Create context on the device
            uint32_t deviceId = root["device_config"][i]["device_id"].asInt();
            if (deviceId < 0) {
                ACLLITE_LOG_ERROR("Invaild deviceId: %d", deviceId);
                return;
            }
            aclrtContext context = aclDev.GetContextByDevice(deviceId);
            if (context == nullptr) {
                ACLLITE_LOG_ERROR("Get acl context in device %d failed", i);
                return;
            }
            kContext.push_back(context);
            for (int j = 0; j < root["device_config"][i]["model_config"].size(); j++)
            {
                // Get modelwidth, modelheight, modelpath, and thread name for each model thread
                string inferName = root["device_config"][i]["model_config"][j]["infer_thread_name"].asString();
                string modelPath = root["device_config"][i]["model_config"][j]["model_path"].asString();
                uint32_t modelWidth = root["device_config"][i]["model_config"][j]["model_width"].asInt();
                uint32_t modelHeigth = root["device_config"][i]["model_config"][j]["model_heigth"].asInt();
                if (root["device_config"][i]["model_config"][j]["model_batch"].type() != Json::nullValue)
                {
                    kBatch = root["device_config"][i]["model_config"][j]["model_batch"].asInt();
                }
                if (root["device_config"][i]["model_config"][j]["postnum"].type() != Json::nullValue)
                {
                    kPostNum =  root["device_config"][i]["model_config"][j]["postnum"].asInt();
                }

                if (root["device_config"][i]["model_config"][j]["frames_per_second"].type() != Json::nullValue)
                {
                    kFramesPerSecond =  root["device_config"][i]["model_config"][j]["frames_per_second"].asInt();
                }

                if (modelWidth < 0 || modelHeigth < 0 || kBatch < 1 || kPostNum < 1 || kFramesPerSecond < 1) {
                    ACLLITE_LOG_ERROR("Invaild model config is given! modelWidth: %d, modelHeigth: %d,"
                                      "batch: %d, postNum: %d, framesPerSecond: %d",
                                      modelWidth, modelHeigth, kBatch, kPostNum, kFramesPerSecond);
                    return;
                }
                // Create inferThread
                AclLiteThreadParam inferParam;
                inferParam.threadInst = new DetectInferenceThread(modelPath);
                inferParam.threadInstName.assign(inferName.c_str());
                inferParam.context = context;
                inferParam.runMode = runMode;
                threadTbl.push_back(inferParam);
                for (int k = 0; k < root["device_config"][i]["model_config"][j]["io_info"].size(); k++)
                {
                    // Get all information for each input data:
                    string inputPath = root["device_config"][i]["model_config"][j]["io_info"][k]["input_path"].asString();
                    string inputType = root["device_config"][i]["model_config"][j]["io_info"][k]["input_type"].asString();
                    string outputPath = root["device_config"][i]["model_config"][j]["io_info"][k]["output_path"].asString();
                    string outputType = root["device_config"][i]["model_config"][j]["io_info"][k]["output_type"].asString();
                    uint32_t channelId =  root["device_config"][i]["model_config"][j]["io_info"][k]["channel_id"].asInt();
                    string dataInputName = kDataInputName + to_string(channelId);
                    string preName = kPreName + to_string(channelId);
                    string dataOutputName = kDataOutputName + to_string(channelId);
                    string rtspDisplayName = kRtspDisplayName + to_string(channelId);
                    // Create Thread for the input data:
                    AclLiteThreadParam dataInputParam;
                    dataInputParam.threadInst = new DataInputThread(deviceId, channelId, runMode,
                        inputType, inputPath, inferName, kPostNum, kBatch, kFramesPerSecond);
                    dataInputParam.threadInstName.assign(dataInputName.c_str());
                    dataInputParam.context = context;
                    dataInputParam.runMode = runMode;
                    dataInputParam.queueSize = kMsgQueueSize;
                    threadTbl.push_back(dataInputParam);

                    AclLiteThreadParam detectPreParam;
                    detectPreParam.threadInst = new DetectPreprocessThread(modelWidth, modelHeigth, kBatch);
                    detectPreParam.threadInstName.assign(preName.c_str());
                    detectPreParam.context = context;
                    detectPreParam.runMode = runMode;
                    detectPreParam.queueSize = kMsgQueueSize;
                    threadTbl.push_back(detectPreParam);
                    for (int m= 0; m < kPostNum; m++)
                    {
                        string postName = kPostName + to_string(channelId) + "_" + to_string(m);
                        AclLiteThreadParam detectPostParam;
                        detectPostParam.threadInst = new DetectPostprocessThread(modelWidth, modelHeigth,
                            runMode, kBatch);
                        detectPostParam.threadInstName.assign(postName.c_str());
                        detectPostParam.context = context;
                        detectPostParam.runMode = runMode;
                        threadTbl.push_back(detectPostParam);
                    }
                    
                    AclLiteThreadParam dataOutputParam;
                    dataOutputParam.threadInst = new DataOutputThread(runMode, outputType, outputPath, kPostNum);
                    dataOutputParam.threadInstName.assign(dataOutputName.c_str());
                    dataOutputParam.context = context;
                    dataOutputParam.runMode = runMode;
                    threadTbl.push_back(dataOutputParam);

                    if (outputType == "rtsp")
                    {
                        AclLiteThreadParam rtspDisplayThreadParam;
                        rtspDisplayThreadParam.threadInst = new PushRtspThread(outputPath + to_string(channelId));
                        rtspDisplayThreadParam.threadInstName.assign(rtspDisplayName.c_str());
                        rtspDisplayThreadParam.context = context;
                        rtspDisplayThreadParam.runMode = runMode;
                        threadTbl.push_back(rtspDisplayThreadParam);
                    }
                    kExitCount++;
                }
            }
        }
    }
    srcFile.close();
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
        if (i != 0) {
            aclrtResetDevice(i);
        }
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

    for (int i = 0; i < threadTbl.size(); i++) {
        ret = SendMessage(threadTbl[i].threadInstId, MSG_APP_START, nullptr);
    }
    app.Wait(MainThreadProcess, nullptr);
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
    StartApp(aclDev);
    return ACLLITE_OK;
}
