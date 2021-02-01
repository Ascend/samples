/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstring>
#include <fstream>
#include <csignal>

#include "CommandLine.h"
#include "ConfigParser/ConfigParser.h"
#include "Log/Log.h"
#include "Framework/ModuleManager/ModuleBase.h"
#include "Framework/ModuleManager/ModuleManager.h"
#include "ImageReader/ImageReader.h"
#include "ImagePreprocess/ImagePreprocess.h"
#include "TextDetection/TextDetection.h"
#include "DetectPost/DetectPost.h"
#include "WarpPerspective/WarpPerspective.h"
#include "TextRecognition/TextRecognition.h"
#include "ResultProcess/ResultProcess.h"

using namespace ascend_base_module;

bool g_signal_recieved = false;
namespace {
    ModuleDesc g_moduleDesc[] = {
        {MT_ImageReader, 1},
        {MT_ImagePreprocess, 1},
        {MT_TextDetection, 1},
        {MT_DetectPost, 1},
        {MT_WarpPerspective, 2},
        {MT_TextRecognition, 8},
        {MT_ResultProcess, 1},
    };

    ModuleConnectDesc g_connectDesc[] = {
        {MT_ImageReader, MT_ImagePreprocess, MODULE_CONNECT_CHANNEL},
        {MT_ImagePreprocess, MT_TextDetection, MODULE_CONNECT_CHANNEL},
        {MT_TextDetection, MT_DetectPost, MODULE_CONNECT_CHANNEL},
        {MT_DetectPost, MT_WarpPerspective, MODULE_CONNECT_RANDOM},
        {MT_WarpPerspective, MT_TextRecognition, MODULE_CONNECT_RANDOM},
        {MT_TextRecognition, MT_ResultProcess, MODULE_CONNECT_ONE},
    };

    const uint8_t MODULE_TYPE_COUNT = sizeof(g_moduleDesc) / sizeof(g_moduleDesc[0]);
    const int MODULE_CONNECT_COUNT = sizeof(g_connectDesc) / sizeof(g_connectDesc[0]);

    APP_ERROR InitModuleManager(ModuleManager &moduleManager, std::string &configPath)
    {
        ConfigParser configParser;
        // 1. create queues of receivers and senders
        APP_ERROR ret = configParser.parse_config(configPath);
        if (ret != APP_ERR_OK) {
            LogFatal << "Failed to parse config file << " << configPath << ", ret = " << ret << ".";
            return ret;
        }
        std::string itemCfgStr = std::string("SystemConfig.deviceId");
        int deviceId = 0;
        ret = configParser.GetIntValue(itemCfgStr, deviceId);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to parse device id, ret = " << ret << ".";
            return ret;
        }
        if (deviceId < 0) {
            LogError << "The device id " << deviceId << " is invalid, please check the configuration in setup.config.";
            return APP_ERR_COMM_INVALID_PARAM;
        }

        // init moduleManager without acl config file
        std::string aclConfigPath("");
        ret = moduleManager.init(configPath, aclConfigPath);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to init system manager, ret = " << ret << ".";
            return APP_ERR_COMM_FAILURE;
        }

        ret = moduleManager.RegisterModules(PIPELINE_DEFAULT, g_moduleDesc, MODULE_TYPE_COUNT, 1);
        if (ret != APP_ERR_OK) {
            return APP_ERR_COMM_FAILURE;
        }

        ret = moduleManager.RegisterModuleConnects(PIPELINE_DEFAULT, g_connectDesc, MODULE_CONNECT_COUNT);
        if (ret != APP_ERR_OK) {
            LogFatal << "Fail to connect module.";
            return APP_ERR_COMM_FAILURE;
        }
        return APP_ERR_OK;
    }

    APP_ERROR DeInitSystemManager(ModuleManager &moduleManager)
    {
        APP_ERROR ret = moduleManager.deinit();
        if (ret != APP_ERR_OK) {
            LogFatal << "Failed to deinit system manager, ret = " << ret << ".";
            return APP_ERR_COMM_FAILURE;
        }

        return APP_ERR_OK;
    }

    inline void MainAssert(int exp)
    {
        if (exp != APP_ERR_OK) {
            exit(exp);
        }
    }

    void SigHandler(int signo)
    {
        if (signo == SIGINT) {
            g_signal_recieved = true;
        }
    }
}

int main(int argc, const char *argv[])
{
    APP_ERROR ret;
    CmdParams cmdParams;
    ret = ParseACommandLine(argc, argv, cmdParams);
    if (ret != APP_ERR_OK) {
        return ret;
    }
    SetLogLevel(cmdParams.logLevel);

    ModuleManager moduleManager;
    std::string configPath("./Data/Config/setup.config");
    MainAssert(InitModuleManager(moduleManager, configPath));

    struct timeval pipelineBegin = {0};
    struct timeval pipelineEnd = {0};
    gettimeofday(&pipelineBegin, nullptr);
    MainAssert(moduleManager.RunPipeline());

    LogInfo << "wait for exit signal";
    if (signal(SIGINT, SigHandler) == SIG_ERR) {
        LogInfo << "cannot catch SIGINT.";
    }
    const uint16_t signalCheckIntervalUs = 1000;
    while (!g_signal_recieved) {
        usleep(signalCheckIntervalUs);
    }

    gettimeofday(&pipelineEnd, nullptr);
    // Calculate the performance for processing images
    const double costMs = SEC2MS * (pipelineEnd.tv_sec - pipelineBegin.tv_sec) + \
                          (pipelineEnd.tv_usec - pipelineBegin.tv_usec) / SEC2MS;
    size_t imageNum = ImageReader::GetTotalImageNum();
    const double fps = imageNum * SEC2MS / costMs;
    LogInfo << "[Performance] Total time: " << costMs << "ms, image number: " << imageNum << ", fps: " << fps;

    MainAssert(DeInitSystemManager(moduleManager));
    if (cmdParams.statEnable) {
        // The unit is second
        const int waitForStatisticResult = 5;
        sleep(waitForStatisticResult);
    }

    return APP_ERR_OK;
}
