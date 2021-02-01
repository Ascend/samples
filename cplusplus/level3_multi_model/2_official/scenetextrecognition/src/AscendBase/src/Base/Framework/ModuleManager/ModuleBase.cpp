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

#include "ModuleBase.h"
#include "Log/Log.h"
#include "BlockingQueue/BlockingQueue.h"
#include "ErrorCode/ErrorCode.h"
#include <sys/time.h>
#include "Statistic/Statistic.h"

namespace ascend_base_module {
const int INPUTQUEUE_WARN_SIZE = 32;
const double TIME_COUNTS = 1000.0;

void ModuleBase::assign_initargs(ModuleInitArgs &initArgs)
{
#ifdef ASCEND_MODULE_USE_ACL
    aclContext_ = initArgs.context;
    runMode_ = initArgs.runMode;
#endif
    pipelineName_ = initArgs.pipelineName;
    moduleName_ = initArgs.moduleName;
    instanceId_ = initArgs.instanceId;
    isStop_ = false;
}

// run module instance in a new thread created
APP_ERROR ModuleBase::Run()
{
    LogDebug << moduleName_ << "[" << instanceId_ << "] Run";
    processThr_ = std::thread(&ModuleBase::ProcessThread, this);
    return APP_ERR_OK;
}

// get the data from input queue then call process function in the new thread
void ModuleBase::ProcessThread()
{
    APP_ERROR ret;
#ifdef ASCEND_MODULE_USE_ACL
    ret = aclrtSetCurrentContext(aclContext_);
    if (ret != APP_ERR_OK) {
        LogFatal << "Fail to set context for " << moduleName_ << "[" << instanceId_ << "]"
                 << ", ret=" << ret << "(" << get_app_errcodeinfo(ret) << ").";
        return;
    }
#endif
    // if the module has no input queue, call process function directly.
    if (withoutInputQueue_ == true) {
        ret = process(nullptr);
        if (ret != APP_ERR_OK) {
            LogError << "Fail to process data for " << moduleName_ << "[" << instanceId_ << "]"
                     << ", ret=" << ret << "(" << get_app_errcodeinfo(ret) << ").";
        }
        return;
    }
    if (inputQueue_ == nullptr) {
        LogFatal << "Invalid input queue of " << moduleName_ << "[" << instanceId_ << "].";
        return;
    }
    LogDebug << "Input queue for " << moduleName_ << "[" << instanceId_ << "], inputQueue=" << inputQueue_;
    // repeatly pop data from input queue and call the process funtion. Results will be pushed to output queues.
    while (!isStop_) {
        std::shared_ptr<void> frameInfo = nullptr;
        ret = inputQueue_->Pop(frameInfo);
        if (ret == APP_ERR_QUEUE_STOPED) {
            LogDebug << moduleName_ << "[" << instanceId_ << "] input queue Stopped";
            break;
        } else if (ret != APP_ERR_OK || frameInfo == nullptr) {
            LogError << "Fail to get data from input queue for " << moduleName_ << "[" << instanceId_ << "]"
                     << ", ret=" << ret << "(" << get_app_errcodeinfo(ret) << ").";
            continue;
        }
        CallProcess(frameInfo);
    }
    LogInfo << moduleName_ << "[" << instanceId_ << "] process thread End";
}

void ModuleBase::CallProcess(std::shared_ptr<void> &frameAiInfo)
{
    struct timeval startTime = { 0, 0 };
    struct timeval endTime = { 0, 0 };
    gettimeofday(&startTime, nullptr);
    APP_ERROR ret = process(frameAiInfo);
    gettimeofday(&endTime, nullptr);
    double costMs =
        (endTime.tv_sec - startTime.tv_sec) * TIME_COUNTS + (endTime.tv_usec - startTime.tv_usec) / TIME_COUNTS;
    int queueSize = inputQueue_->GetSize();
    if (queueSize > INPUTQUEUE_WARN_SIZE) {
        LogWarn << "[Statistic] [Module] [" << moduleName_ << "] [" << instanceId_ << "] [QueueSize] [" << queueSize <<
            "] [process] [" << costMs << " ms]";
    }

    if (ret != APP_ERR_OK) {
        LogError << "Fail to process data for " << moduleName_ << "[" << instanceId_ << "]"
                 << ", ret=" << ret << "(" << get_app_errcodeinfo(ret) << ").";
    }
}

void ModuleBase::SetOutputInfo(std::string moduleName, ModuleConnectType connectType,
    std::vector<std::shared_ptr<BlockingQueue<std::shared_ptr<void>>>> outputQueVec)
{
    if (outputQueVec.size() == 0) {
        LogFatal << "outputQueVec is Empty! " << moduleName;
        return;
    }

    ModuleOutputInfo outputInfo;
    outputInfo.moduleName = moduleName;
    outputInfo.connectType = connectType;
    outputInfo.outputQueVec = outputQueVec;
    outputInfo.outputQueVecSize = outputQueVec.size();
    outputQueMap_[moduleName] = outputInfo;
}

const std::string ModuleBase::GetModuleName()
{
    return moduleName_;
}

const int ModuleBase::GetInstanceId()
{
    return instanceId_;
}

void ModuleBase::SetInputVec(const std::shared_ptr<BlockingQueue<std::shared_ptr<void>>>& inputQueue)
{
    inputQueue_ = inputQueue;
}

void ModuleBase::SendToNextModule(std::string moduleName, std::shared_ptr<void> outputData, int channelId)
{
    if (isStop_) {
        LogDebug << moduleName_ << "[" << instanceId_ << "] is Stopped, can't send to next module";
        return;
    }

    if (outputQueMap_.find(moduleName) == outputQueMap_.end()) {
        LogFatal << "No Next Module " << moduleName;
        return;
    }

    auto itr = outputQueMap_.find(moduleName);
    if (itr == outputQueMap_.end()) {
        LogFatal << "No Next Module " << moduleName;
        return;
    }
    ModuleOutputInfo outputInfo = itr->second;

    if (outputInfo.connectType == MODULE_CONNECT_ONE) {
        outputInfo.outputQueVec[0]->Push(outputData, true);
    } else if (outputInfo.connectType == MODULE_CONNECT_CHANNEL) {
        uint32_t ch = channelId % outputInfo.outputQueVecSize;
        if (ch >= outputInfo.outputQueVecSize) {
            LogFatal << "No Next Module!";
            return;
        }
        outputInfo.outputQueVec[ch]->Push(outputData, true);
    } else if (outputInfo.connectType == MODULE_CONNECT_PAIR) {
        outputInfo.outputQueVec[instanceId_]->Push(outputData, true);
    } else if (outputInfo.connectType == MODULE_CONNECT_RANDOM) {
        outputInfo.outputQueVec[sendCount_ % outputInfo.outputQueVecSize]->Push(outputData, true);
    }
    sendCount_++;
}

// clear input queue and stop the thread of the instance, called before destroy the instance
APP_ERROR ModuleBase::Stop()
{
#ifdef ASCEND_MODULE_USE_ACL
    APP_ERROR ret = aclrtSetCurrentContext(aclContext_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ModuleManager: fail to set context, ret[%d]" << ret << ".";
        return ret;
    }
#endif

    // stop input queue
    isStop_ = true;

    if (inputQueue_ != nullptr) {
        inputQueue_->Stop();
    }

    if (processThr_.joinable()) {
        processThr_.join();
    }

    return deinit();
}
}
