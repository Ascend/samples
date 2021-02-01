/*
 * Copyright (c) 2020.Huawei Technologies Co., Ltd. All rights reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ResourceManager.h"
#include <algorithm>

std::shared_ptr<ResourceManager> ResourceManager::ptr_ = nullptr;

void ResourceManager::release()
{
    APP_ERROR ret;
    for (size_t i = 0; i < deviceIds_.size(); i++) {
        if (contexts_[i] != nullptr) {
            ret = aclrtDestroyContext(contexts_[i]); // Destroy context
            if (ret != APP_ERR_OK) {
                LogError << "Failed to destroy context, ret = " << ret << ".";
                return;
            }
            contexts_[i] = nullptr;
        }
        ret = aclrtResetDevice(deviceIds_[i]); // Reset device
        if (ret != APP_ERR_OK) {
            LogError << "Failed to reset device, ret = " << ret << ".";
            return;
        }
    }
    ret = aclFinalize();
    if (ret != APP_ERR_OK) {
        LogError << "Failed to finalize acl, ret = " << ret << ".";
        return;
    }
    LogInfo << "Finalized acl successfully.";
}

std::shared_ptr<ResourceManager> ResourceManager::GetInstance()
{
    if (ptr_ == nullptr) {
        ResourceManager* temp = new ResourceManager();
        ptr_.reset(temp);
    }
    return ptr_;
}

APP_ERROR ResourceManager::InitResource(ResourceInfo& resourceInfo)
{
    std::string& aclConfigPath = resourceInfo.aclConfigPath;
    APP_ERROR ret;
    if (aclConfigPath.length() == 0) {
        // init acl without aclconfig
        ret = aclInit(nullptr);
    } else {
        ret = ExistFile(aclConfigPath);
        if (ret != APP_ERR_OK) {
            LogError << "Acl config file not exist, ret = " << ret << ".";
            return ret;
        }
        ret = aclInit(aclConfigPath.c_str()); // Initialize ACL
    }
    if (ret != APP_ERR_OK) {
        LogError << "Failed to init acl, ret = " << ret;
        return ret;
    }
    std::copy(resourceInfo.deviceIds.begin(), resourceInfo.deviceIds.end(), std::back_inserter(deviceIds_));
    LogInfo << "Initialized acl successfully.";
    // Open device and create context for each chip, note: it create one context for each chip
    for (size_t i = 0; i < deviceIds_.size(); i++) {
        deviceIdMap_[deviceIds_[i]] = i;
        ret = aclrtSetDevice(deviceIds_[i]);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to open acl device: " << deviceIds_[i];
            return ret;
        }
        LogInfo << "Open device " << deviceIds_[i] << " successfully.";
        aclrtContext context;
        ret = aclrtCreateContext(&context, deviceIds_[i]);
        if (ret != APP_ERR_OK) {
            LogError << "Failed to create acl context, ret = " << ret << ".";
            return ret;
        }
        LogInfo << "Created context for device " << deviceIds_[i] << " successfully";
        contexts_.push_back(context);
    }
    std::string singleOpPath = resourceInfo.singleOpFolderPath;
    if (!singleOpPath.empty()) {
        ret = aclopSetModelDir(singleOpPath.c_str()); // Set operator model directory for application
        if (ret != APP_ERR_OK) {
            LogError << "Failed to aclopSetModelDir, ret = " << ret << ".";
            return ret;
        }
    }
    LogInfo << "init resource successfully.";
    return APP_ERR_OK;
}

aclrtContext ResourceManager::GetContext(int deviceId)
{
    return contexts_[deviceIdMap_[deviceId]];
}