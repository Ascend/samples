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

#ifndef RESOURCEMANAGER_H
#define RESOURCEMANAGER_H

#include <vector>
#include <set>
#include <unordered_map>
#include <mutex>
#include "CommonDataType/CommonDataType.h"
#include "Log/Log.h"
#include "ErrorCode/ErrorCode.h"
#include "FileManager/FileManager.h"

enum ModelLoadMethod {
    LOAD_FROM_FILE = 0, // Loading from file, memory of model and weights are managed by ACL
    LOAD_FROM_MEM, // Loading from memory, memory of model and weights are managed by ACL
    LOAD_FROM_FILE_WITH_MEM, // Loading from file, memory of model and weight are managed by user
    LOAD_FROM_MEM_WITH_MEM // Loading from memory, memory of model and weight are managed by user
};

struct ModelInfo {
    std::string modelName;
    std::string modelPath; // Path of om model file
    size_t modelFileSize; // Size of om model file
    std::shared_ptr<void> modelFilePtr; // Smart pointer of model file data
    uint32_t modelWidth; // Input width of model
    uint32_t modelHeight; // Input height of model
    ModelLoadMethod method; // Loading method of model
};

// Device resource info, such as model infos, etc
struct DeviceResInfo {
    std::vector<ModelInfo> modelInfos;
};

struct ResourceInfo {
    std::set<int> deviceIds;
    std::string aclConfigPath;
    std::string singleOpFolderPath;
    std::unordered_map<int, DeviceResInfo> deviceResInfos; // map <deviceId, deviceResourceInfo>
};

class ResourceManager {
public:

    ~ResourceManager() {};

    // Get the Instance of resource manager
    static std::shared_ptr<ResourceManager> GetInstance();

    // init the resource of resource manager
    APP_ERROR InitResource(ResourceInfo& resourceInfo);

    aclrtContext GetContext(int deviceId);

    void release();

protected:

    ResourceManager() : initFlag(false) {};

private:

    static std::shared_ptr<ResourceManager> ptr_;
    bool initFlag;
    std::vector<int> deviceIds_;
    std::vector<aclrtContext> contexts_;
    std::unordered_map<int, int> deviceIdMap_; // Map of device to index
};

#endif