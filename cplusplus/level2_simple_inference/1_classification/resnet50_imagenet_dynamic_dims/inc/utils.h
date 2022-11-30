/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#ifndef RESNET50_IMAGENET_DYNAMIC_DIMS_INC_UTILS_H
#define RESNET50_IMAGENET_DYNAMIC_DIMS_INC_UTILS_H

#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <unistd.h>
#include <cstring>
#include <dirent.h>
#include <regex>
#include <vector>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "acl/acl.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...)fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

enum Result {
    SUCCESS = 0,
    FAILED = 1
};

class Utils {
public:
    static Result ReadBinFile(const std::string &fileName, void *&inputBuff, uint32_t &fileSize);
    static void* MemcpyToDeviceBuffer(const void* data, uint32_t size, aclrtRunMode runMode);
    static void GetAllFiles(const std::string &pathList, std::vector<std::string> &fileVec);
    static void SplitPath(const std::string &path, std::vector<std::string> &pathVec);
    static void GetPathFiles(const std::string &path, std::vector<std::string> &fileVec);
    static bool IsDirectory(const std::string &path);
    static bool IsPathExist(const std::string &path);
    static Result CheckPathIsFile(const std::string &fileName);
};
#endif
