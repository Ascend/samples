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

* File utils.cpp
* Description: handle file operations
*/
#include "utils.h"
#include <map>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <dirent.h>
#include <vector>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "acl/acl.h"

using namespace std;

namespace {
const std::string kImagePathSeparator = ",";
const int kStatSuccess = 0;
const std::string kFileSperator = "/";
const std::string kPathSeparator = "/";
// output image prefix
const std::string kOutputFilePrefix = "out_";
}

bool Utils::is_directory(const string &path) {
    // get path stat
    struct stat buf;
    if (stat(path.c_str(), &buf) != kStatSuccess) {
        return false;
    }

    // check
    if (S_ISDIR(buf.st_mode)) {
        return true;
    } else {
    return false;
    }
}

bool Utils::is_path_exist(const string &path) {
    ifstream file(path);
    if (!file) {
        return false;
    }
    return true;
}

void Utils::split_path(const string &path, vector<string> &path_vec) {
    char *char_path = const_cast<char*>(path.c_str());
    const char *char_split = kImagePathSeparator.c_str();
    char *tmp_path = strtok(char_path, char_split);
    while (tmp_path) {
        path_vec.emplace_back(tmp_path);
        tmp_path = strtok(nullptr, char_split);
    }
}

void Utils::get_all_files(const string &path, vector<string> &file_vec) {
    // split file path
    vector<string> path_vector;
    split_path(path, path_vector);

    for (string every_path : path_vector) {
        // check path exist or not
        if (!is_path_exist(path)) {
        ERROR_LOG("Failed to deal path=%s. Reason: not exist or can not access.",
                every_path.c_str());
        continue;
        }
        // get files in path and sub-path
        get_path_files(every_path, file_vec);
    }
}

void Utils::get_path_files(const string &path, vector<string> &file_vec) {
    struct dirent *dirent_ptr = nullptr;
    DIR *dir = nullptr;
    if (is_directory(path)) {
        dir = opendir(path.c_str());
        while ((dirent_ptr = readdir(dir)) != nullptr) {
            // skip . and ..
            if (dirent_ptr->d_name[0] == '.') {
            continue;
            }

            // file path
            string full_path = path + kPathSeparator + dirent_ptr->d_name;
            // directory need recursion
            if (is_directory(full_path)) {
                get_path_files(full_path, file_vec);
            } else {
                // put file
                file_vec.emplace_back(full_path);
            }
        }
    } 
    else {
        file_vec.emplace_back(path);
    }
}

void* Utils::copy_data_device_to_local(void* deviceData, uint32_t dataSize) {
    void* hostPtr = new uint8_t[dataSize];
    if (hostPtr == nullptr) {
        ERROR_LOG("malloc host data buffer failed");
        return nullptr;
    }

    aclError aclRet = aclrtMemcpy(hostPtr, dataSize, deviceData,
                                  dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("aclrtMemcpy device to host failed, aclRet is %d", aclRet);
        delete[]((uint8_t *)hostPtr);
        return nullptr;
    }

    return (uint8_t *)hostPtr;
}
