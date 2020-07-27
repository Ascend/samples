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

* File utils.h
* Description: handle file operations
*/
#pragma once
#include <iostream>
#include <vector>
#include "acl/acl.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/imgproc/types_c.h"

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR]  " fmt "\n", ##args)
#define RGBU8_IMAGE_SIZE(width, height) ((width) * (height) * 3)

template<class Type>
std::shared_ptr<Type> MakeSharedNoThrow() {
    try {
        return std::make_shared<Type>();
    }
    catch (...) {
        return nullptr;
    }
}

#define MAKE_SHARED_NO_THROW(memory, memory_type) \
    do { \
            memory = MakeSharedNoThrow<memory_type>(); \
    }while(0);    

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
}Result;

struct ImageData {
    uint32_t width = 0;
    uint32_t height = 0;
    int32_t size = 0;
    void* data;
};

/**
 * Utils
 */
class Utils {
public:

    /**
    * @brief create device buffer of pic
    * @param [in] picDesc: pic desc
    * @param [in] PicBufferSize: aligned pic size
    * @return device buffer of pic
    */
    static bool IsDirectory(const std::string &path);

    static bool IsPathExist(const std::string &path);

    static void SplitPath(const std::string &path, std::vector<std::string> &path_vec);

    static void GetAllFiles(const std::string &path, std::vector<std::string> &file_vec);

    static void GetPathFiles(const std::string &path, std::vector<std::string> &file_vec);

    static void* CopyDataDeviceToLocal(void* deviceData, uint32_t dataSize);
};

