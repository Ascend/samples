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
#include "utils.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

namespace {
const std::string g_imagePathSeparator = ",";
const int STATSUCCESS = 0;
const std::string g_fileSperator = "/";
const std::string g_pathSeparator = "/";
// output image prefix
const std::string g_outputFilePrefix = "out_";

}

bool Utils::IsDirectory(const string &path)
{
    // get path stat
    struct stat buf;
    if (stat(path.c_str(), &buf) != STATSUCCESS) {
        return false;
    }

    // check
    if (S_ISDIR(buf.st_mode)) {
        return true;
    } else {
        return false;
    }
}

bool Utils::IsPathExist(const string &path)
{
    ifstream file(path);
    if (!file) {
        return false;
    }
    return true;
}

void Utils::SplitPath(const string &path, vector<string> &path_vec)
{
    char *tmp_path = strtok(const_cast<char*>(path.c_str()), g_imagePathSeparator.c_str());
    while (tmp_path) {
        path_vec.emplace_back(tmp_path);
        tmp_path = strtok(nullptr, g_imagePathSeparator.c_str());
    }
}

void Utils::GetAllFiles(const string &path, vector<string> &file_vec)
{
    // split file path
    vector<string> path_vector;
    SplitPath(path, path_vector);

    for (string every_path : path_vector) {
        // check path exist or not
        if (!IsPathExist(every_path)) {
            ERROR_LOG("Failed to deal path=%s. Reason: not exist "
                      "or can not access.", every_path.c_str());
            continue;
        }
        // get files in path and sub-path
        GetPathFiles(every_path, file_vec);
    }
}

void Utils::GetPathFiles(const string &path, vector<string> &file_vec)
{
    struct dirent *dirent_ptr = nullptr;
    DIR *dir = nullptr;
    if (IsDirectory(path)) {
        dir = opendir(path.c_str());
        while ((dirent_ptr = readdir(dir)) != nullptr) {
            // skip . and ..
            if (dirent_ptr->d_name[0] == '.') {
                continue;
            }

            // file path
            string full_path = path + g_pathSeparator + dirent_ptr->d_name;
            // directory need recursion
            if (IsDirectory(full_path)) {
                GetPathFiles(full_path, file_vec);
            } else {
                // put file
                file_vec.emplace_back(full_path);
            }
        }
    } else {
        file_vec.emplace_back(path);
    }
}

void* Utils::CopyDataHostToDvpp(void* data, int size)
{
    void* buffer = nullptr;

    auto aclRet = acldvppMalloc(&buffer, size);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl malloc dvpp data failed, dataSize=%u, ret=%d",
                  size, aclRet);
        return nullptr;
    }
    INFO_LOG("malloc dvpp memory size %d ok", size);
    // copy input to device memory
    aclRet = aclrtMemcpy(buffer, size, data, size, ACL_MEMCPY_HOST_TO_DEVICE);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl memcpy data to dvpp failed, size %u, error %d", size, aclRet);
        acldvppFree(buffer);
        return nullptr;
    }
    INFO_LOG("copy data to dvpp ok");

    return buffer;
}

void* Utils::CopyDataDeviceToDvpp(void* data, int size)
{
    void* buffer = nullptr;

    auto aclRet = acldvppMalloc(&buffer, size);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl malloc dvpp data failed, dataSize=%u, ret=%d",
                  size, aclRet);
        return nullptr;
    }
    INFO_LOG("malloc dvpp memory size %d ok", size);
    // copy input to device memory
    aclRet = aclrtMemcpy(buffer, size, data, size, ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl memcpy data to dvpp failed, size %u, error %d", size, aclRet);
        acldvppFree(buffer);
        return nullptr;
    }
    INFO_LOG("copy data to dvpp ok");

    return buffer;
}

Result Utils::CopyImageDataToDvpp(ImageData& imageDevice, ImageData srcImage)
{
    aclrtRunMode runMode_;
    aclError ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    void* buffer = nullptr;
    if (runMode_ == ACL_HOST) {
        buffer = Utils::CopyDataHostToDvpp(srcImage.data.get(), srcImage.size);
        if (buffer == nullptr) {
            ERROR_LOG("Copy image to device failed");
            return FAILED;
        }
    } else {
        buffer = Utils::CopyDataDeviceToDvpp(srcImage.data.get(), srcImage.size);
        if (buffer == nullptr) {
            ERROR_LOG("Copy image to device failed");
            return FAILED;
        }
    }

    imageDevice.width = srcImage.width;
    imageDevice.height = srcImage.height;
    imageDevice.size = srcImage.size;
    imageDevice.data.reset((uint8_t*)buffer, [](uint8_t* p) { acldvppFree((void *)p); });
    return SUCCESS;
}

void* Utils::CopyDataDeviceToLocal(void* deviceData, uint32_t dataSize)
{
    uint8_t* buffer = new uint8_t[dataSize];
    if (buffer == nullptr) {
        ERROR_LOG("New malloc memory failed\n");
        return nullptr;
    }

    aclError aclRet = aclrtMemcpy(buffer, dataSize, deviceData, dataSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("Copy device data to local failed, aclRet is %d\n", aclRet);
        delete[](buffer);
        return nullptr;
    }

    return (void*)buffer;
}

Result Utils::CopyDeviceToLocal(ImageData& imageout, ImageData srcImage, aclrtRunMode mode)
{
    void* buffer = nullptr;
    if (mode == ACL_HOST) {
        buffer = Utils::CopyDataDeviceToLocal(srcImage.data.get(), srcImage.size);
    } else {
        buffer = Utils::CopyDataDeviceToLocal(srcImage.data.get(), srcImage.size);
    }

    if (buffer == nullptr) {
        ERROR_LOG("Copy image to local failed\n");
    return FAILED;
    }

    imageout.width = srcImage.width;
    imageout.height = srcImage.height;
    imageout.alignWidth = srcImage.alignWidth;
    imageout.alignHeight = srcImage.alignHeight;
    imageout.size = srcImage.size;
    imageout.data.reset((uint8_t*)buffer, [](uint8_t* p) { delete[](p);  });

    return SUCCESS;
}

void* Utils::CopyDataToDevice(void* data, uint32_t dataSize, aclrtMemcpyKind policy)
{
    void* buffer = nullptr;
    aclError aclRet = aclrtMalloc(&buffer, dataSize, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return nullptr;
    }

    aclRet = aclrtMemcpy(buffer, dataSize, data, dataSize, policy);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
        (void)aclrtFree(buffer);
        return nullptr;
    }

    return buffer;
}

void* Utils::CopyDataDeviceToDevice(void* deviceData, uint32_t dataSize)
{
    return CopyDataToDevice(deviceData, dataSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
}

void* Utils::CopyDataHostToDevice(void* deviceData, uint32_t dataSize)
{
    return CopyDataToDevice(deviceData, dataSize, ACL_MEMCPY_HOST_TO_DEVICE);
}

Result Utils::CopyImageDataToDevice(ImageData& imageDevice,
                                    ImageData srcImage, aclrtRunMode mode)
{
    void* buffer;
    if (mode == ACL_HOST)
        buffer = Utils::CopyDataHostToDevice(srcImage.data.get(), srcImage.size);
    else
        buffer = Utils::CopyDataDeviceToDevice(srcImage.data.get(), srcImage.size);

    if (buffer == nullptr) {
        ERROR_LOG("Copy image to device failed");
        return FAILED;
    }

    imageDevice.width = srcImage.width;
    imageDevice.height = srcImage.height;
    imageDevice.size = srcImage.size;
    imageDevice.data.reset((uint8_t*)buffer, [](uint8_t* p) { aclrtFree((void *)p); });

    return SUCCESS;
}

int Utils::ReadImageFile(ImageData& image, std::string fileName)
{
    struct stat sBuf;
    int fileStatus = stat(fileName.data(), &sBuf);
    if (fileStatus == -1) {
        ERROR_LOG("failed to get file");
        return FAILED;
    }
    if (S_ISREG(sBuf.st_mode) == 0) {
        ERROR_LOG("%s is not a file, please enter a file", fileName.c_str());
        return FAILED;
    }
    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ERROR_LOG("open file %s failed", fileName.c_str());
        return FAILED;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ERROR_LOG("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return FAILED;
    }

    binFile.seekg(0, binFile.beg);

    uint8_t* binFileBufferData = new(std::nothrow) uint8_t[binFileBufferLen];
    if (binFileBufferData == nullptr) {
        ERROR_LOG("malloc binFileBufferData failed");
        binFile.close();
        return FAILED;
    }
    binFile.read((char *)binFileBufferData, binFileBufferLen);
    binFile.close();

    int32_t ch = 0;
    acldvppJpegGetImageInfo(binFileBufferData, binFileBufferLen,
                            &(image.width), &(image.height), &ch);
    image.data.reset(binFileBufferData, [](uint8_t* p) { delete[](p); });
    image.size = binFileBufferLen;

    return SUCCESS;
}

void Utils::SaveBinFile(const string& filename, const void* data, uint32_t size)
{
    FILE *outFileFp = fopen(filename.c_str(), "wb+");
    if (outFileFp == nullptr) {
        ERROR_LOG("Save file %s failed for open error", filename.c_str());
        return;
    }
    fwrite(data, 1, size, outFileFp);

    fflush(outFileFp);
    fclose(outFileFp);
}
