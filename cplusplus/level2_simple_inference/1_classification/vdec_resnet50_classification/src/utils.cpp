/**
* @file utils.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "utils.h"
#include <functional>
#if defined(_MSC_VER)
#include <windows.h>
#include <io.h>
#else
#include <unistd.h>
#include <dirent.h>
#endif

bool RunStatus::isDevice_ = false;

bool Utils::ReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize)
{
    // read data from file.
    FILE *fp = fopen(fileName, "rb");
    if (fp == nullptr) {
        ERROR_LOG("open file %s failed.", fileName);
        return false;
    }

    fseek(fp, 0, SEEK_END);
    long fileLenLong = ftell(fp);
    if (fileLenLong <= 0) {
        ERROR_LOG("file %s len is invalid.", fileName);
        fclose(fp);
        return false;
    }
    fseek(fp, 0, SEEK_SET);

    auto fileLen = static_cast<uint32_t>(fileLenLong);
    dataSize = fileLen;
    size_t readSize;
    // Malloc input device memory
    auto aclRet = acldvppMalloc(&dataDev, dataSize);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acl malloc dvpp data failed, dataSize = %u, errorCode = %d.",
            dataSize, static_cast<int32_t>(aclRet));
        fclose(fp);
        return false;
    }

    if (!RunStatus::GetDeviceStatus()) {
         void *dataHost = nullptr;
        auto aclRet = aclrtMallocHost(&dataHost, fileLen);
        if (dataHost == nullptr) {
            ERROR_LOG("acl malloc host data buffer failed. dataSize = %u, errorCode = %d.",
                fileLen, static_cast<int32_t>(aclRet));
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }

        readSize = fread(dataHost, 1, fileLen, fp);
        if (readSize < fileLen) {
            ERROR_LOG("need read file %s %u bytes, but only %zu read.", fileName, fileLen, readSize);
            (void)aclrtFreeHost(dataHost);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }

        // copy input to device memory
        aclRet = aclrtMemcpy(dataDev, dataSize, dataHost, fileLen, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acl memcpy data to dev failed, fileLen = %u, errorCode = %d.",
                fileLen, static_cast<int32_t>(aclRet));
            (void)aclrtFreeHost(dataHost);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }
        (void)aclrtFreeHost(dataHost);
    } else {
        readSize = fread(dataDev, 1, fileLen, fp);
        if (readSize < fileLen) {
            ERROR_LOG("need read file %s %u bytes, but only %zu read.", fileName, fileLen, readSize);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }
    }

    fclose(fp);
    return true;
}

bool Utils::WriteDeviceMemoryToFile(const char *fileName, void *dataDev, uint32_t dataSize)
{
    if (dataDev == nullptr) {
        ERROR_LOG("dataDev is nullptr!");
        return false;
    }

    // copy output to host memory
    void *data = nullptr;
    aclError aclRet;
    if (!(RunStatus::GetDeviceStatus())) {
        aclRet = aclrtMallocHost(&data, dataSize);
        if (data == nullptr) {
            ERROR_LOG("malloc host data buffer failed. dataSize = %u, errorCode = %d.",
                dataSize, static_cast<int32_t>(aclRet));
            return false;
        }
        aclRet = aclrtMemcpy(data, dataSize, dataDev, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acl memcpy data to host failed, dataSize=%u, ret=%d.", dataSize, aclRet);
            (void)aclrtFreeHost(data);
            return false;
        }
    } else {
        data = dataDev;
    }

    FILE *outFileFp = fopen(fileName, "wb+");
    if (outFileFp == nullptr) {
        ERROR_LOG("fopen out file %s failed, error=%s.", fileName, strerror(errno));
        (void)aclrtFreeHost(data);
        return false;
    }

    bool ret = true;
    size_t writeRet = fwrite(data, 1, dataSize, outFileFp);
    if (writeRet != dataSize) {
        ERROR_LOG("need write %u bytes to %s, but only write %zu bytes, error=%s.\n",
                      dataSize, fileName, writeRet, strerror(errno));
        ret = false;
    }

    if (!(RunStatus::GetDeviceStatus())) {
        (void)aclrtFreeHost(data);
    }
    fflush(outFileFp);
    fclose(outFileFp);
    return ret;
}

Result Utils::PullModelOutputData(aclmdlDataset *modelOutput, const char *fileName)
{
    size_t outDatasetNum = aclmdlGetDatasetNumBuffers(modelOutput);
    if (outDatasetNum == 0) {
        ERROR_LOG("aclmdlGetDatasetNumBuffers from model output failed, outDatasetNum = 0");
         return FAILED;
    }
    FILE *outputFile = fopen(fileName, "wb+");
    if (outputFile == nullptr) {
        ERROR_LOG("create output file %s failed", fileName);
        return FAILED;
    }
    for (size_t i = 0; i < outDatasetNum; ++i) {
        // get model output data
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(modelOutput, i);
        if (dataBuffer == nullptr) {
            ERROR_LOG("aclmdlGetDatasetBuffer from model output failed.");
            continue;
        }
        void *data = aclGetDataBufferAddr(dataBuffer);
        if (data == nullptr) {
            ERROR_LOG("aclGetDataBufferAddr from dataBuffer failed.");
            continue;
        }
        uint32_t bufferSize = aclGetDataBufferSize(dataBuffer);
        INFO_LOG("output[%zu] DataBuffer, buffer addr = %p, buffer size = %u",
                i, data, bufferSize);

        void *dataPtr = nullptr;
        aclError ret;
        if (!(RunStatus::GetDeviceStatus())) { // app is running in host
            ret = aclrtMallocHost(&dataPtr, bufferSize);
            if (ret !=  ACL_ERROR_NONE) {
                ERROR_LOG("malloc host data buffer failed, errorCode = %d.", static_cast<int32_t>(ret));
                fclose(outputFile);
                return FAILED;
            }
            // if app is running in host, need copy model output data from device to host
            ret = aclrtMemcpy(dataPtr, bufferSize, data, bufferSize, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_ERROR_NONE) {
                (void)aclrtFreeHost(dataPtr);
                ERROR_LOG("memcpy device to host failed, errorCode = %d.", static_cast<int32_t>(ret));
            }
            INFO_LOG("memcopy output data from device to host buffer success.");
        } else {
            dataPtr = data;
        }
        uint32_t len = static_cast<uint32_t>(bufferSize);
        fwrite(static_cast<char *>(dataPtr), len, sizeof(char), outputFile);
        INFO_LOG("create output file success, filename=%s, size=%u", fileName, len);

        if (!(RunStatus::GetDeviceStatus())) { // app is running in device
            (void)aclrtFreeHost(dataPtr);
        }
    }
    fclose(outputFile);
    return SUCCESS;
}

Result Utils::CheckFile(const char *fileName)
{
    int i = 0;
    INFO_LOG("start check result file:%s", fileName);
    while (i < 10) {
        std::ifstream f(fileName);
        if (f.good()) {
            break;
        }
        SleepTime(1); // sleep 1s
        INFO_LOG("check result, wait time [%ds]", i+1);
        i++;
    }
    // 10 is max time of checking
    if (i == 10) {
        ERROR_LOG("check result failed, timeout,expect file:%s", fileName);
        return FAILED;
    }
    INFO_LOG("check result success, file exist");
    return SUCCESS;
}

Result Utils::CheckAndCreateFolder(const char *foldName)
{
    INFO_LOG( "start check result fold:%s", foldName);
#if defined(_MSC_VER)
    DWORD ret = GetFileAttributes((LPCSTR)foldName);
    if (ret == INVALID_FILE_ATTRIBUTES) {
        BOOL flag = CreateDirectory((LPCSTR)foldName, nullptr);
        if (flag) {
            INFO_LOG("make dir successfully.");
        } else {
            INFO_LOG("make dir errorly.");
            return FAILED;
        }
    }
#else
    if (access(foldName , 0) == -1) {
        int flag=mkdir(foldName , 0777);
        if (flag == 0)
        {
            INFO_LOG("make dir successfully.");
        } else {
            ERROR_LOG("make dir errorly.");
            return FAILED;
        }
    }
#endif
    INFO_LOG("check result success, fold exist");
    return SUCCESS;
}

Result Utils::SaveModelOutputData(const char *srcfileName, const char *dstfileName)
{
    Result ret = CheckFile(srcfileName);
    if (ret != SUCCESS) {
        ERROR_LOG("model output file not exist.");
        return FAILED;
    }
    FILE *model_output;
    model_output = fopen(srcfileName,"rb" );

    FILE *model_output_txt;
    model_output_txt = fopen(dstfileName, "wb+");
    INFO_LOG("open result file: [%s]", dstfileName);

    int i = 0;
    float prop = 0.0;
    std::map<float, int, std::greater<float>> mp;
    std::map<float, int>::iterator ite;
    while (feof(model_output) == 0) {
        ite = mp.end();
        fread(&prop, sizeof(float), 1, model_output);
        mp.insert(ite, std::map<float, int>::value_type(prop, i));
        fprintf(model_output_txt, "%f, %d", prop, i);
        i++;
    }
    fclose(model_output);
    ite = mp.begin();
    float sum = 0.0;
    float max = ite->first;
    int classType = ite->second;
    for (i = 0 ; i < 5; i++) {
        sum+=ite->first;
        ite++;
    }
    fprintf(model_output_txt, "classType[%d], top1[%f], top5[%f]", classType, max, sum);
    fclose(model_output_txt);
    INFO_LOG( "result:classType[%d], top1[%f], top5[%f]", classType,max,sum);
    return SUCCESS;
}

std::vector<std::string> Utils::ReadDir(const char* folder)
{
    std::vector<std::string> fileList;
#if defined(_MSC_VER)
    std::string inputDirectory = folder;
    inputDirectory = inputDirectory.append("*");

    _finddata_t fileinfo;
    long long handle = (long long)_findfirst(inputDirectory.c_str(), &fileinfo);
    if (handle == -1) {
        ERROR_LOG("_findfirst failed!");
        return fileList;
    }

    do {
        DWORD ret = GetFileAttributes((LPCSTR)fileinfo.name);
        if (ret == FILE_ATTRIBUTE_DIRECTORY) {
            continue;
        }
        fileList.push_back(fileinfo.name);
    } while (!_findnext(handle, &fileinfo));

    _findclose(handle);
#else
    struct dirent *dirp;
    DIR* dir = opendir(folder);
    while ((dirp = readdir(dir)) != nullptr) {
        if (dirp->d_type == DT_REG) {
            fileList.push_back(dirp->d_name);
        }
    }
    closedir(dir);
#endif
    return fileList;
}

void Utils::RemoveDir(const char* outFolder_)
{
#if defined(_MSC_VER)
    RemoveDirectory((LPCSTR)outFolder_);
#else
    rmdir(outFolder_);
#endif
}

void Utils::SleepTime(unsigned int seconds)
{
#if defined(_MSC_VER)
    unsigned long secs = static_cast<unsigned long>(seconds);
    Sleep(secs * 1000); // sleep 1 second
#else
    sleep(seconds);
#endif
}
