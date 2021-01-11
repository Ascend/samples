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
#include <map>
#include <functional>
#include <iostream>
#include <fstream>
#include <cstring>
#if defined(_MSC_VER)
#include <windows.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

bool RunStatus::isDevice_ = false;

Result Utils::ReadBinFile(const std::string &fileName, void *&inputBuff, uint32_t &fileSize)
{
    std::ifstream binFile(fileName, std::ifstream::binary);
    if (!binFile.is_open()) {
        ERROR_LOG("open file %s failed", fileName.c_str());
        return FAILED;
    }

    binFile.seekg(0, binFile.end);
    auto binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ERROR_LOG("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return FAILED;
    }
    binFile.seekg(0, binFile.beg);

    aclError aclRet;
    if (!(RunStatus::GetDeviceStatus())) { // app is running in host
        aclRet = aclrtMallocHost(&inputBuff, binFileBufferLen);
        if (inputBuff == nullptr) {
            ERROR_LOG("host malloc binFileBufferData failed, errorCode = %d", static_cast<int32_t>(aclRet));
            binFile.close();
            return FAILED;
        }
    } else { // app is running in device
        aclRet = acldvppMalloc(&inputBuff, binFileBufferLen);
        if (aclRet !=  ACL_ERROR_NONE) {
            ERROR_LOG("device malloc binFileBufferData failed, errorCode = %d", static_cast<int32_t>(aclRet));
            binFile.close();
            return FAILED;
        }
    }
    binFile.read(static_cast<char *>(inputBuff), binFileBufferLen);
    binFile.close();
    fileSize = binFileBufferLen;

    return SUCCESS;
}

Result Utils::GetPicDevBuffer4JpegD(PicDesc &picDesc, char *&picDevBuffer, uint32_t &devPicBufferSize)
{
    if (picDesc.picName.empty()) {
        ERROR_LOG("picture file name is empty");
        return FAILED;
    }

    uint32_t inputBuffSize = 0;
    void *inputBuff = nullptr;
    auto ret = ReadBinFile(picDesc.picName, inputBuff, inputBuffSize);
    if (ret != SUCCESS) {
        ERROR_LOG("read bin file failed, file name is %s", picDesc.picName.c_str());
        return FAILED;
    }
    aclError aclRet = acldvppJpegGetImageInfo(inputBuff, inputBuffSize, &picDesc.width, &picDesc.height, nullptr);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("get jpeg image info failed, errorCode = %d", static_cast<int32_t>(aclRet));
        if (!(RunStatus::GetDeviceStatus())) {
            (void)aclrtFreeHost(inputBuff);
        } else {
            (void)acldvppFree(inputBuff);
        }
        return FAILED;
    }
    aclRet = acldvppJpegPredictDecSize(inputBuff, inputBuffSize, PIXEL_FORMAT_YUV_SEMIPLANAR_420,
        &picDesc.jpegDecodeSize);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("get jpeg decode size failed, errorCode = %d", static_cast<int32_t>(aclRet));
        if (!(RunStatus::GetDeviceStatus())) {
            (void)aclrtFreeHost(inputBuff);
        } else {
            (void)acldvppFree(inputBuff);
        }
        return FAILED;
    }

    void *inBufferDev = nullptr;
    uint32_t inBufferSize = inputBuffSize;
    if (!(RunStatus::GetDeviceStatus())) { // app is running in host
        aclRet = acldvppMalloc(&inBufferDev, inBufferSize);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("malloc inBufferSize failed, errorCode = %d", static_cast<int32_t>(aclRet));
            (void)aclrtFreeHost(inputBuff);
            return FAILED;
        }

        // if app is running in host, need copy data from host to device
        aclRet = aclrtMemcpy(inBufferDev, inBufferSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("memcpy from host to device failed, errorCode = %d", static_cast<int32_t>(aclRet));
            (void)acldvppFree(inBufferDev);
            (void)aclrtFreeHost(inputBuff);
            return FAILED;
        }
        (void)aclrtFreeHost(inputBuff);
    } else { // app is running in device
        inBufferDev = inputBuff;
    }
    devPicBufferSize = inBufferSize;
    picDevBuffer = reinterpret_cast<char *>(inBufferDev);

    return SUCCESS;
}

void *Utils::GetPicDevBuffer(const PicDesc &picDesc, uint32_t &picBufferSize)
{
    if (picDesc.picName.empty()) {
        ERROR_LOG("picture file name is empty");
        return nullptr;
    }

    FILE *fp = fopen(picDesc.picName.c_str(), "rb");
    if (fp == nullptr) {
        ERROR_LOG("open file %s failed", picDesc.picName.c_str());
        return nullptr;
    }

    fseek(fp, 0, SEEK_END);
    long fileLen = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (static_cast<uint32_t>(fileLen) < picBufferSize) {
        ERROR_LOG("need read %u bytes but file %s only %ld bytes",
            picBufferSize, picDesc.picName.c_str(), fileLen);
        fclose(fp);
        return nullptr;
    }

    void *inputDevBuff = nullptr;
    aclError aclRet = acldvppMalloc(&inputDevBuff, picBufferSize);
    if (aclRet !=  ACL_ERROR_NONE) {
        ERROR_LOG("malloc device data buffer failed, errorCode = %d", static_cast<int32_t>(aclRet));
        fclose(fp);
        return nullptr;
    }

    void *inputBuff = nullptr;
    size_t readSize;
    if (!(RunStatus::GetDeviceStatus())) { // app is running in host
        aclRet = aclrtMallocHost(&inputBuff, picBufferSize);
        if (aclRet !=  ACL_ERROR_NONE) {
            ERROR_LOG("malloc host data buffer failed, errorCode = %d", static_cast<int32_t>(aclRet));
            fclose(fp);
            (void)acldvppFree(inputDevBuff);
            return nullptr;
        }

        readSize = fread(inputBuff, sizeof(char), picBufferSize, fp);
        if (readSize < picBufferSize) {
            ERROR_LOG("need read file %s %u bytes, but only %zu readed",
                picDesc.picName.c_str(), picBufferSize, readSize);
            (void)aclrtFreeHost(inputBuff);
            (void)acldvppFree(inputDevBuff);
            fclose(fp);
            return nullptr;
        }

        // if app is running in host, need copy model output data from host to device
        aclRet = aclrtMemcpy(inputDevBuff, picBufferSize, inputBuff, picBufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("memcpy from host to device failed, errorCode = %d", static_cast<int32_t>(aclRet));
            (void)acldvppFree(inputDevBuff);
            (void)aclrtFreeHost(inputBuff);
            fclose(fp);
            return nullptr;
        }
        (void)aclrtFreeHost(inputBuff);
    } else { // app is running in device
        readSize = fread(inputDevBuff, sizeof(char), picBufferSize, fp);
        if (readSize < picBufferSize) {
            ERROR_LOG("need read file %s %u bytes, but only %zu readed",
                picDesc.picName.c_str(), picBufferSize, readSize);
            (void)acldvppFree(inputDevBuff);
            fclose(fp);
            return nullptr;
        }
    }

    fclose(fp);
    return inputDevBuff;
}

Result Utils::PullModelOutputData(aclmdlDataset *modelOutput, const char *fileName)
{
    size_t outDatasetNum = aclmdlGetDatasetNumBuffers(modelOutput);
    if (outDatasetNum == 0) {
        ERROR_LOG("model out dataset num can't be 0");
    }
    for (size_t i = 0; i < outDatasetNum; ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(modelOutput, i);
        if (dataBuffer == nullptr) {
            ERROR_LOG("aclmdlGetDatasetBuffer failed");
            return FAILED;
        }

        void *dataBufferDev = aclGetDataBufferAddr(dataBuffer);
        if (dataBufferDev == nullptr) {
            ERROR_LOG("aclGetDataBufferAddr failed");
            return FAILED;
        }

        uint32_t bufferSize = aclGetDataBufferSize(dataBuffer);
        void *dataPtr = nullptr;
        aclError aclRet;
        if (!(RunStatus::GetDeviceStatus())) {
            aclRet = aclrtMallocHost(&dataPtr, bufferSize);
            if (aclRet !=  ACL_ERROR_NONE) {
                ERROR_LOG("malloc host data buffer failed, errorCode = %d", static_cast<int32_t>(aclRet));
                return FAILED;
            }

            aclRet = aclrtMemcpy(dataPtr, bufferSize, dataBufferDev, bufferSize, ACL_MEMCPY_DEVICE_TO_HOST);
            if (aclRet != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMemcpy device to host failed, errorCode = %d", static_cast<int32_t>(aclRet));
                (void)aclrtFreeHost(dataPtr);
            }
        } else {
            dataPtr = dataBufferDev;
        }

        uint32_t len = static_cast<uint32_t>(bufferSize);
        FILE *outputFile = fopen(fileName, "wb+");
        if (outputFile != nullptr) {
            fwrite(static_cast<char *>(dataPtr), len, sizeof(char), outputFile);
            fclose(outputFile);
            if (!(RunStatus::GetDeviceStatus())) {
                (void)aclrtFreeHost(dataPtr);
            }
        } else {
            ERROR_LOG("create output file %s failed, size is %u", fileName, len);
            if (!(RunStatus::GetDeviceStatus())) {
                (void)aclrtFreeHost(dataPtr);
            }
            return FAILED;
        }
    }
    return SUCCESS;
}

Result Utils::SaveDvppOutputData(const char *fileName, void *devPtr, uint32_t dataSize)
{
    void *dataPtr = nullptr;
    aclError aclRet;
    if (!(RunStatus::GetDeviceStatus())) {
        aclRet = aclrtMallocHost(&dataPtr, dataSize);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("malloc host data buffer failed, errorCode = %d", static_cast<int32_t>(aclRet));
            return FAILED;
        }

        aclRet = aclrtMemcpy(dataPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("dvpp output memcpy to host failed, errorCode = %d", static_cast<int32_t>(aclRet));
            (void)aclrtFreeHost(dataPtr);
            return FAILED;
        }
    } else {
        dataPtr = devPtr;
    }

    FILE *outFileFp = fopen(fileName, "wb+");
    if (outFileFp == nullptr) {
        ERROR_LOG("fopen out file %s failed.", fileName);
        if (!(RunStatus::GetDeviceStatus())) {
            (void)aclrtFreeHost(dataPtr);
        }
        return FAILED;
    }

    size_t writeSize = fwrite(dataPtr, sizeof(char), dataSize, outFileFp);
    if (writeSize != dataSize) {
        ERROR_LOG("need write %u bytes to %s, but only write %zu bytes.",
            dataSize, fileName, writeSize);
        fclose(outFileFp);
        if (!(RunStatus::GetDeviceStatus())) {
            (void)aclrtFreeHost(dataPtr);
        }
        return FAILED;
    }

    if (!(RunStatus::GetDeviceStatus())) {
        (void)aclrtFreeHost(dataPtr);
    }
    fflush(outFileFp);
    fclose(outFileFp);
    return SUCCESS;
}

Result Utils::CheckFile(const char *fileName)
{
    int i = 0;
    while (i < 10) {
        std::ifstream f (fileName);
        if (f.good()) {
            break;
        }
        SleepTime(1); // slepp 1s
        INFO_LOG("check result, wait time %d second", i + 1);
        i++;
    }
    // 10 is max time of checking
    if (i == 10) {
        ERROR_LOG("check result failed, timeout, expect file:%s", fileName);
        return FAILED;
    }
    return SUCCESS;
}

Result Utils::SaveModelOutputData(const char *srcfileName, const char *dstfileName)
{
    Result ret = CheckFile(srcfileName);
    if (ret != SUCCESS) {
        ERROR_LOG("model output file not exist");
        return FAILED;
    }
    FILE *model_output = fopen(srcfileName, "rb");
    if (model_output == nullptr) {
        ERROR_LOG("fopen out file %s failed.", srcfileName);
        return FAILED;
    }

    FILE *model_output_txt = fopen(dstfileName, "wb+");
    if (model_output_txt == nullptr) {
        ERROR_LOG("fopen out file %s failed.", dstfileName);
        fclose(model_output);
        return FAILED;
    }

    int i = 0;
    float prop = 0.0;
    std::map<float, int, std::greater<float>> mp;
    std::map<float, int>::iterator ite;
    while (feof(model_output) == 0) {
        ite = mp.end();
        fread(&prop, sizeof(float), 1, model_output);
        mp.insert(ite, std::map<float, int>::value_type(prop, i));
        fprintf(model_output_txt, "%f,%d\n", prop, i);
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
    INFO_LOG("result : classType[%d], top1[%f], top5[%f]", classType, max, sum);
    INFO_LOG("-------------------------------------------");
    return SUCCESS;
}

Result Utils::CheckAndCreateFolder(const char* foldName)
{
    INFO_LOG("start check result fold:%s", foldName);
#if defined(_MSC_VER)
    DWORD ret = GetFileAttributes((LPCSTR)foldName);
    if (ret == INVALID_FILE_ATTRIBUTES) {
        BOOL flag = CreateDirectory((LPCSTR)foldName, nullptr);
        if (flag) {
            INFO_LOG("make directory successfully.");
        } else {
            INFO_LOG("make directory errorly.");
            return FAILED;
        }
    }
#else
    if (access(foldName , 0) == -1) {
        int flag = mkdir(foldName , 0777);
        if (flag == 0) {
            INFO_LOG("make directory successfully.");
        } else {
            ERROR_LOG("make directory errorly.");
            return FAILED;
        }
    }
#endif
    INFO_LOG("check result success, fold exist");
    return SUCCESS;
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
