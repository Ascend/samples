/**
* @file Main.cpp
*
* Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "acl/acl.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <sys/stat.h>
#include <map>
#include <sstream>

using namespace std;
uint32_t modelId_;
aclmdlDesc *modelDesc_;
aclmdlDataset *input_;
aclmdlDataset *output_;
aclDataBuffer* inputBuffer;
aclDataBuffer* batchBuffer;
aclDataBuffer* outputBuffer;
aclrtRunMode runMode;
int32_t deviceId_ = 0;

void* pictureData = nullptr;
void* pictureDeviceData = nullptr;

uint32_t pictureDataSize = 0;
uint32_t totalDataSize = 0;

void* outputHostData = nullptr;
void* outputDeviceData = nullptr;
size_t outputDataSize = 0;

void InitResource()
{
    aclError ret = aclInit(nullptr);
    ret = aclrtSetDevice(deviceId_);
    aclrtGetRunMode(&runMode);
}

void ReadPicture(const string &picturePath)
{
    ifstream binFile(picturePath, ifstream::binary);
    binFile.seekg(0, binFile.end);
    pictureDataSize = binFile.tellg();
    binFile.seekg(0, binFile.beg);
    if (runMode == ACL_HOST) {
        aclError ret = aclrtMallocHost(&pictureData, pictureDataSize);
    }
    else{
        aclError ret = aclrtMalloc(&pictureData, pictureDataSize, ACL_MEM_MALLOC_HUGE_FIRST);
    }
    binFile.read(static_cast<char *>(pictureData), pictureDataSize);
    binFile.close();
}

void CopyData(int fileCount, uint32_t pos)
{
    aclError ret;
    if (pictureDeviceData == nullptr)
    {
        ret = aclrtMalloc(&pictureDeviceData, pictureDataSize * fileCount, ACL_MEM_MALLOC_HUGE_FIRST);
    }
    if (runMode == ACL_HOST) {
        ret = aclrtMemcpy((char *)pictureDeviceData + pos, pictureDataSize, \
        pictureData, pictureDataSize, ACL_MEMCPY_HOST_TO_DEVICE);
    }
    else
    {
        ret = aclrtMemcpy((char *)pictureDeviceData + pos, pictureDataSize, \
        pictureData, pictureDataSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }
}

void CreateModelInput()
{
    aclError ret;
    modelDesc_ = aclmdlCreateDesc();
    ret = aclmdlGetDesc(modelDesc_, modelId_);
    input_ = aclmdlCreateDataset();
    for(size_t index = 0; index < aclmdlGetNumInputs(modelDesc_); ++index)
    {
        const char* name = aclmdlGetInputNameByIndex(modelDesc_, index);
        size_t inputLen = aclmdlGetInputSizeByIndex(modelDesc_, index);
        if(strcmp(name, ACL_DYNAMIC_TENSOR_NAME) == 0)
        {
            void *data = nullptr;
            ret = aclrtMalloc(&data, inputLen, ACL_MEM_MALLOC_HUGE_FIRST);
            batchBuffer = aclCreateDataBuffer(data, inputLen);
            ret = aclmdlAddDatasetBuffer(input_, batchBuffer);
        }
        else
        {
            inputBuffer = aclCreateDataBuffer(pictureDeviceData, totalDataSize);
            ret = aclmdlAddDatasetBuffer(input_, inputBuffer);
        }
    }

}

void CreateModelOutput()
{
    aclError ret;
    output_ = aclmdlCreateDataset();
    outputDataSize = aclmdlGetOutputSizeByIndex(modelDesc_, 0);
    ret = aclrtMalloc(&outputDeviceData, outputDataSize, ACL_MEM_MALLOC_NORMAL_ONLY);
    outputBuffer = aclCreateDataBuffer(outputDeviceData, outputDataSize);
    ret = aclmdlAddDatasetBuffer(output_, outputBuffer);
}

void LoadPicture(int fileCount)
{
    uint32_t pos = 0;
    for(int i = 0;i < fileCount;i++)
    {
        string fileName = "../data/dog" + (to_string(i+1)) + "_1024_683.bin";
        ReadPicture(fileName);
        CopyData(fileCount, pos);
        pos += pictureDataSize;
    }
    totalDataSize = pictureDataSize * fileCount;
    CreateModelInput();
    CreateModelOutput();
}

void LoadModel(const char *modelPath)
{
    aclError ret = aclmdlLoadFromFile(modelPath, &modelId_);
}

void Inference(int batchSize)
{
    size_t index;
    aclError ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME,&index);
    ret = aclmdlSetDynamicBatchSize(modelId_, input_, index, batchSize);
    ret = aclmdlExecute(modelId_, input_, output_);
}


void PrintResult(int batchSize)
{
    float* outFloatData;
    if (runMode == ACL_HOST) {
        aclError ret = aclrtMallocHost(&outputHostData, outputDataSize);
        ret = aclrtMemcpy(outputHostData, outputDataSize, outputDeviceData, outputDataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        outFloatData = reinterpret_cast < float * > (outputHostData);
    }
    else
    {
        outFloatData = reinterpret_cast < float * > (outputDeviceData);
    }
    for(int i = 0;i < batchSize;i++)
    {
        map<float, unsigned int, greater<float>> resultMap;
        for (unsigned int j = 0; j < outputDataSize / (sizeof(float) * batchSize); ++j)
        {
            resultMap[*outFloatData] = j;
            outFloatData++;
        }
        printf("=================Result of picture %d=================\n", (i+1));
        int cnt = 0;
        for (auto it = resultMap.begin(); it != resultMap.end(); ++it)
        {
            if(++cnt > 5)
            {
                break;
            }

            printf("top %d: index[%d] value[%lf] \n", cnt, it->second, it->first);
        }
    }

}

void UnloadModel()
{
    aclmdlDestroyDesc(modelDesc_);
    aclmdlUnload(modelId_);
}

void UnloadPicture()
{
    if (runMode == ACL_HOST) {
        aclrtFreeHost(pictureData);
        pictureData = nullptr;
        aclrtFreeHost(outputHostData);
        outputHostData = nullptr;
    }
    else
    {
        aclrtFree(pictureData);
        pictureData = nullptr;
    }
    aclrtFree(pictureDeviceData);
    pictureDeviceData = nullptr;
    aclDestroyDataBuffer(inputBuffer);
    inputBuffer = nullptr;
    aclmdlDestroyDataset(input_);
    input_ = nullptr;

    aclrtFree(outputDeviceData);
    outputDeviceData = nullptr;
    aclDestroyDataBuffer(outputBuffer);
    outputBuffer = nullptr;
    aclmdlDestroyDataset(output_);
    output_ = nullptr;
}

void DestroyResource()
{
    aclError ret = aclrtResetDevice(deviceId_);
    ret = aclFinalize();
}


int main()
{
    int batchSize = 2;
    char *modelPath = (char *)"../model/googlenet_dynamicbatch.om";
    InitResource();
    LoadModel(modelPath);
    LoadPicture(batchSize);
    Inference(batchSize);
    PrintResult(batchSize);
    UnloadModel();
    UnloadPicture();
    DestroyResource();
}