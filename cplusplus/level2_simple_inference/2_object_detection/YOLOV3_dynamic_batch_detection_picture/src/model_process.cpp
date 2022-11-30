/**
* @file model_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "model_process.h"
#include <map>
#include <sstream>
#include <cstring>
#include <fstream>
#include <algorithm>
#include <functional>
#include "acl/acl_mdl.h"

using namespace std;
extern bool g_isDevice;

typedef struct DataInfo {
    void *data;
    size_t size;
} DataInfo;

ModelProcess::ModelProcess()
    : modelId_(0), modelWorkPtr_(nullptr), modelWorkSize_(0), modelWeightPtr_(nullptr), modelWeightSize_(0),
      loadFlag_(false), modelDesc_(nullptr), input_(nullptr), output_(nullptr)
{
}

ModelProcess::~ModelProcess()
{
    UnloadModel();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
}

Result ModelProcess::LoadModel(const char *modelPath)
{
    if (modelPath == nullptr) {
        ERROR_LOG("modelPath is empty.");
        return FAILED;
    }

    if (loadFlag_) {
        ERROR_LOG("model has already been loaded.");
        return FAILED;
    }

    aclError ret = aclmdlQuerySize(modelPath, &modelWorkSize_, &modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("query model failed, model file is %s, errorCode = %d.",
            modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    // using ACL_MEM_MALLOC_HUGE_FIRST to malloc memory, huge memory is preferred to use
    // and huge memory can improve performance.
    ret = aclrtMalloc(&modelWorkPtr_, modelWorkSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for work failed, require size is %zu, errorCode = %d.",
            modelWorkSize_, static_cast<int32_t>(ret));
        return FAILED;
    }

    // using ACL_MEM_MALLOC_HUGE_FIRST to malloc memory, huge memory is preferred to use
    // and huge memory can improve performance.
    ret = aclrtMalloc(&modelWeightPtr_, modelWeightSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for weight failed, require size is %zu, errorCode = %d.",
            modelWeightSize_, static_cast<int32_t>(ret));
        (void)aclrtFree(modelWorkPtr_);
        modelWorkPtr_ = nullptr;
        modelWorkSize_ = 0;
        return FAILED;
    }

    ret = aclmdlLoadFromFileWithMem(modelPath, &modelId_, modelWorkPtr_,
        modelWorkSize_, modelWeightPtr_, modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("load model from file failed, model file is %s, errorCode = %d.",
            modelPath,  static_cast<int32_t>(ret));

        (void)aclrtFree(modelWorkPtr_);
        modelWorkPtr_ = nullptr;
        modelWorkSize_ = 0;

        (void)aclrtFree(modelWeightPtr_);
        modelWeightPtr_ = nullptr;
        modelWeightSize_ = 0;

        return FAILED;
    }

    loadFlag_ = true;
    INFO_LOG("load model %s success.", modelPath);

    return SUCCESS;
}

void ModelProcess::UnloadModel()
{
    if (!loadFlag_) {
        WARN_LOG("no model had been loaded.");
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("unload model failed, modelId is %u, errorCode = %d.",
            modelId_, static_cast<int32_t>(ret));
    }

    if (modelWorkPtr_ != nullptr) {
        (void)aclrtFree(modelWorkPtr_);
        modelWorkPtr_ = nullptr;
        modelWorkSize_ = 0;
    }

    if (modelWeightPtr_ != nullptr) {
        (void)aclrtFree(modelWeightPtr_);
        modelWeightPtr_ = nullptr;
        modelWeightSize_ = 0;
    }

    loadFlag_ = false;
    INFO_LOG("unload model success, modelId is %u.", modelId_);
}

Result ModelProcess::CreateDesc()
{
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed.");
        return FAILED;
    }

    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get model description failed, modelId_ = %u, errorCode = %d.",
            modelId_, static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("create model description success.");

    return SUCCESS;
}

void ModelProcess::DestroyDesc()
{
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
    INFO_LOG("destroy model description success.");
}

Result ModelProcess::CreateInput(const ImageMemoryInfo &imageMemInfo)
{
    if ((imageMemInfo.imageDataBuf == nullptr) || (imageMemInfo.imageInfoBuf == nullptr)) {
        ERROR_LOG("input image memory is nullptr.");
        return FAILED;
    }

    const size_t mdlInputNum = aclmdlGetNumInputs(modelDesc_);
    // dynamic batch yolov3 has three inputs, two are data input, one is dynamic input
    if (mdlInputNum != 3) {
        ERROR_LOG("the input number of dynamic batch yolov3 must be 3.");
        return FAILED;
    }
    size_t dynamicIdx = 0;
    auto ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME, &dynamicIdx);
    if ((ret != ACL_SUCCESS) || (dynamicIdx != (mdlInputNum - 1))) {
        ERROR_LOG("get input index by name failed, dynamicIdx = %zu, errorCode = %d.",
            dynamicIdx, static_cast<int32_t>(ret));
        return FAILED;
    }
    size_t dataLen = aclmdlGetInputSizeByIndex(modelDesc_, dynamicIdx);
    void *data = nullptr;
    ret = aclrtMalloc(&data, dataLen, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc device memory failed, errorCode = %d.", static_cast<int32_t>(ret));
        return FAILED;
    }

    // the first two inputs of yolov3 model are data input, the third input is dynamic input
    DataInfo inputDataInfo[mdlInputNum] = {{imageMemInfo.imageDataBuf, imageMemInfo.imageDataSize},
        {imageMemInfo.imageInfoBuf, imageMemInfo.imageInfoSize},
        {data, dataLen}};

    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ERROR_LOG("can't create dataset, create input failed.");
        aclrtFree(data);
        data = nullptr;
        return FAILED;
    }

    for (size_t index = 0; index < mdlInputNum; ++index) {
        aclDataBuffer *inputData = aclCreateDataBuffer(inputDataInfo[index].data, inputDataInfo[index].size);
        if (inputData == nullptr) {
            ERROR_LOG("can't create data buffer, create input failed.");
            (void)aclrtFree(inputDataInfo[index].data);
            inputDataInfo[index].data = nullptr;
            return FAILED;
        }

        aclError ret = aclmdlAddDatasetBuffer(input_, inputData);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("add input dataset buffer failed, errorCode = %d.", static_cast<int32_t>(ret));
            aclDestroyDataBuffer(inputData);
            inputData = nullptr;
            (void)aclrtFree(inputDataInfo[index].data);
            inputDataInfo[index].data = nullptr;
            return FAILED;
        }
    }
    INFO_LOG("create model input success.");

    return SUCCESS;
}

void ModelProcess::DestroyInput()
{
    if (input_ == nullptr) {
        return;
    }

    size_t bufNum = aclmdlGetDatasetNumBuffers(input_);
    for (size_t i = 0; i < bufNum; ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(input_, i);
        if (dataBuffer == nullptr){
            continue;
        }
        void *data = aclGetDataBufferAddr(dataBuffer);
        if (data == nullptr){
            (void)aclDestroyDataBuffer(dataBuffer);
            continue;
        }
        (void)aclrtFree(data);
        data = nullptr;
        (void)aclDestroyDataBuffer(dataBuffer);
        dataBuffer = nullptr;
    }
    (void)aclmdlDestroyDataset(input_);
    input_ = nullptr;
    INFO_LOG("destroy model input success.");
}

Result ModelProcess::CreateOutput()
{
    if (modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create ouput failed.");
        return FAILED;
    }

    output_ = aclmdlCreateDataset();
    if (output_ == nullptr) {
        ERROR_LOG("can't create dataset, create output failed.");
        return FAILED;
    }

    size_t outputSize = aclmdlGetNumOutputs(modelDesc_);
    for (size_t i = 0; i < outputSize; ++i) {
        size_t buffer_size = aclmdlGetOutputSizeByIndex(modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't malloc buffer, size is %zu, create output failed, errorCode = %d.",
                buffer_size, static_cast<int32_t>(ret));
            return FAILED;
        }

        aclDataBuffer* outputData = aclCreateDataBuffer(outputBuffer, buffer_size);
        if (outputData == nullptr) {
            ERROR_LOG("can't create data buffer, create output failed.");
            (void)aclrtFree(outputBuffer);
            outputBuffer = nullptr;
            return FAILED;
        }

        ret = aclmdlAddDatasetBuffer(output_, outputData);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't add data buffer, create output failed, errorCode = %d.", static_cast<int32_t>(ret));
            (void)aclrtFree(outputBuffer);
            outputBuffer = nullptr;
            (void)aclDestroyDataBuffer(outputData);
            outputData = nullptr;
            return FAILED;
        }
    }

    INFO_LOG("create model output success.");
    return SUCCESS;
}

void ModelProcess::DestroyOutput()
{
    if (output_ == nullptr) {
        return;
    }

    size_t outputNum = aclmdlGetDatasetNumBuffers(output_);
    for (size_t i = 0; i < outputNum; ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output_, i);
        if (dataBuffer == nullptr){
            continue;
        }

        void* data = aclGetDataBufferAddr(dataBuffer);
        if (data == nullptr){
            (void)aclDestroyDataBuffer(dataBuffer);
            continue;
        }

        (void)aclrtFree(data);
        data = nullptr;
        (void)aclDestroyDataBuffer(dataBuffer);
        dataBuffer = nullptr;
    }

    (void)aclmdlDestroyDataset(output_);
    output_ = nullptr;
    INFO_LOG("destroy model output success.");
}

Result ModelProcess::SetDynamicBatchSize(uint64_t batchSize)
{
    size_t index;
    aclError ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME, &index);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get input index by name[%s] failed, errorCode = %d.",
            ACL_DYNAMIC_TENSOR_NAME, static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclmdlSetDynamicBatchSize(modelId_, input_, index, batchSize);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("set dynamic batch size[%lu] failed, errorCode = %d.",
            batchSize, static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

Result ModelProcess::SetDynamicHWSize(uint64_t height, uint64_t width)
{
    size_t index;
    aclError ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME, &index);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get input index by name[%s] failed, errorCode = %d.",
            ACL_DYNAMIC_TENSOR_NAME, static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclmdlSetDynamicHWSize(modelId_, input_, index, height, width);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("set dynamic hw[%lu, %lu] failed, errorCode = %d.",
            height, width, static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

Result ModelProcess::SetDynamicSize(const DynamicInfo &dynamicInfo)
{
    Result ret = SUCCESS;
    DynamicType dynamicType = dynamicInfo.dynamicType;
    if (dynamicType == DYNAMIC_BATCH) {
        ret = SetDynamicBatchSize(dynamicInfo.dynamicArr[0]);
        if (ret != SUCCESS) {
            return FAILED;
        }
        INFO_LOG("set dynamic batch size[%lu] success.", dynamicInfo.dynamicArr[0]);
    } else if (dynamicType == DYNAMIC_HW) {
        ret = SetDynamicHWSize(dynamicInfo.dynamicArr[0], dynamicInfo.dynamicArr[1]);
        if (ret != SUCCESS) {
            return FAILED;
        }
        INFO_LOG("set dynamic hw[%lu, %lu] success.", dynamicInfo.dynamicArr[0], dynamicInfo.dynamicArr[1]);
    } else {
        ERROR_LOG("invalid dynamic type %d.", static_cast<int32_t>(dynamicType));
        return FAILED;
    }

    return SUCCESS;
}

Result ModelProcess::Execute()
{
    aclError ret = aclmdlExecute(modelId_, input_, output_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, modelId is %u, errorCode = %d.",
            modelId_, static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("model execute success.");
    return SUCCESS;
}

void ModelProcess::DumpModelOutputResult()
{
    if (output_ == nullptr) {
        ERROR_LOG("output is empty.");
        return;
    }

    size_t outputNum = aclmdlGetDatasetNumBuffers(output_);
    static int executeNum = 0;

    for (size_t i = 0; i < outputNum; ++i) {
        stringstream ss;
        ss << "output" << ++executeNum << "_" << i << ".bin";
        string outputFileName = ss.str();
        FILE *outputFile = fopen(outputFileName.c_str(), "wb");
        if (outputFile != nullptr) {
            aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output_, i);
            if (dataBuffer == nullptr){
                ERROR_LOG("output data buffer is empty! The seq is %zu.", i);
                fclose(outputFile);
                continue;
            }

            void* data = aclGetDataBufferAddr(dataBuffer);
            if (data == nullptr){
                ERROR_LOG("output data is empty! The seq is %zu.", i);
                fclose(outputFile);
                continue;
            }

            size_t len = aclGetDataBufferSizeV2(dataBuffer);

            void* outHostData = nullptr;
            aclError ret = ACL_SUCCESS;
            if (!g_isDevice) {
                ret = aclrtMallocHost(&outHostData, len);
                if (ret != ACL_SUCCESS) {
                    ERROR_LOG("aclrtMallocHost failed, errorCode = %d.", static_cast<int32_t>(ret));
                    fclose(outputFile);
                    outputFile = nullptr;
                    return;
                }

                ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
                if (ret != ACL_SUCCESS) {
                    ERROR_LOG("aclrtMemcpy failed, errorCode = %d.", static_cast<int32_t>(ret));
                    (void)aclrtFreeHost(outHostData);
                    outHostData = nullptr;
                    fclose(outputFile);
                    outputFile = nullptr;
                    return;
                }
                fwrite(outHostData, len, sizeof(char), outputFile);
                ret = aclrtFreeHost(outHostData);
                outHostData = nullptr;
                if (ret != ACL_SUCCESS) {
                    ERROR_LOG("aclrtFreeHost failed, errorCode = %d.", static_cast<int32_t>(ret));
                    fclose(outputFile);
                    outputFile = nullptr;
                    return;
                }
            } else {
                fwrite(data, len, sizeof(char), outputFile);
            }
            fclose(outputFile);
            outputFile = nullptr;
        } else {
            ERROR_LOG("create output file [%s] failed.", outputFileName.c_str());
            return;
        }
    }
    INFO_LOG("dump data success.");
    return;
}

void ModelProcess::PrintModelDescInfo(DynamicType dynamicType)
{
    if (modelDesc_ == nullptr) {
        ERROR_LOG("modelDesc_ is empty.");
        return;
    }

    size_t inputNum = aclmdlGetNumInputs(modelDesc_);
    size_t outputNum = aclmdlGetNumOutputs(modelDesc_);
    INFO_LOG("model input num[%zu], output num[%zu].", inputNum, outputNum);
    INFO_LOG("start to print input tensor desc:");

    for (size_t i = 0; i < inputNum; ++i) {
        size_t inputSize = aclmdlGetInputSizeByIndex(modelDesc_, i);
        const char* name = aclmdlGetInputNameByIndex(modelDesc_, i);
        if (name == nullptr){
            ERROR_LOG("fail to get input name, index[%zu].", i);
            continue;
        }
        aclFormat format = aclmdlGetInputFormat(modelDesc_, i);
        aclDataType dataType = aclmdlGetInputDataType(modelDesc_, i);
        INFO_LOG("index[%zu]: name[%s], inputSize[%zu], fotmat[%d], dataType[%d]",
            i, name, inputSize,static_cast<int>(format), static_cast<int>(dataType));

        aclmdlIODims ioDims;
        auto ret = aclmdlGetInputDims(modelDesc_, i, &ioDims);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("fail to get input tendor dims, index[%zu], errorCode = %d.",
                i, static_cast<int32_t>(ret));
            return;
        }

        stringstream ss;
        ss << "dimcount:[" << ioDims.dimCount << "],dims:";
        for(size_t j = 0; j < ioDims.dimCount; ++j) {
            ss << "[" << ioDims.dims[j] << "]";
        }

        INFO_LOG("%s", ss.str().c_str());
    }

    INFO_LOG("start to print output tensor desc:");
    for (size_t i = 0; i < outputNum; ++i) {
        size_t outputSize = aclmdlGetOutputSizeByIndex(modelDesc_, i);
        const char* name = aclmdlGetOutputNameByIndex(modelDesc_, i);
        if (name == nullptr){
            ERROR_LOG("fail to get output name, index[%zu].", i);
            continue;
        }

        aclFormat format = aclmdlGetOutputFormat(modelDesc_, i);
        aclDataType dataType = aclmdlGetOutputDataType(modelDesc_, i);
        INFO_LOG("index[%zu]: name[%s], outputSize[%zu], fotmat[%d], dataType[%d]",
            i, name, outputSize, static_cast<int>(format), static_cast<int>(dataType));

        aclmdlIODims ioDims;
        auto ret = aclmdlGetOutputDims(modelDesc_, i, &ioDims);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("fail to get output tendor dims, index[%zu], errorCode = %d.",
                i, static_cast<int32_t>(ret));
            return;
        }

        stringstream ss;
        ss << "dimcount:[" << ioDims.dimCount << "],dims:";
        for(size_t j = 0; j < ioDims.dimCount; ++j) {
            ss << "[" << ioDims.dims[j] << "]";
        }

        INFO_LOG("%s", ss.str().c_str());
    }

    if (dynamicType == DYNAMIC_BATCH) {
        PrintDynamicBatchInfo();
    } else {
        PrintDynamicHWInfo();
    }
}

void ModelProcess::PrintDynamicBatchInfo()
{
    if (modelDesc_ == nullptr) {
        ERROR_LOG("modelDesc_ is empty.");
        return;
    }

    INFO_LOG("start to print model dynamic batch info:");
    aclmdlBatch batchInfo;
    auto ret = aclmdlGetDynamicBatch(modelDesc_, &batchInfo);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to get dynamic batch info, errorCode = %d", static_cast<int32_t>(ret));
        return;
    }

    stringstream ss;
    ss << "dynamic batch count:[" << batchInfo.batchCount << "],dims:{";
    for(size_t i = 0; i < batchInfo.batchCount; ++i) {
        ss << "[" << batchInfo.batch[i] << "]";
    }
    ss << "}";
    INFO_LOG("%s", ss.str().c_str());
}

void ModelProcess::PrintDynamicHWInfo()
{
    if (modelDesc_ == nullptr) {
        ERROR_LOG("modelDesc_ is empty.");
        return;
    }

    INFO_LOG("start to print model dynamic hw info:");
    aclmdlHW hwInfo;
    auto ret = aclmdlGetDynamicHW(modelDesc_, -1, &hwInfo);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to get dynamic hw info, errorCode = %d", static_cast<int32_t>(ret));
        return;
    }

    stringstream ss;
    ss << "dynamic hw count:[" << hwInfo.hwCount << "],dims:{";
    for(size_t i = 0; i < hwInfo.hwCount; ++i) {
        ss << "[" << hwInfo.hw[i][0];
        ss << ", " << hwInfo.hw[i][1] << "]";
    }
    ss << "}";
    INFO_LOG("%s", ss.str().c_str());
}

void ModelProcess::PrintModelCurOutputDims()
{
    if (modelDesc_ == nullptr) {
        ERROR_LOG("modelDesc_ is empty.");
        return;
    }

    INFO_LOG("start to print model current output shape info:");
    aclmdlIODims ioDims;
    for (size_t i = 0; i < aclmdlGetNumOutputs(modelDesc_); ++i) {
        auto ret = aclmdlGetCurOutputDims(modelDesc_, i, &ioDims);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("fail to get cur output dims, index[%zu], errorCode = %d.",
                i, static_cast<int32_t>(ret));
            return;
        }

        stringstream ss;
        ss << "index:" << i << ",dims:";
        for(size_t j = 0; j < ioDims.dimCount; ++j) {
            ss << "[" << ioDims.dims[j] << "]";
        }
        INFO_LOG("%s", ss.str().c_str());
    }
}

