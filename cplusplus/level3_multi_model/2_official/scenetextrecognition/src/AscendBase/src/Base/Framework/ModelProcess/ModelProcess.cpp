/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ModelProcess.h"
#include "FileManager/FileManager.h"

ModelProcess::ModelProcess() {}

ModelProcess::~ModelProcess()
{
    if (!isDeInit_) {
        deinit();
    }
}

void ModelProcess::DestroyDataset(aclmdlDataset *dataset)
{
    // Just release the DataBuffer object and DataSet object, remain the buffer, because it is managerd by user
    if (dataset != nullptr) {
        for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(dataset); i++) {
            aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(dataset, i);
            if (dataBuffer != nullptr) {
                aclDestroyDataBuffer(dataBuffer);
                dataBuffer = nullptr;
            }
        }
        aclmdlDestroyDataset(dataset);
    }
}

aclmdlDesc *ModelProcess::GetModelDesc()
{
    return modelDesc_.get();
}

int ModelProcess::ModelInference(std::vector<void *> &inputBufs, std::vector<size_t> &inputSizes,
    std::vector<void *> &ouputBufs, std::vector<size_t> &outputSizes, size_t dynamicBatchSize)
{
    LogDebug << "ModelProcess:Begin to inference.";
    aclmdlDataset *input = nullptr;
    input = CreateAndFillDataset(inputBufs, inputSizes);
    if (input == nullptr) {
        return APP_ERR_COMM_FAILURE;
    }
    APP_ERROR ret = 0;
    if (dynamicBatchSize != 0) {
        size_t index = 0;
        ret = aclmdlGetInputIndexByName(modelDesc_.get(), ACL_DYNAMIC_TENSOR_NAME, &index);
        if (ret != ACL_ERROR_NONE) {
            LogError << "aclmdlGetInputIndexByName failed, maybe static model";
            return APP_ERR_COMM_CONNECTION_FAILURE;
        }
        ret = aclmdlSetDynamicBatchSize(modelId_, input, index, dynamicBatchSize);
        if (ret != ACL_ERROR_NONE) {
            LogError << "dynamic batch set failed, modelId_=" << modelId_ << ", input=" << input << ", index=" << index
                     << ", dynamicBatchSize=" << dynamicBatchSize;
            return APP_ERR_COMM_CONNECTION_FAILURE;
        }
        LogDebug << "set dynamicBatchSize success, dynamicBatchSize=" << dynamicBatchSize;
    }
    aclmdlDataset *output = nullptr;
    output = CreateAndFillDataset(ouputBufs, outputSizes);
    if (output == nullptr) {
        DestroyDataset(input);
        input = nullptr;
        return APP_ERR_COMM_FAILURE;
    }
    mtx_.lock();
    ret = aclmdlExecute(modelId_, input, output);
    mtx_.unlock();
    if (ret != APP_ERR_OK) {
        LogError << "aclmdlExecute failed, ret[" << ret << "].";
        return ret;
    }

    DestroyDataset(input);
    DestroyDataset(output);
    return APP_ERR_OK;
}

int ModelProcess::ModelInferDynamicHW(const std::vector<void *> &inputBufs, const std::vector<size_t> &inputSizes,
                                      const std::vector<void *> &ouputBufs, const std::vector<size_t> &outputSizes)
{
    LogDebug << "ModelProcess:Begin to inference with dynamic width and height.";
    aclmdlDataset *input = nullptr;
    input = CreateAndFillDataset(inputBufs, inputSizes);
    if (input == nullptr) {
        return APP_ERR_COMM_FAILURE;
    }
    size_t index = 0;
    APP_ERROR ret = aclmdlGetInputIndexByName(modelDesc_.get(), ACL_DYNAMIC_TENSOR_NAME, &index);
    if (ret != ACL_ERROR_NONE) {
        LogError << "Failed to execute aclmdlGetInputIndexByName, maybe static model.";
        return APP_ERR_COMM_CONNECTION_FAILURE;
    }
    ret = aclmdlSetDynamicHWSize(modelId_, input, index, modelHeight_, modelWidth_);
    if (ret != ACL_ERROR_NONE) {
        LogError << "Failed to set dynamic HW, modelId_=" << modelId_ << ", input=" << input << ", index=" \
                 << index << ", dynamicW=" << modelWidth_ << ", dynamicH=" << modelHeight_;
        return APP_ERR_COMM_CONNECTION_FAILURE;
    }
    LogDebug << "Set dynamicHWSize success, dynamicHWSize=" << modelWidth_ << ", " << modelHeight_;

    aclmdlDataset *output = nullptr;
    output = CreateAndFillDataset(ouputBufs, outputSizes);
    if (output == nullptr) {
        DestroyDataset(input);
        input = nullptr;
        return APP_ERR_COMM_FAILURE;
    }
    mtx_.lock();
    ret = aclmdlExecute(modelId_, input, output);
    mtx_.unlock();
    if (ret != APP_ERR_OK) {
        LogError << "aclmdlExecute failed, ret[" << ret << "].";
        return ret;
    }

    DestroyDataset(input);
    DestroyDataset(output);
    return APP_ERR_OK;
}

int ModelProcess::deinit()
{
    LogInfo << "Model[" << modelName_ << "][" << deviceId_ << "] deinit begin";
    isDeInit_ = true;
    APP_ERROR ret = aclmdlUnload(modelId_);
    if (ret != APP_ERR_OK) {
        LogError << "aclmdlUnload  failed, ret[" << ret << "].";
        return ret;
    }

    if (modelDevPtr_ != nullptr) {
        ret = aclrtFree(modelDevPtr_);
        if (ret != APP_ERR_OK) {
            LogError << "aclrtFree  failed, ret[" << ret << "].";
            return ret;
        }
        modelDevPtr_ = nullptr;
    }
    if (weightDevPtr_ != nullptr) {
        ret = aclrtFree(weightDevPtr_);
        if (ret != APP_ERR_OK) {
            LogError << "aclrtFree  failed, ret[" << ret << "].";
            return ret;
        }
        weightDevPtr_ = nullptr;
    }
    for (size_t i = 0; i < inputBuffers_.size(); i++) {
        if (inputBuffers_[i] != nullptr) {
            aclrtFree(inputBuffers_[i]);
            inputBuffers_[i] = nullptr;
        }
    }

    for (size_t i = 0; i < outputBuffers_.size(); i++) {
        if (outputBuffers_[i] != nullptr) {
            aclrtFree(outputBuffers_[i]);
            outputBuffers_[i] = nullptr;
        }
    }
    inputSizes_.clear();
    outputSizes_.clear();
    LogInfo << "Model[" << modelName_ << "][" << deviceId_ << "] deinit success";
    return APP_ERR_OK;
}

APP_ERROR ModelProcess::init(std::string modelPath)
{
    LogInfo << "ModelProcess:Begin to init instance.";
    int modelSize = 0;
    std::shared_ptr<uint8_t> modelData = nullptr;
    APP_ERROR ret = ReadBinaryFile(modelPath, modelData, modelSize);
    if (ret != APP_ERR_OK) {
        LogError << "read model file failed, ret[" << ret << "].";
        return ret;
    }
    ret = aclmdlQuerySizeFromMem(modelData.get(), modelSize, &modelDevPtrSize_, &weightDevPtrSize_);
    if (ret != APP_ERR_OK) {
        LogError << "aclmdlQuerySizeFromMem failed, ret[" << ret << "].";
        return ret;
    }
    LogDebug << "modelDevPtrSize_[" << modelDevPtrSize_ << "], weightDevPtrSize_[" << weightDevPtrSize_ << "].";

    ret = aclrtMalloc(&modelDevPtr_, modelDevPtrSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != APP_ERR_OK) {
        LogError << "aclrtMalloc dev_ptr failed, ret[" << ret << "].";
        return ret;
    }
    ret = aclrtMalloc(&weightDevPtr_, weightDevPtrSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != APP_ERR_OK) {
        LogError << "aclrtMalloc weight_ptr failed, ret[" << ret << "] (" << get_app_errcodeinfo(ret) << ").";
        return ret;
    }
    ret = aclmdlLoadFromMemWithMem(modelData.get(), modelSize, &modelId_, modelDevPtr_, modelDevPtrSize_,
        weightDevPtr_, weightDevPtrSize_);
    if (ret != APP_ERR_OK) {
        LogError << "aclmdlLoadFromMemWithMem failed, ret[" << ret << "].";
        return ret;
    }
    ret = aclrtGetCurrentContext(&contextModel_);
    if (ret != APP_ERR_OK) {
        LogError << "aclrtMalloc weight_ptr failed, ret[" << ret << "].";
        return ret;
    }
    // get input and output size
    aclmdlDesc *modelDesc = aclmdlCreateDesc();
    if (modelDesc == nullptr) {
        LogError << "aclmdlCreateDesc failed.";
        return ret;
    }
    ret = aclmdlGetDesc(modelDesc, modelId_);
    if (ret != APP_ERR_OK) {
        LogError << "aclmdlGetDesc ret fail, ret:" << ret << ".";
        return ret;
    }
    modelDesc_.reset(modelDesc, aclmdlDestroyDesc);
    return APP_ERR_OK;
}

aclmdlDataset *ModelProcess::CreateAndFillDataset(const std::vector<void *> &bufs, const std::vector<size_t> &sizes)
{
    APP_ERROR ret = APP_ERR_OK;
    aclmdlDataset *dataset = aclmdlCreateDataset();
    if (dataset == nullptr) {
        LogError << "ACL_ModelInputCreate failed.";
        return nullptr;
    }

    for (size_t i = 0; i < bufs.size(); ++i) {
        aclDataBuffer *data = aclCreateDataBuffer(bufs[i], sizes[i]);
        if (data == nullptr) {
            DestroyDataset(dataset);
            LogError << "aclCreateDataBuffer failed.";
            return nullptr;
        }

        ret = aclmdlAddDatasetBuffer(dataset, data);
        if (ret != APP_ERR_OK) {
            DestroyDataset(dataset);
            LogError << "ACL_ModelInputDataAdd failed, ret[" << ret << "].";
            return nullptr;
        }
    }
    return dataset;
}

size_t ModelProcess::GetModelNumInputs()
{
    return aclmdlGetNumInputs(modelDesc_.get());
}

size_t ModelProcess::GetModelNumOutputs()
{
    return aclmdlGetNumOutputs(modelDesc_.get());
}

size_t ModelProcess::GetModelInputSizeByIndex(const size_t &i)
{
    return aclmdlGetInputSizeByIndex(modelDesc_.get(), i);
}

size_t ModelProcess::GetModelOutputSizeByIndex(const size_t &i)
{
    return aclmdlGetOutputSizeByIndex(modelDesc_.get(), i);
}

APP_ERROR ModelProcess::InputBufferWithSizeMalloc(aclrtMemMallocPolicy policy)
{
    size_t inputNum = aclmdlGetNumInputs(modelDesc_.get());
    LogDebug << modelName_ << "model inputNum is : " << inputNum << ".";
    for (size_t i = 0; i < inputNum; ++i) {
        void *buffer = nullptr;
        // modify size
        size_t size = aclmdlGetInputSizeByIndex(modelDesc_.get(), i);
        APP_ERROR ret = aclrtMalloc(&buffer, size, policy);
        if (ret != APP_ERR_OK) {
            LogFatal << modelName_ << "model input aclrtMalloc fail(ret=" << ret
                     << "), buffer=" << buffer << ", size=" << size << ".";
            // Free the buffer malloced successfully before return error
            ReleaseModelBuffer(inputBuffers_);
            return ret;
        }
        inputBuffers_.push_back(buffer);
        inputSizes_.push_back(size);
        LogDebug << modelName_ << "model inputBuffer i=" << i << ", size=" << size << ".";
    }
    return APP_ERR_OK;
}

APP_ERROR ModelProcess::OutputBufferWithSizeMalloc(aclrtMemMallocPolicy policy)
{
    size_t outputNum = aclmdlGetNumOutputs(modelDesc_.get());
    LogDebug << modelName_ << "model outputNum is : " << outputNum << ".";
    for (size_t i = 0; i < outputNum; ++i) {
        void *buffer = nullptr;
        // modify size
        size_t size = aclmdlGetOutputSizeByIndex(modelDesc_.get(), i);
        APP_ERROR ret = aclrtMalloc(&buffer, size, policy);
        if (ret != APP_ERR_OK) {
            LogFatal << modelName_ << "model output aclrtMalloc fail(ret=" << ret
                     << "), buffer=" << buffer << ", size=" << size << ".";
            // Free the buffer malloced successfully before return error
            ReleaseModelBuffer(outputBuffers_);
            return ret;
        }
        outputBuffers_.push_back(buffer);
        outputSizes_.push_back(size);
        LogDebug << modelName_ << "model outputBuffer i=" << i << ", size=" << size << ".";
    }
    return APP_ERR_OK;
}

void ModelProcess::ReleaseModelBuffer(std::vector<void *> &modelBuffers) const
{
    for (size_t i = 0; i < modelBuffers.size(); i++) {
        if (modelBuffers[i] != nullptr) {
            aclrtFree(modelBuffers[i]);
            modelBuffers[i] = nullptr;
        }
    }
}

void ModelProcess::SetModelWH(uint32_t width, uint32_t height)
{
    modelWidth_ = width;
    modelHeight_ = height;
}
