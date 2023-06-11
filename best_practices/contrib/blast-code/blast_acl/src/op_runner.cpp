/**
* @file op_runner.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "op_runner.h"
#include <limits>
#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include "common.h"

using namespace std;

bool g_isDevice = false;
const size_t DEFAULT_WHITE_COUNT = 10;
const size_t DEFAULT_PRECISION_COUNT = 4;
aclrtStream g_stream = nullptr;

OpRunner::OpRunner(OpTestDesc *opDesc) : opDesc_(opDesc)
{
    numInputs_ = opDesc->inputDesc.size();
    numOutputs_ = opDesc->outputDesc.size();
}

OpRunner::~OpRunner()
{
    for (size_t i = 0; i < inputBuffers_.size(); ++i) {
        RETURN_IF_NOT_SUCCESS(aclDestroyDataBuffer(inputBuffers_[i]),
            "Failed to destroy inputs data buffer.", GetRecentErrMsg());
    }
    for (size_t i = 0; i < devInputs_.size(); ++i) {
        if (devInputs_[i] != nullptr) {
            RETURN_IF_NOT_SUCCESS(aclrtFree(devInputs_[i]),
                "Failed to free devInputs memory.", GetRecentErrMsg());
        }
        if (g_isDevice) {
            if (hostInputs_[i] != nullptr) {
                RETURN_IF_NOT_SUCCESS(aclrtFree(hostInputs_[i]),
                    "Failed to free host inputs memory.", GetRecentErrMsg());
            }
        } else {
            if (hostInputs_[i] != nullptr) {
                RETURN_IF_NOT_SUCCESS(aclrtFreeHost(hostInputs_[i]),
                    "Failed to free host or device inputs memory.", GetRecentErrMsg());
            }
        }
    }

    for (size_t i = 0; i < outputBuffers_.size(); ++i) {
        RETURN_IF_NOT_SUCCESS(aclDestroyDataBuffer(outputBuffers_[i]),
            "Failed to destroy outputs data buffer.", GetRecentErrMsg());

        RETURN_IF_NOT_SUCCESS(aclrtFree(devOutputs_[i]),
            "Failed to free devOutputs memory.", GetRecentErrMsg());
        if (g_isDevice) {
            RETURN_IF_NOT_SUCCESS(aclrtFree(hostOutputs_[i]),
                "Failed to free hostOutputs memory.", GetRecentErrMsg());
        } else {
            RETURN_IF_NOT_SUCCESS(aclrtFreeHost(hostOutputs_[i]),
                "Failed to free host or device outputs memory.", GetRecentErrMsg());
        }
    }
}

void OpRunner::GetRecentErrMsg() const
{
    const char *aclRecentErrMsg = nullptr;
    aclRecentErrMsg = aclGetRecentErrMsg();
    if (aclRecentErrMsg != nullptr) {
        ACL_ERROR_LOG("%s", aclRecentErrMsg);
    } else {
        ACL_ERROR_LOG("Failed to get recent error message.");
    }
}

bool OpRunner::OpExecuteInit()
{
    static bool hasInited = false;
    if (hasInited == false) {
        hasInited = true;

        std::string output = "./result_files";
        if (access(output.c_str(), 0) == -1) {
            int ret = mkdir(output.c_str(), 0700);
            if (ret == 0) {
                INFO_LOG("Make output directory successfully.");
            }else {
                ERROR_LOG("Failed to make output directory.");
                return false;
            }
        }

        std::ofstream resultFile;
        resultFile.open("./result_files/result.txt", std::ios::out);
        if (!resultFile.is_open()) {
            ERROR_LOG("Failed to prepare result file.");
            return false;
        }
        resultFile << "Test Result:" << std::endl;
        resultFile.close();

        IF_NOT_SUCCESS_RETURN_FALSE(aclInit("test_data/config/acl.json"),
            "Failed to init acl.", GetRecentErrMsg());

        IF_NOT_SUCCESS_RETURN_FALSE(aclopSetModelDir("op_models"),
            "Failed to load single op model.", GetRecentErrMsg());
    }
    return true;
}

bool OpRunner::CheckDeviceCount(uint32_t deviceId)
{
    uint32_t deviceCount = 0;
    aclError getDeviceStatus = aclrtGetDeviceCount(&deviceCount);
    IF_NOT_SUCCESS_RETURN_FALSE(getDeviceStatus,
        "Failed to get device count.", GetRecentErrMsg());
    if (deviceId >= deviceCount) {
        ERROR_LOG("Device[%d] is out of range, device id maximum is [%d]", deviceId, deviceCount - 1);
        return false;
    }
    return true;
}


bool OpRunner::SetOpExecuteDevice(uint32_t deviceId)
{
    INFO_LOG("------------------Open device[%d]------------------", deviceId);
    if (aclrtSetDevice(deviceId) != ACL_SUCCESS) {
        std::cerr << "Open device failed. device id = " << deviceId << std::endl;
        GetRecentErrMsg();
        return false;
    }
    INFO_LOG("Open device[%d] success", deviceId);
    return true;
}

bool OpRunner::GetDeviceRunMode()
{
    aclrtRunMode runMode;
    IF_NOT_SUCCESS_RETURN_FALSE(aclrtGetRunMode(&runMode),
        "Failed to get acl run mode.", GetRecentErrMsg());
    g_isDevice = (runMode == ACL_DEVICE);
    return true;
}

bool OpRunner::ResetDevice(uint32_t deviceId)
{
    IF_NOT_SUCCESS_RETURN_FALSE(aclrtResetDevice(deviceId),
        "Failed to reset device.", GetRecentErrMsg());
    return true;
}

bool OpRunner::Init()
{
    bool InputInitResult = InputsInit();
    bool OutputInitResult = OutputsInit();
    if (InputInitResult && OutputInitResult) {
        return true;
    } else {
        return false;
    }
}

bool OpRunner::InputsInit()
{
    for (size_t i = 0; i < numInputs_; ++i) {
        auto size = GetInputSize(i);
        // if size is zero, input is an optional
        void *devMem = nullptr;
        if (size != 0) {
            IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMalloc(&devMem, size, ACL_MEM_MALLOC_NORMAL_ONLY),
                "Failed to malloc device memory for input[%zu].", i, GetRecentErrMsg());
            devInputs_.emplace_back(devMem);

            void *hostMem = nullptr;
            if (g_isDevice) {
                IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMalloc(&hostMem, size, ACL_MEM_MALLOC_NORMAL_ONLY),
                    "Failed to malloc device memory for input[%zu].", i, GetRecentErrMsg());
            } else {
                IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMallocHost(&hostMem, size),
                    "Failed to malloc host or device memory for input[%zu].", i, GetRecentErrMsg());
            }
            if (hostMem == nullptr) {
                ERROR_LOG("Failed to malloc memory for input[%zu].", i);
                return false;
            }
            hostInputs_.emplace_back(hostMem);
        }
        inputBuffers_.emplace_back(aclCreateDataBuffer(devMem, size));
    }
    return true;
}

bool OpRunner::OutputsInit()
{
    for (size_t i = 0; i < numOutputs_; ++i) {
        auto size = GetOutputSize(i);
        void *devMem = nullptr;
        IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMalloc(&devMem, size, ACL_MEM_MALLOC_NORMAL_ONLY),
            "Failed to malloc device memory for output[%zu].", i, GetRecentErrMsg());
        devOutputs_.emplace_back(devMem);
        outputBuffers_.emplace_back(aclCreateDataBuffer(devMem, size));

        void *hostOutput = nullptr;
        if (g_isDevice) {
            IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMalloc(&hostOutput, size, ACL_MEM_MALLOC_NORMAL_ONLY),
                "Failed to malloc device memory for output[%zu].", i, GetRecentErrMsg());
        } else {
            IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMallocHost(&hostOutput, size),
                "Failed to malloc host or device memory for output[%zu].", i, GetRecentErrMsg());
        }
        if (hostOutput == nullptr) {
            ERROR_LOG("Failed to Malloc host memory for output[%zu].", i);
            return false;
        }
        hostOutputs_.emplace_back(hostOutput);
    }
    return true;
}

const size_t OpRunner::NumInputs()
{
    return numInputs_;
}

const size_t OpRunner::NumOutputs()
{
    return numOutputs_;
}

const size_t OpRunner::GetInputSize(size_t index) const
{
    if (index >= opDesc_->inputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numInputs = %zu.", index, numInputs_);
        return 0;
    }

    return aclGetTensorDescSize(opDesc_->inputDesc[index]);
}

std::vector<int64_t> OpRunner::GetInputShape(size_t index) const
{
    std::vector<int64_t> ret;
    if (index >= opDesc_->inputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numInputs = %zu.", index, numInputs_);
        return ret;
    }

    auto desc = opDesc_->inputDesc[index];
    for (size_t i = 0; i < aclGetTensorDescNumDims(desc); ++i) {
        int64_t dimSize;
        if (aclGetTensorDescDimV2(desc, i, &dimSize) != ACL_SUCCESS) {
            ERROR_LOG("Failed to get dimension size from tensor desc , dimension index = %zu.", i);
            GetRecentErrMsg();
            ret.clear();
            return ret;
        }
        ret.emplace_back(dimSize);
    }

    return ret;
}

std::vector<int64_t> OpRunner::GetOutputShape(size_t index) const
{
    std::vector<int64_t> ret;
    if (index >= opDesc_->outputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numOutputs = %zu.", index, numOutputs_);
        return ret;
    }

    auto desc = opDesc_->outputDesc[index];
    for (size_t i = 0; i < aclGetTensorDescNumDims(desc); ++i) {
        int64_t dimSize;
        if (aclGetTensorDescDimV2(desc, i, &dimSize) != ACL_SUCCESS) {
            ERROR_LOG("Failed to get dimension size from tensor desc, dimension index = %zu.", i);
            GetRecentErrMsg();
            ret.clear();
            return ret;
        }
        ret.emplace_back(dimSize);
    }
    return ret;
}

size_t OpRunner::GetInputElementCount(size_t index) const
{
    if (index >= opDesc_->inputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numInputs = %zu.", index, numInputs_);
        return 0;
    }

    return aclGetTensorDescElementCount(opDesc_->inputDesc[index]);
}

size_t OpRunner::GetOutputElementCount(size_t index) const
{
    if (index >= opDesc_->outputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numOutputs = %zu.", index, numOutputs_);
        return 0;
    }

    return aclGetTensorDescElementCount(opDesc_->outputDesc[index]);
}

size_t OpRunner::GetOutputSize(size_t index) const
{
    if (index >= opDesc_->outputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numOutputs = %zu.", index, numOutputs_);
        return 0;
    }

    return aclGetTensorDescSize(opDesc_->outputDesc[index]);
}

bool OpRunner::RunOp()
{
    for (size_t i = 0; i < devInputs_.size(); ++i) {
        auto size = GetInputSize(i);
        aclrtMemcpyKind kind = ACL_MEMCPY_HOST_TO_DEVICE;
        if (g_isDevice) {
            kind = ACL_MEMCPY_DEVICE_TO_DEVICE;
        }

        IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMemcpy(devInputs_[i], size, hostInputs_[i], size, kind),
            "Failed to copy input[%zu].", i, GetRecentErrMsg());
        INFO_LOG("Copy input[%zu] success.", i);
    }

    if (g_stream == nullptr) {
        IF_NOT_SUCCESS_RETURN_FALSE(aclrtCreateStream(&g_stream),
            "Failed to create stream.", GetRecentErrMsg());
        INFO_LOG("Create stream success.");
    }

    auto ret = aclopExecuteV2(opDesc_->opType.c_str(),
                              numInputs_,
                              opDesc_->inputDesc.data(),
                              inputBuffers_.data(),
                              numOutputs_,
                              opDesc_->outputDesc.data(),
                              outputBuffers_.data(),
                              opDesc_->opAttr,
                              g_stream);
    if (ret == ACL_ERROR_OP_TYPE_NOT_MATCH || ret == ACL_ERROR_OP_INPUT_NOT_MATCH ||
        ret == ACL_ERROR_OP_OUTPUT_NOT_MATCH || ret == ACL_ERROR_OP_ATTR_NOT_MATCH) {
        ERROR_LOG("[%s] op with the given description is not compiled. Please run atc first", opDesc_->opType.c_str());
        return false;
    } else if (ret != ACL_SUCCESS) {
        ERROR_LOG("Execute %s failed. ret = %d", opDesc_->opType.c_str(), ret);
        GetRecentErrMsg();
        return false;
    }
    INFO_LOG("Execute %s success.", opDesc_->opType.c_str());

    IF_NOT_SUCCESS_RETURN_FALSE(aclrtSynchronizeStream(g_stream),
        "Failed to synchronize stream.", GetRecentErrMsg());
    INFO_LOG("Synchronize stream success.");

    for (size_t i = 0; i < numOutputs_; ++i) {
        auto size = GetOutputSize(i);
        aclrtMemcpyKind kind = ACL_MEMCPY_DEVICE_TO_HOST;
        if (g_isDevice) {
            kind = ACL_MEMCPY_DEVICE_TO_DEVICE;
        }

        IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(aclrtMemcpy(hostOutputs_[i], size, devOutputs_[i], size, kind),
            "Failed to copy output[%zu].", i, GetRecentErrMsg());
        INFO_LOG("Copy output[%zu] success.", i);
        INFO_LOG("PrintOutput output[%zu] success.", i);
        PrintOutput(i,size);
    }
    printf("\n");
    return true;
}


template<typename T>
void DoPrintData(const T *data, size_t count, size_t elementsPerRow)
{
    ios::fmtflags f(cout.flags());
    for (size_t i = 0; i < count; ++i) {
        std::cout << std::setw(DEFAULT_WHITE_COUNT) << data[i];
        if (i % elementsPerRow == elementsPerRow - 1) {
            std::cout << std::endl;
        }
    }
    cout.flags(f);
}

void DoPrintFp16Data(const aclFloat16 *data, size_t count, size_t elementsPerRow)
{
    ios::fmtflags f(cout.flags()); // save old format
    for (size_t i = 0; i < count; ++i) {
        std::cout << std::setw(DEFAULT_WHITE_COUNT) << \
        std::setprecision(DEFAULT_PRECISION_COUNT) << \
        aclFloat16ToFloat(data[i]);
        if (i % elementsPerRow == elementsPerRow - 1) {
            std::cout << std::endl;
        }
    }
    cout.flags(f);
}

void PrintData(const void *data, size_t count, aclDataType dataType, size_t elementsPerRow)
{
    if (data == nullptr) {
        ERROR_LOG("Failed to print data, data is nullptr.");
        return;
    }

    switch (dataType) {
        case ACL_BOOL:
            DoPrintData(reinterpret_cast<const bool *>(data), count, elementsPerRow);
            break;
        case ACL_INT8:
            DoPrintData(reinterpret_cast<const int8_t *>(data), count, elementsPerRow);
            break;
        case ACL_UINT8:
            DoPrintData(reinterpret_cast<const uint8_t *>(data), count, elementsPerRow);
            break;
        case ACL_INT16:
            DoPrintData(reinterpret_cast<const int16_t *>(data), count, elementsPerRow);
            break;
        case ACL_UINT16:
            DoPrintData(reinterpret_cast<const uint16_t *>(data), count, elementsPerRow);
            break;
        case ACL_INT32:
            DoPrintData(reinterpret_cast<const int32_t *>(data), count, elementsPerRow);
            break;
        case ACL_UINT32:
            DoPrintData(reinterpret_cast<const uint32_t *>(data), count, elementsPerRow);
            break;
        case ACL_INT64:
            DoPrintData(reinterpret_cast<const int64_t *>(data), count, elementsPerRow);
            break;
        case ACL_UINT64:
            DoPrintData(reinterpret_cast<const uint64_t *>(data), count, elementsPerRow);
            break;
        case ACL_FLOAT16:
            DoPrintFp16Data(reinterpret_cast<const aclFloat16 *>(data), count, elementsPerRow);
            break;
        case ACL_FLOAT:
            DoPrintData(reinterpret_cast<const float *>(data), count, elementsPerRow);
            break;
        case ACL_DOUBLE:
            DoPrintData(reinterpret_cast<const double *>(data), count, elementsPerRow);
            break;
        default:
            ERROR_LOG("Unsupported type: %d", dataType);
    }
}

void OpRunner::PrintInput(size_t index, size_t numElementsPerRow)
{
    if (index >= opDesc_->inputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numOutputs = %zu.", index, numInputs_);
        return;
    }

    auto desc = opDesc_->inputDesc[index];
    PrintData(hostInputs_[index], GetInputElementCount(index), aclGetTensorDescType(desc), numElementsPerRow);
}

void OpRunner::PrintOutput(size_t index, size_t numElementsPerRow)
{
    if (index >= opDesc_->outputDesc.size()) {
        ERROR_LOG("Index out of range. index = %zu, numOutputs = %zu.", index, numOutputs_);
        return;
    }

    auto desc = opDesc_->outputDesc[index];
    PrintData(hostOutputs_[index], GetOutputElementCount(index), aclGetTensorDescType(desc), numElementsPerRow);
}
