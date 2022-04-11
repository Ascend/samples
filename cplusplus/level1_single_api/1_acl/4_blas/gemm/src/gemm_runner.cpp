/**
* @file gemm_runner.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "gemm_runner.h"

#include "acl/ops/acl_cblas.h"
#include "common.h"

extern bool g_isDevice;

namespace {
    constexpr uint32_t MAX_ROWS = 16;
}

GemmRunner::GemmRunner(uint32_t m, uint32_t n, uint32_t k,
    aclDataType inputType, aclDataType outputType)
  : m_(m), n_(n), k_(k),
    inputType_(inputType), outputType_(outputType)
{
    sizeA_ = m * k * aclDataTypeSize(inputType);
    sizeB_ = k * n * aclDataTypeSize(inputType);
    sizeC_ = m * n * aclDataTypeSize(outputType);
    sizeAlphaBeta_ = aclDataTypeSize(outputType);
}

GemmRunner::~GemmRunner()
{
    if (devMatrixA_ != nullptr) {
        (void)aclrtFree(devMatrixA_);
    }
    if (devMatrixB_ != nullptr) {
        (void)aclrtFree(devMatrixB_);
    }
    if (devMatrixC_ != nullptr) {
        (void)aclrtFree(devMatrixC_);
    }
    if (devAlpha_ != nullptr) {
        (void) aclrtFree(devAlpha_);
    }
    if (devBeta_ != nullptr) {
        (void)aclrtFree(devBeta_);
    }
    if (!g_isDevice) {
        (void)aclrtFreeHost(hostMatrixA_);
        (void)aclrtFreeHost(hostMatrixB_);
        (void)aclrtFreeHost(hostMatrixC_);
    }
}

bool GemmRunner::Init()
{
    if (aclrtMalloc((void **) &devMatrixA_, sizeA_, ACL_MEM_MALLOC_HUGE_FIRST) != ACL_SUCCESS) {
        ERROR_LOG("malloc device memory for matrix A failed");
        return false;
    }

    if (aclrtMalloc((void **) &devMatrixB_, sizeB_, ACL_MEM_MALLOC_HUGE_FIRST) != ACL_SUCCESS) {
        ERROR_LOG("malloc device memory for matrix B failed");
        return false;
    }

    if (aclrtMalloc((void **) &devMatrixC_, sizeC_, ACL_MEM_MALLOC_HUGE_FIRST) != ACL_SUCCESS) {
        ERROR_LOG("malloc device memory for matrix C failed");
        return false;
    }

    if (aclrtMalloc((void **) &devAlpha_, sizeAlphaBeta_, ACL_MEM_MALLOC_HUGE_FIRST) != ACL_SUCCESS) {
        ERROR_LOG("malloc device memory for alpha failed");
        return false;
    }

    if (aclrtMalloc((void **) &devBeta_, sizeAlphaBeta_, ACL_MEM_MALLOC_HUGE_FIRST) != ACL_SUCCESS) {
        ERROR_LOG("malloc device memory for beta failed");
        return false;
    }

    if (g_isDevice) {
        hostMatrixA_ = devMatrixA_;
        hostMatrixB_ = devMatrixB_;
        hostMatrixC_ = devMatrixC_;
    } else {
        if (aclrtMallocHost((void **) &hostMatrixA_, sizeA_) != ACL_SUCCESS) {
            ERROR_LOG("malloc host memory for matrix A failed");
            return false;
        }

        if (aclrtMallocHost((void **) &hostMatrixB_, sizeB_) != ACL_SUCCESS) {
            ERROR_LOG("malloc host memory for matrix B failed");
            return false;
        }

        if (aclrtMallocHost((void **) &hostMatrixC_, sizeC_) != ACL_SUCCESS) {
            ERROR_LOG("malloc host memory for matrix C failed");
            return false;
        }
    }
    return true;
}

bool GemmRunner::PrepareInputs()
{
    size_t fileSize;
    // Read matrix A
    char *fileData = ReadFile("test_data/data/matrix_a.bin", fileSize, hostMatrixA_, sizeA_);
    if (fileData == nullptr) {
        ERROR_LOG("Read matrix A failed.");
        return false;
    }

    // Read matrix B
    fileData = ReadFile("test_data/data/matrix_b.bin", fileSize, hostMatrixB_, sizeB_);
    if (fileData == nullptr) {
        ERROR_LOG("Read matrix B failed.");
        return false;
    }

    // Read matrix C
    fileData = ReadFile("test_data/data/matrix_c.bin", fileSize, hostMatrixC_, sizeC_);
    if (fileData == nullptr) {
        ERROR_LOG("Read matrix C failed.");
        return false;
    }

    return true;
}

bool GemmRunner::CopyInput()
{
    aclError ret = ACL_SUCCESS;
    if (!g_isDevice) {
        ret = aclrtMemcpy(devMatrixA_, sizeA_, hostMatrixA_, sizeA_,
                          ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("Copy matrix A from host to device failed, errorCode[%d]",
                static_cast<int32_t>(ret));
            return false;
        }

        ret = aclrtMemcpy(devMatrixB_, sizeB_, hostMatrixB_, sizeB_,
                          ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("Copy matrix B from host to device failed, errorCode[%d]",
                static_cast<int32_t>(ret));
            return false;
        }

        ret = aclrtMemcpy(devMatrixC_, sizeC_, hostMatrixC_, sizeC_,
                          ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("Copy matrix C from host to device failed, errorCode[%d]",
                static_cast<int32_t>(ret));
            return false;
        }
    }

    aclrtMemcpyKind kind = g_isDevice ? ACL_MEMCPY_DEVICE_TO_DEVICE : ACL_MEMCPY_HOST_TO_DEVICE;
    ret = aclrtMemcpy(devAlpha_, sizeAlphaBeta_, hostAlpha_, sizeAlphaBeta_, kind);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Copy alpha from host to device failed, errorCode[%d]",
            static_cast<int32_t>(ret));
        return false;
    }

    ret = aclrtMemcpy(devBeta_, sizeAlphaBeta_, hostBeta_, sizeAlphaBeta_, kind);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Copy beta from host to device failed, errorCode[%d]",
            static_cast<int32_t>(ret));
        return false;
    }

    return true;
}

bool GemmRunner::CopyOutput()
{
    if (!g_isDevice) {
        auto ret = aclrtMemcpy(hostMatrixC_, sizeC_, devMatrixC_, sizeC_, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("Copy output from device to host failed, errorCode[%d]",
                static_cast<int32_t>(ret));
            return false;
        }
    }
    return true;
}

bool GemmRunner::RunOp()
{
    // 1. Create stream
    aclrtStream stream = nullptr;
    if (aclrtCreateStream(&stream) != ACL_SUCCESS) {
        ERROR_LOG("Create stream failed");
        return false;
    }
    INFO_LOG("Create stream success");

    // 2. Copy inputs
    if (!CopyInput()) {
        ERROR_LOG("Copy inputs failed");
        (void)aclrtDestroyStream(stream);
        return false;
    }
    INFO_LOG("Copy inputs success");

    // 3. Launch kernel
    if (aclblasGemmEx(ACL_TRANS_N, ACL_TRANS_N, ACL_TRANS_N, m_, n_, k_,
                      devAlpha_, devMatrixA_, -1, inputType_, devMatrixB_, -1, inputType_,
                      devBeta_, devMatrixC_, -1, outputType_, ACL_COMPUTE_HIGH_PRECISION,
                      stream) != ACL_SUCCESS) {
        ERROR_LOG("Launch Gemm kernel failed");
        (void)aclrtDestroyStream(stream);
        return false;
    }
    INFO_LOG("Launch Gemm kernel success");

    // 4. Synchronize stream
    if (aclrtSynchronizeStream(stream) != ACL_SUCCESS) {
        ERROR_LOG("Synchronize stream failed");
        (void)aclrtDestroyStream(stream);
        return false;
    }
    INFO_LOG("Synchronize stream success");

    // 5. Copy output
    if (!CopyOutput()) {
        ERROR_LOG("Copy output failed");
        (void)aclrtDestroyStream(stream);
        return false;
    }
    INFO_LOG("Copy output success");

    // release resources
    (void)aclrtDestroyStream(stream);
    return true;
}

template<typename T>
void DoPrintMatrix(const T *matrix, uint32_t numRows, uint32_t numCols)
{
    uint32_t rows = numRows;
    if (rows >= MAX_ROWS) {
        rows = MAX_ROWS;
    }

    for (uint32_t i = 0; i < rows; ++i) {
        for (uint32_t j = 0; j < numCols; ++j) {
            std::cout << std::setw(10) << matrix[i * numCols + j];
        }
        std::cout << std::endl;
    }

    if (rows < numRows) {
        std::cout << std::setw(10) << "......" << std::endl;
    }
}

template<>
void DoPrintMatrix(const int8_t *matrix, uint32_t numRows, uint32_t numCols)
{
  uint32_t rows = numRows;
  if (rows >= MAX_ROWS) {
    rows = MAX_ROWS;
  }

  for (uint32_t i = 0; i < rows; ++i) {
    for (uint32_t j = 0; j < numCols; ++j) {
      std::cout << std::setw(6) << static_cast<int>(matrix[i * numCols + j]);
    }
    std::cout << std::endl;
  }

  if (rows < numRows) {
    std::cout << std::setw(10) << "......" << std::endl;
  }
}

void DoPrintMatrixFp16(const aclFloat16 *matrix, uint32_t numRows, uint32_t numCols)
{
    uint32_t rows = numRows;
    if (rows >= MAX_ROWS) {
        rows = MAX_ROWS;
    }

    for (uint32_t i = 0; i < numRows; ++i) {
        for (uint32_t j = 0; j < numCols; ++j) {
            std::cout << std::setw(10) << aclFloat16ToFloat(matrix[i * numCols + j]);
        }
        std::cout << std::endl;
    }

    if (rows < numRows) {
        std::cout << std::setw(10) << "......" << std::endl;
    }
}

void PrintMatrix(const void *matrix, aclDataType type, uint32_t numRows, uint32_t numCols)
{
    switch (type) {
        case ACL_FLOAT16:
            DoPrintMatrixFp16(reinterpret_cast<const aclFloat16 *>(matrix), numRows, numCols);
            return;
        case ACL_FLOAT:
            DoPrintMatrix(reinterpret_cast<const float *>(matrix), numRows, numCols);
            return;
        case ACL_INT8:
            DoPrintMatrix(reinterpret_cast<const int8_t *>(matrix), numRows, numCols);
            return;
        case ACL_INT32:
            DoPrintMatrix(reinterpret_cast<const int32_t *>(matrix), numRows, numCols);
            return;
        default:
            ERROR_LOG("unsupported type: %d", static_cast<int>(type));
    }
}

bool GemmRunner::WriteOutput()
{
    if (!WriteFile("result_files/matrix_c.bin", hostMatrixC_, sizeC_)) {
        ERROR_LOG("Write output to result_files/matrix_c.bin failed");
        return false;
    }

    INFO_LOG("Write result success. file = result_files/matrix_c.bin");
    return true;
}

void GemmRunner::PrintMatrixA()
{
    if (hostMatrixA_ == nullptr) {
        ERROR_LOG("GemmRunner is not initialized");
        return;
    }

    PrintMatrix(hostMatrixA_, inputType_, m_, k_);
}

void GemmRunner::PrintMatrixB()
{
    if (hostMatrixB_ == nullptr) {
        ERROR_LOG("GemmRunner is not initialized");
        return;
    }

    PrintMatrix(hostMatrixB_, inputType_, k_, n_);
}

void GemmRunner::PrintMatrixC()
{
    if (hostMatrixC_ == nullptr) {
        ERROR_LOG("GemmRunner is not initialized");
        return;
    }

    PrintMatrix(hostMatrixC_, outputType_, m_, n_);
}
