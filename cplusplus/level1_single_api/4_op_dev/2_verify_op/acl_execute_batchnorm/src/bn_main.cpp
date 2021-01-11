/**
* @file main.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <iostream>
#include <fstream>

#include <dirent.h>
#include <cassert>

#include "acl/acl.h"
#include "batchnorm_tiling.h"

#define ACL_REQUIRES_OK(expr) \
    do { \
        auto __ret = (expr); \
        if (__ret != ACL_SUCCESS) { \
            return __ret; \
        } \
    } \
    while (0)

bool CheckDims(size_t n, size_t c, size_t h, size_t w) {
    if (n == 0 || c == 0 || h == 0 || w == 0) {
        std::cout << "[ERROR] All dim can't support 0" << std::endl;
        return false;
    }

    if(n != 1){
        std::cout << "[ERROR] Batch N only support 1" << std::endl;
        return false;
    }

    return true;
}

void DisplayResult(int64_t &n, int64_t &c, int64_t &h, int64_t &w, aclFloat16 *output) {
    std::cout << "The result of BatchNorm is(showing no more than 16 elements):" << std::endl;
    int i = 0;
    for (; i < (n * c * h * w); ++i) {
        if (i < 16) {
            std::cout << aclFloat16ToFloat(output[i]) << ", ";
        }
        if (aclFloat16ToFloat(output[i]) != 5.0) {
            printf("Case failed! Index %d element is %f but 5.0 expected.\n",
                   i, aclFloat16ToFloat(output[i]));
            break;
        }
    }
    if (i == (n * c * h * w)) {
        std::cout << "Case passed!" << std::endl;
    }
}

aclError BatchNormTest(int64_t n, int64_t c, int64_t h, int64_t w)
{
    aclrtRunMode runMode;
    if (aclrtGetRunMode(&runMode) != ACL_SUCCESS) {
        std::cout << "[ERROR] Get run mode failed" << std::endl;
        return ACL_ERROR_BAD_ALLOC;
    }

    // input size is shape * sizeof(float16)
    int64_t size_input = n * c * h * w * 2;
    int64_t size_output = n * c * h * w * 2;
    int64_t size_gamma = c * 2;
    int64_t size_beta = c * 2;

    int64_t shape_input[] = {n, c, h, w};
    int64_t shape_gamma[] = {c};
    int64_t shape_beta[] = {c};
    int64_t shape_out[] = {n, c, h, w};

    if (CheckDims(n, c, h, w) == false) {
        return ACL_ERROR_BAD_ALLOC;
    }

    aclFloat16 *input = new(std::nothrow) aclFloat16[n * c * h * w];
    if (input == NULL) {
        return ACL_ERROR_BAD_ALLOC;
    }
    aclFloat16 *gamma = new(std::nothrow) aclFloat16[c];
    if (gamma == NULL) {
        delete[]input;
        return ACL_ERROR_BAD_ALLOC;
    }
    aclFloat16 *beta = new(std::nothrow) aclFloat16[c];
    if (beta == NULL) {
        delete[]input;
        delete[]gamma;
        return ACL_ERROR_BAD_ALLOC;
    }
    aclFloat16 *output = new(std::nothrow) aclFloat16[n * c * h * w];
    if (output == NULL) {
        delete[]input;
        delete[]gamma;
        delete[]beta;
        return ACL_ERROR_BAD_ALLOC;
    }

    aclTensorDesc *input_desc[3];
    aclTensorDesc *output_desc[1];
    input_desc[0] = aclCreateTensorDesc(ACL_FLOAT16, 4, shape_input, ACL_FORMAT_NCHW);
    input_desc[1] = aclCreateTensorDesc(ACL_FLOAT16, 1, shape_gamma, ACL_FORMAT_ND);
    input_desc[2] = aclCreateTensorDesc(ACL_FLOAT16, 1, shape_beta, ACL_FORMAT_ND);
    output_desc[0] = aclCreateTensorDesc(ACL_FLOAT16, 4, shape_out, ACL_FORMAT_NCHW);


    //initilize data
    for (int i = 0; i < n * c * h * w; ++i) {
        input[i] = aclFloatToFloat16(1.0f);
    }

    for (int i = 0; i < c; ++i) {
        gamma[i] = aclFloatToFloat16(0.5f);
        beta[i] = aclFloatToFloat16(0.1f);
    }

    aclrtStream stream;
    ACL_REQUIRES_OK(aclrtCreateStream(&stream));

    void *devInput = nullptr;
    void *devInput_gamma = nullptr;
    void *devInput_beta = nullptr;
    void *devOutput = nullptr;

    ACL_REQUIRES_OK(aclrtMalloc(&devInput, size_input, ACL_MEM_MALLOC_NORMAL_ONLY));
    ACL_REQUIRES_OK(aclrtMalloc(&devInput_gamma, size_gamma, ACL_MEM_MALLOC_NORMAL_ONLY));
    ACL_REQUIRES_OK(aclrtMalloc(&devInput_beta, size_beta, ACL_MEM_MALLOC_NORMAL_ONLY));
    ACL_REQUIRES_OK(aclrtMalloc(&devOutput, size_output, ACL_MEM_MALLOC_NORMAL_ONLY));

    aclrtMemcpyKind kindInput = ACL_MEMCPY_HOST_TO_DEVICE;
    if (runMode == ACL_DEVICE) {
        kindInput = ACL_MEMCPY_DEVICE_TO_DEVICE;
    }
    ACL_REQUIRES_OK(aclrtMemcpy(devInput, size_input, input, size_input, kindInput));
    ACL_REQUIRES_OK(aclrtMemcpy(devInput_gamma, size_gamma, gamma, size_gamma, kindInput));
    ACL_REQUIRES_OK(aclrtMemcpy(devInput_beta, size_beta, beta, size_beta, kindInput));

    aclDataBuffer *inputBuffer = aclCreateDataBuffer(devInput, size_input);
    aclDataBuffer *inputgammaBuffer = aclCreateDataBuffer(devInput_gamma, size_gamma);
    aclDataBuffer *inputbetaBuffer = aclCreateDataBuffer(devInput_beta, size_beta);
    aclDataBuffer *outputBuffer = aclCreateDataBuffer(devOutput, size_output);

    aclDataBuffer *inputs[] = {inputBuffer, inputgammaBuffer, inputbetaBuffer};
    aclDataBuffer *outputs[] = {outputBuffer};

    ACL_REQUIRES_OK(
            aclopUpdateParams("BatchNorm", 3, input_desc, 1, output_desc, nullptr));
    ACL_REQUIRES_OK(aclopExecuteV2("BatchNorm", 3, input_desc, inputs, 1, output_desc, outputs, nullptr, stream));

    ACL_REQUIRES_OK(aclrtSynchronizeStream(stream));
    ACL_REQUIRES_OK(aclrtDestroyStream(stream));

    aclrtMemcpyKind kindOutput = ACL_MEMCPY_DEVICE_TO_HOST;
    if (runMode == ACL_DEVICE) {
        kindOutput = ACL_MEMCPY_DEVICE_TO_DEVICE;
    }
    ACL_REQUIRES_OK(aclrtMemcpy(output, size_output, devOutput, size_output, kindOutput));

    // display the result of this case
    DisplayResult(n, c, h, w, output);

    ACL_REQUIRES_OK(aclrtFree(devInput));
    ACL_REQUIRES_OK(aclrtFree(devInput_gamma));
    ACL_REQUIRES_OK(aclrtFree(devInput_beta));
    ACL_REQUIRES_OK(aclrtFree(devOutput));

    ACL_REQUIRES_OK(aclDestroyDataBuffer(inputBuffer));
    ACL_REQUIRES_OK(aclDestroyDataBuffer(inputgammaBuffer));
    ACL_REQUIRES_OK(aclDestroyDataBuffer(inputbetaBuffer));
    ACL_REQUIRES_OK(aclDestroyDataBuffer(outputBuffer));

    for (auto desc : input_desc) {
        aclDestroyTensorDesc(desc);
    }

    for (auto desc : output_desc) {
        aclDestroyTensorDesc(desc);
    }

    delete[]input;
    delete[]gamma;
    delete[]beta;
    delete[]output;

    return ACL_SUCCESS;
}

bool ReadBytesFromBinaryFile(const char *file_name, char **buffer, int &length)
{
    if (file_name == nullptr) {
        std::cout << "File is nullptr, parameter error." << std::endl;
        return false;
    }

    std::string real_path = file_name;
    real_path = "kernel_meta/" + real_path;

    std::ifstream file(real_path.c_str(), std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        printf("Read file %s failed.", file_name);
        return false;
    }

    length = static_cast<int>(file.tellg());
    if (length <= 0) {
        printf("File length <= 0");
        file.close();
        return false;
    }

    file.seekg(0, std::ios::beg);

    *buffer = new(std::nothrow) char[length]();
    if (buffer == nullptr) {
        printf("New an object failed");
        file.close();
        return false;
    }

    file.read(*buffer, length);
    file.close();
    return true;
}

void Deallocator(void *data, size_t size)
{
    auto addr = reinterpret_cast<char *>(data);
    delete[]addr;
}

int main(int argc, char **argv)
{
    if (argc != 5) {
        printf("argc number is not 4!\n");
        return -1;
    }

    if (aclInit(nullptr) != ACL_SUCCESS) {
        std::cout << "Acl init failed." << std::endl;
        return -1;
    }

    const char file_1[] = "tiling_mode_1.o";
    const char file_2[] = "tiling_mode_2.o";
    const char file_3[] = "tiling_mode_3.o";

    char *buffer = nullptr;
    int length = 0;

    aclrtSetDevice(0);

    aclopRegisterCompileFunc("BatchNorm", SelectAclopBatchNorm);

    ReadBytesFromBinaryFile(file_1, &buffer, length);
    aclopCreateKernel("BatchNorm", "tiling_mode_1__kernel0", "tiling_mode_1__kernel0",
                      buffer, length, ACL_ENGINE_AICORE, Deallocator);

    ReadBytesFromBinaryFile(file_2, &buffer, length);
    aclopCreateKernel("BatchNorm", "tiling_mode_2__kernel0", "tiling_mode_2__kernel0",
                      buffer, length, ACL_ENGINE_AICORE, Deallocator);

    ReadBytesFromBinaryFile(file_3, &buffer, length);
    aclopCreateKernel("BatchNorm", "tiling_mode_3__kernel0", "tiling_mode_3__kernel0",
                      buffer, length, ACL_ENGINE_AICORE, Deallocator);

    int n = atoi(argv[1]);
    int c = atoi(argv[2]);
    int h = atoi(argv[3]);
    int w = atoi(argv[4]);
    if (n <= 0 || c <= 0 || h <= 0 || w <= 0) {
            std::cout << "[ERROR] The shape is invalid that n/c/h/w must be positive integer."
                      << std::endl;
            return -1;
    }

    //main process
    if (BatchNormTest(n, c, h, w) != ACL_SUCCESS) {
        std::cout << "[ERROR] BatchNorm test failed." << std::endl;
        return -1;
    }

    aclrtResetDevice(0);

    if (aclFinalize() != ACL_SUCCESS) {
      std::cout << "[ERROR] Finalize acl failed." << std::endl;
      return -1;
    }
    return 0;
}

