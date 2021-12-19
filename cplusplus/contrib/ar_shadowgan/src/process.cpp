#include "process.h"
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <fstream>
#include <cstring>
#include <sys/stat.h>

#include "acl/acl.h"
#include "model_process.h"
#include "utils.h"

using namespace std;

namespace {
    uint32_t kTopNConfidenceLevels = 5;
}

Process::Process(const char* modelPath,
                                 uint32_t modelWidth, uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), inputBuf_(nullptr), inputBuf2_(nullptr),
modelWidth_(modelWidth), modelHeight_(modelHeight), isInited_(false){
    modelPath_ = modelPath;
    inputDataSize_ = RGBU8_IMAGE_SIZE(modelWidth_, modelHeight_) * 4;     // 256 * 256 * 3
    inputDataSize2_ = GRAY_IMAGE_SIZE(modelWidth_, modelHeight_) * 4;     // 256 * 256 * 1
}

Process::~Process() {
    DestroyResource();
}

Result Process::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("Open device %d success", deviceId_);

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result Process::InitModel(const char* omModelPath, const string& inputimage, const string& maskimage) {
    Result ret = model_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = model_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = model_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ERROR_LOG("Acl malloc image buffer failed.");
        return FAILED;
    }

    aclrtMalloc(&inputBuf2_, (size_t)(inputDataSize2_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf2_ == nullptr) {
        ERROR_LOG("Acl malloc image buffer failed.");
        return FAILED;
    }

    struct stat sBuf;
    int fileStatus = stat(inputimage.data(), &sBuf);
    if (fileStatus == -1) {
        ERROR_LOG("failed to get file");
        return FAILED;
    }

    if (S_ISREG(sBuf.st_mode) == 0) {
        ERROR_LOG("%s is not a file, please enter a file", inputimage.c_str());
        return FAILED;
    }

    std::ifstream binFile(inputimage, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ERROR_LOG("open file %s failed", inputimage.c_str());
        return FAILED;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ERROR_LOG("binfile is empty, filename is %s", inputimage.c_str());
        binFile.close();
        return FAILED;
    }

    binFile.seekg(0, binFile.beg);

    binFile.read(static_cast<char *>(inputBuf_), binFileBufferLen);
    binFile.close();

    struct stat sBuf2;
    int fileStatus2 = stat(maskimage.data(), &sBuf2);
    if (fileStatus2 == -1) {
        ERROR_LOG("failed to get file");
        return FAILED;
    }
    if (S_ISREG(sBuf2.st_mode) == 0) {
        ERROR_LOG("%s is not a file, please enter a file", maskimage.c_str());
        return FAILED;
    }

    std::ifstream binFile2(maskimage, std::ifstream::binary);
    if (binFile2.is_open() == false) {
        ERROR_LOG("open file %s failed", maskimage.c_str());
        return FAILED;
    }

    binFile2.seekg(0, binFile2.end);
    uint32_t binFileBufferLen2 = binFile2.tellg();
    if (binFileBufferLen2 == 0) {
        ERROR_LOG("binfile is empty, filename is %s", maskimage.c_str());
        binFile2.close();
        return FAILED;
    }

    binFile2.seekg(0, binFile2.beg);

    binFile2.read(static_cast<char *>(inputBuf2_), binFileBufferLen2);
    binFile2.close();

    ret = model_.CreateInput(inputBuf_, inputBuf2_, inputDataSize_, inputDataSize2_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    return SUCCESS;
}

Result Process::Init(const string& imageFile, const string& maskFile) {
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }

    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }

    ret = InitModel(modelPath_, imageFile, maskFile);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result Process::Inference(aclmdlDataset*& inferenceOutput) {
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result Process::Postprocess(aclmdlDataset* modelOutput){
    stringstream ss;
    size_t outputNum = aclmdlGetDatasetNumBuffers(modelOutput);
    static int executeNum = 0;
    for (size_t i = 0; i < outputNum; ++i) {
        ss << "./output/output" << ++executeNum << "_" << i << ".bin";
        string outputFileName = ss.str();
        FILE *outputFile = fopen(outputFileName.c_str(), "wb");
        if (outputFile) {
            aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelOutput, i);
            void* data = aclGetDataBufferAddr(dataBuffer);
            uint32_t len = aclGetDataBufferSize(dataBuffer);
            fwrite(data, len, 1, outputFile);
            fclose(outputFile);
            outputFile = nullptr;
        } else {
            ERROR_LOG("create output file [%s] failed", outputFileName.c_str());
            return FAILED;
        }
    }
    INFO_LOG("save data success");
    return SUCCESS;
}

void Process::DestroyResource()
{   aclrtFree(inputBuf_);
    inputBuf_ = nullptr;
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}
