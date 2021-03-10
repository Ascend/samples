#pragma once
#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"
#include <bits/stdint-uintn.h>
#include <memory>
#include <string>

//using namespace std;

/**
* ClassifyProcess
*/
class Process {
public:
    Process(const char* modelPath, uint32_t modelWidth, uint32_t modelHeight);
    ~Process();

    Result Init(const std::string& imageFile, const std::string& maskFile);
    Result Inference(aclmdlDataset*& inferenceOutput);
    Result Postprocess(aclmdlDataset* modelOutput);
private:
    Result InitResource();
    Result InitModel(const char* omModelPath, const std::string& imageFile, const std::string& maskFile);
    void DestroyResource();

private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    ModelProcess model_;

    const char* modelPath_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t inputDataSize_;
    uint32_t inputDataSize2_;
    void*    inputBuf_;
    void*    inputBuf2_;
    aclrtRunMode runMode_;

    bool isInited_;
};

