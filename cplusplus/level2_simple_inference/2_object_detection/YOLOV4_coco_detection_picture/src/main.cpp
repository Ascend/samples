/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File main.cpp
* Description: tensorflow yolov4 demo
*/
#include <iostream>
#include <fstream>
#include <map>
#include <cstdint>
#include <string>
#include <memory>
#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"
#include "post_process.h"

using namespace std;

#define CHECK_FUNCTION_RESULT(name, result) \
{ \
    if(result != SUCCESS) \
    { \
        ERROR_LOG(#name" failed"); \
        return -1; \
    } \
}

void BufferDeleter(void *p) {
    if (!RunStatus::GetDeviceStatus()) {
        if (p != nullptr) {
            (void) aclrtFreeHost(p);
        }
    }
}

/**
* @brief post processing for output
* @param [in] output: output data for model inference
* @param [in] originImage: origin image file path
* @param [in] xScale: x scale of the origin image and the model input image
* @param [in] yScale: y scale of the origin image and the model input image
*/
void ProcessOutput(const aclmdlDataset *output, const char *originImage, float xScale, float yScale) {
    FILE *outputFile = nullptr;
    shared_ptr<void> dataBuff[2] = {nullptr, nullptr};
    size_t index = 0;
    for (size_t i = 0; (i < aclmdlGetDatasetNumBuffers(output)) && (i < 2); ++i) {
        std::string name;
        name = "../out/output_" + to_string(i) + ".bin";
        if (outputFile != nullptr) {
            fclose(outputFile);
        }
        outputFile = fopen(name.c_str(), "wb");
        //get model output data
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(output, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        size_t len = aclGetDataBufferSize(dataBuffer);

        if (RunStatus::GetDeviceStatus()) {
            fwrite(data, len, sizeof(char), outputFile);
            shared_ptr<void> ptr(data, BufferDeleter);
            dataBuff[i] = ptr;
        } else {
            void *outHostData = nullptr;
            aclError ret = aclrtMallocHost(&outHostData, len);
            if (ret != ACL_ERROR_NONE) {
                cout << "aclrtMallocHost failed, result code is " << ret << endl;
                break;
            }
            ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_ERROR_NONE) {
                cout << "aclrtMemcpy failed, result code is " << ret << endl;
                (void) aclrtFree(outHostData);
                break;
            }
            fwrite(outHostData, len, sizeof(char), outputFile);
            shared_ptr<void> ptr(outHostData, BufferDeleter);
            dataBuff[i] = ptr;
        }

    }
    if (outputFile != nullptr) {
        fclose(outputFile);
    }

    string imagePath(originImage);
    PostProcess postProcess(dataBuff[0].get(), dataBuff[1].get(), xScale, yScale, imagePath);
    Result rlt = postProcess.Process();
    if (rlt != SUCCESS) {
        ERROR_LOG("post process failed.");
    }
    INFO_LOG("post process success.");
}

/**
* @brief post get memory buffer after image decode and resize
* @param [in] imageFile: image file path
* @param [in] resizeWidth: resize width
* @param [in] resizeHeight: resize height
* @param [out] buffer: buffer
* @param [out] bufferLen: length of buffer
* @param [out] xScale: x scale of the origin image and resized image
* @param [out] yScale: y scale of the origin image and resized image
* @return result failed or success
*/
Result GetImageResizeBuffer(const char *imageFile, int resizeWidth, int resizeHeight,
                            void *&buffer, size_t &bufferLen,
                            float &xScale, float &yScale) {
    aclrtStream stream = nullptr;
    aclError ret = aclrtCreateStream(&stream);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("create stream failed, error code is %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    uint32_t devBufferSize = 0;
    void *devBuffer = nullptr;
    PicDesc testPic = {imageFile, 0, 0, 0};
    if (SUCCESS != Utils::GetDeviceBufferOfPicture(testPic, devBuffer, devBufferSize)) {
        ERROR_LOG("get image device buffer failed.");
        (void) aclrtDestroyStream(stream);
        return FAILED;
    }
    INFO_LOG("get device buffer for picture success.");

    xScale = static_cast<float>(testPic.width) / static_cast<float>(resizeWidth);
    yScale = static_cast<float>(testPic.height) / static_cast<float>(resizeHeight);

    DvppProcess dvppProcess(stream);
    if (SUCCESS != dvppProcess.InitResource()) {
        ERROR_LOG("dvpp process init failed.");
        (void) aclrtDestroyStream(stream);
        return FAILED;
    }

    dvppProcess.SetInput(devBuffer, devBufferSize, testPic);
    if (SUCCESS != dvppProcess.InitDvppOutputPara(resizeWidth, resizeHeight)) {
        ERROR_LOG("dvpp init output parameters failed.");
        (void) acldvppFree(devBuffer);
        (void) aclrtDestroyStream(stream);
        return FAILED;
    }

    if (SUCCESS != dvppProcess.Process()) {
        ERROR_LOG("dvpp process failed.");
        (void) acldvppFree(devBuffer);
        (void) aclrtDestroyStream(stream);
        return FAILED;
    }

    INFO_LOG("dvpp decode and resize success.");

    int len = 0;
    dvppProcess.GetDvppOutput(&buffer, len);
    bufferLen = static_cast<size_t>(len);

    (void) acldvppFree(devBuffer);
    devBuffer = nullptr;
    (void) aclrtDestroyStream(stream);

    return SUCCESS;
}


/**
* @brief main
* @param [in] argc: the count of arguments
* @param [in] argv: the value of arguments
* @return process result, 0 is success, other is failure
*/
int main(int argc, const char *argv[]) {
    if ((argc < 3) || (argv[1] == nullptr) || (argv[2] == nullptr)) {
        ERROR_LOG("Please input:./main <model_file> <image_dir>");
        return -1;
    }

    // model process init
    ModelProcess modelProcess;
    Result rlt = modelProcess.Init(0);
    CHECK_FUNCTION_RESULT(modelProcess.Init, rlt);

    rlt = modelProcess.LoadModel(argv[1]);
    CHECK_FUNCTION_RESULT(modelProcess.LoadModel, rlt);

    int width = 0;
    int height = 0;
    rlt = modelProcess.GetModelInputWH(width, height);
    CHECK_FUNCTION_RESULT(modelProcess.GetModelInputWH, rlt);

    void *inputBuffer = nullptr;
    size_t inputLen = 0;
    float xScale = 1;
    float yScale = 1;
    rlt = GetImageResizeBuffer(argv[2], width, height,
                               inputBuffer, inputLen, xScale, yScale);
    CHECK_FUNCTION_RESULT(GetImageResizeBuffer, rlt);

    // Input memory for creating models
    rlt = modelProcess.CreateInput(inputBuffer, inputLen);
    CHECK_FUNCTION_RESULT(modelProcess.LoadModel, rlt);

    // Create the output memory of the model
    rlt = modelProcess.CreateOutput();
    CHECK_FUNCTION_RESULT(modelProcess.CreateOutput, rlt);

    // The input and output memory is transferred to the model instance for synchronous execution
    rlt = modelProcess.Execute();
    CHECK_FUNCTION_RESULT(modelProcess.Execute, rlt);

    // After execution, the post-processing is carried out for the output memory
    const aclmdlDataset *output = modelProcess.GetModelOutputData();
    ProcessOutput(output, argv[2], xScale, yScale);

    // Free output memory
    modelProcess.DestroyOutput();

    // Free input memory
    modelProcess.DestroyInput();

    // Unload model instance
    modelProcess.UnloadModel();

    // Release device and thread context
    modelProcess.Finalize();

    // Return success
    return 0;
}
