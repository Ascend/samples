/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#include <iostream>
#include "object_detect.h"
#include "model_process.h"
#include "acl/acl.h"
#include "utils.h"

using namespace std;

namespace {
const static std::vector<std::string> yolov3Label = { "person", "bicycle", "car", "motorbike",
    "aeroplane","bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag","tie",
    "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "potted plant", "bed", "dining table", "toilet",
    "TV monitor", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush" };
// Inferential output dataset subscript 0 unit is detection box information data
const uint32_t g_bBoxDataBufId = 0;
// The unit with subscript 1 is the number of boxes
const uint32_t g_boxNumDataBufId = 1;
// Each field subscript in the box message
enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
}

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
                           uint32_t modelHeight)
    : g_deviceId_(0), g_imageDataBuf_(nullptr), g_imageInfoBuf_(nullptr),
      g_modelWidth_(modelWidth), g_modelHeight_(modelHeight),
      g_isInited_(false)
{
    g_modelPath_ = modelPath;
    g_imageDataSize_ = RGBU8_IMAGE_SIZE(modelWidth, modelHeight);
    g_channel_ = nullptr;
    PresenterErrorCode openChannelret = OpenChannelByConfig(g_channel_, "./param.conf");
    if (openChannelret != PresenterErrorCode::kNone) {
        ERROR_LOG("Open channel failed, error %d\n", (int)openChannelret);
    }
}

ObjectDetect::~ObjectDetect()
{
    DestroyResource();
}

Result ObjectDetect::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    // open device
    ret = aclrtSetDevice(g_deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Acl open device %d failed", g_deviceId_);
        return FAILED;
    }
    INFO_LOG("Open device %d success", g_deviceId_);
    // Gets whether the current application is running on host or Device
    ret = aclrtGetRunMode(&g_runMode_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::InitModel(const char* omModelPath)
{
    Result ret = g_model_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = g_model_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = g_model_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::CreateModelInputdDataset()
{
    // Request image data memory for input model
    aclError aclRet = aclrtMalloc(&g_imageDataBuf_, g_imageDataSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return FAILED;
    }
    // The second input to Yolov3 is the input image width and height parameter
    const float imageInfo[4] = {(float)g_modelWidth_, (float)g_modelHeight_,
                                (float)g_modelWidth_, (float)g_modelHeight_};
    g_imageInfoSize_ = sizeof(imageInfo);
    if (g_runMode_ == ACL_HOST)
        g_imageInfoBuf_ = Utils::CopyDataHostToDevice((void *)imageInfo, g_imageInfoSize_);
    else
        g_imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *)imageInfo, g_imageInfoSize_);
    if (g_imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed");
        return FAILED;
    }
    // Use the applied memory to create the model and input dataset. After creation,
    // only update the memory data for each frame of inference, instead of creating the input dataset every time
    Result ret = g_model_.CreateInput(g_imageDataBuf_, g_imageDataSize_,
    g_imageInfoBuf_, g_imageInfoSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Init()
{
    // If it is already initialized, it is returned
    if (g_isInited_) {
        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }
    // Initializes the ACL resource
    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }
    // Initializes the model management instance
    ret = InitModel(g_modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }
	
    ret = CreateModelInputdDataset();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed");
        return FAILED;
    }

    g_isInited_ = true;
    return SUCCESS;
}

Result ObjectDetect::Preprocess(cv::Mat& frame)
{
    // Scale the frame image to the desired size of the model
    cv::Mat reiszeMat;
    cv::resize(frame, reiszeMat, cv::Size(g_modelWidth_, g_modelHeight_));
    if (reiszeMat.empty()) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }
    // Copy the data into the cache of the input dataset
    aclrtMemcpyKind policy = (g_runMode_ == ACL_HOST)?
                             ACL_MEMCPY_HOST_TO_DEVICE:ACL_MEMCPY_DEVICE_TO_DEVICE;
    aclError ret = aclrtMemcpy(g_imageDataBuf_, g_imageDataSize_,
                               reiszeMat.ptr<uint8_t>(), g_imageDataSize_, policy);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Copy resized image data to device failed.");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Inference(aclmdlDataset*& inferenceOutput)
{
    // Perform reasoning
    Result ret = g_model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }
    // Get inference output
    inferenceOutput = g_model_.GetModelOutputData();

    return SUCCESS;
}

Result ObjectDetect::Postprocess(cv::Mat& frame,
                                 aclmdlDataset* modelOutput)
{
    // Get box information data
    uint32_t dataSize = 0;
    float* detectData = (float*)GetInferenceOutputItem(dataSize, modelOutput,
                                                       g_bBoxDataBufId);
    if (detectData == nullptr) return FAILED;
    // Gets the number of boxes
    uint32_t* boxNum = (uint32_t*)GetInferenceOutputItem(dataSize, modelOutput,
                                                         g_boxNumDataBufId);
    if (boxNum == nullptr) return FAILED;

    // Number of boxes The first data is valid
    uint32_t totalBox = boxNum[0];
    float widthScale = (float)(frame.cols) / g_modelWidth_;
    float heightScale = (float)(frame.rows) / g_modelHeight_;

    vector<DetectionResult> detectResults;
    for (uint32_t i = 0; i < totalBox; i++) {
        DetectionResult oneResult;
        Point point_lt, point_rb;
        // get the confidence of the detected object. Anything less than 0.8 is considered invalid
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        uint32_t scoreLine = 80;
        if (score < scoreLine) continue;
        // get the frame coordinates and converts them to the coordinates on the original frame
        oneResult.lt.x = detectData[totalBox * TOPLEFTX + i] * widthScale;
        oneResult.lt.y = detectData[totalBox * TOPLEFTY + i] * heightScale;
        oneResult.rb.x = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
        oneResult.rb.y = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
        // Construct a string that marks the object: object name + confidence
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        oneResult.result_text = yolov3Label[objIndex] + std::to_string(score) + "\%";

        detectResults.emplace_back(oneResult);
    }
    // If it is the host side, the data is copied from the device and the memory used by the copy is freed
    if (g_runMode_ == ACL_HOST) {
        delete[]((uint8_t*)detectData);
        delete[]((uint8_t*)boxNum);
    }

    // Sends inference results and images to presenter Server for display
    SendImage(detectResults, frame);

    return SUCCESS;
}

void* ObjectDetect::GetInferenceOutputItem(uint32_t& itemDataSize,
                                           aclmdlDataset* inferenceOutput,
                                           uint32_t idx)
{
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(inferenceOutput, idx);
    if (dataBuffer == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer from model "
                  "inference output failed", idx);
        return nullptr;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer address "
                  "from model inference output failed", idx);
        return nullptr;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The %dth dataset buffer size of "
                  "model inference output is 0", idx);
        return nullptr;
    }

    void* data = nullptr;
    if (g_runMode_ == ACL_HOST) {
        data = Utils::CopyDataDeviceToLocal(dataBufferDev, bufferSize);
        if (data == nullptr) {
            ERROR_LOG("Copy inference output to host failed");
            return nullptr;
        }
    } else {
        data = dataBufferDev;
    }

    itemDataSize = bufferSize;
    return data;
}

void ObjectDetect::EncodeImage(vector<uint8_t>& encodeImg,
                                  cv::Mat& origImg)
{
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = 95; // default(95) 0-100
    // Jpeg images must serialize the Proto message before they can be sent
    cv::imencode(".jpg", origImg, encodeImg, param);
}

Result ObjectDetect::SendImage(vector<DetectionResult>& detectionResults,
                               cv::Mat& origImg)
{
    vector<uint8_t> encodeImg;
    EncodeImage(encodeImg, origImg);

    ImageFrame imageParam;
    imageParam.format = ImageFormat::kJpeg;
    imageParam.width = origImg.cols;
    imageParam.height = origImg.rows;
    imageParam.size = encodeImg.size();
    imageParam.data = reinterpret_cast<uint8_t*>(encodeImg.data());
    imageParam.detection_results = detectionResults;
    // Sends the detected object frame information and frame image to the Presenter Server for display
    PresenterErrorCode errorCode = PresentImage(g_channel_, imageParam);
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("PresentImage failed %d", static_cast<int>(errorCode));
        return FAILED;
    }

    return SUCCESS;
}

void ObjectDetect::DestroyResource()
{
    aclrtFree(g_imageDataBuf_);
    aclrtFree(g_imageInfoBuf_);

    delete g_channel_;
	
    // The ACL resource held by the model instance must be released before the ACL exits or ABORT will be torn down
    g_model_.DestroyResource();

    aclError ret = aclrtResetDevice(g_deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", g_deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}