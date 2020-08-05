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

* File sample_process.cpp
* Description: handle acl resource
*/
#include "object_detect.h"
#include <iostream>
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
"refrigerator", "book", "clock", "vase","scissors",
"teddy bear", "hair drier", "toothbrush" };
//推理输出dataset中下标为0的单元为检测框信息数据
const uint32_t kBBoxDataBufId = 0;
//下标为1的单元为框个数数据
const uint32_t kBoxNumDataBufId = 1;
//框信息中各个字段下标
enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
}

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
                           uint32_t modelHeight)
:deviceId_(0), imageDataBuf_(nullptr), imageInfoBuf_(nullptr), modelWidth_(modelWidth),
modelHeight_(modelHeight), channel_(nullptr), isInited_(false){
    modelPath_ = modelPath;
    imageDataSize_ = RGBU8_IMAGE_SIZE(modelWidth, modelHeight);
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

Result ObjectDetect::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("Open device %d success", deviceId_);
    //获取当前应用程序运行在host还是device
    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::InitModel(const char* omModelPath) {
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


    return SUCCESS;
}

Result ObjectDetect::CreateModelInputdDataset()
{
    //申请输入模型的图像数据内存
    aclError aclRet = aclrtMalloc(&imageDataBuf_, imageDataSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return FAILED;
    }
    //yolov3的第二个输入,为输入图像宽高参数
    const float imageInfo[4] = {(float)modelWidth_, (float)modelHeight_,
    (float)modelWidth_, (float)modelHeight_};
    imageInfoSize_ = sizeof(imageInfo);
    if (runMode_ == ACL_HOST)
        imageInfoBuf_ = Utils::CopyDataHostToDevice((void *)imageInfo, imageInfoSize_);
    else
        imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *)imageInfo, imageInfoSize_);
    if (imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed");
        return FAILED;
    }
    //使用申请的内存创建模型输入dataset.创建后每帧推理只更新内存数据即可,不用每次创建输入dataset
    Result ret = model_.CreateInput(imageDataBuf_, imageDataSize_,
    imageInfoBuf_, imageInfoSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::OpenPresenterChannel() {
    OpenChannelParam param;
    param.host_ip = "192.168.0.194";  //IP address of Presenter Server
    param.port = 7006;  //port of present service
    param.channel_name = "video";
    param.content_type = ContentType::kVideo;  //content type is Video
    INFO_LOG("OpenChannel start");
    PresenterErrorCode errorCode = OpenChannel(channel_, param);
    INFO_LOG("OpenChannel param");
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("OpenChannel failed %d", static_cast<int>(errorCode));
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Init() {
    //如果已经初始化,则直接返回
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }
    //初始化ACL资源
    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }
    //初始化模型管理实例
    ret = InitModel(modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }
	
    ret = CreateModelInputdDataset();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed");
        return FAILED;
    }	
	
    //连接presenter server
    ret = OpenPresenterChannel();
    if (ret != SUCCESS) {
        ERROR_LOG("Open presenter channel failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result ObjectDetect::Preprocess(cv::Mat& frame) {
    //将帧图像缩放到模型要求大小
    cv::Mat reiszeMat;
    cv::resize(frame, reiszeMat, cv::Size(modelWidth_, modelHeight_));
    if (reiszeMat.empty()) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }
    //将数据拷贝到输入dataset的缓存中
    aclrtMemcpyKind policy = (runMode_ == ACL_HOST)?
                             ACL_MEMCPY_HOST_TO_DEVICE:ACL_MEMCPY_DEVICE_TO_DEVICE;
    aclError ret = aclrtMemcpy(imageDataBuf_, imageDataSize_,
                               reiszeMat.ptr<uint8_t>(), imageDataSize_, policy);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Copy resized image data to device failed.");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Inference(aclmdlDataset*& inferenceOutput) {
    //执行推理
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }
    //获取推理输出
    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result ObjectDetect::Postprocess(cv::Mat& frame,
                                    aclmdlDataset* modelOutput){
    //获取框信息数据
    uint32_t dataSize = 0;
    float* detectData = (float*)GetInferenceOutputItem(dataSize, modelOutput,
                                                       kBBoxDataBufId);
    if (detectData == nullptr) return FAILED;
    //获取框个数信息
    uint32_t* boxNum = (uint32_t*)GetInferenceOutputItem(dataSize, modelOutput,
                                                         kBoxNumDataBufId);
    if (boxNum == nullptr) return FAILED;

    //框个数数据第一个数据是有效的
    uint32_t totalBox = boxNum[0];
    //框信息中框坐标是基于送入模型图片尺寸的,转换到原始帧图片上需要乘以缩放系数
    float widthScale = (float)(frame.cols) / modelWidth_;
    float heightScale = (float)(frame.rows) / modelHeight_;

    vector<DetectionResult> detectResults;
    for (uint32_t i = 0; i < totalBox; i++) {
        DetectionResult oneResult;
        Point point_lt, point_rb;
        //获取检测到的物体的置信度,低于0.8的认为无效
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < 80) continue;
        printf("score %d", score);
        //获取框坐标信息并转换为原始帧上的坐标
        oneResult.lt.x = detectData[totalBox * TOPLEFTX + i] * widthScale;
        oneResult.lt.y = detectData[totalBox * TOPLEFTY + i] * heightScale;
        oneResult.rb.x = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
        oneResult.rb.y = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
        //构造标记物体的字符串:物体名称+置信度
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        oneResult.result_text = yolov3Label[objIndex] + std::to_string(score) + "\%";
        printf("%d %d %d %d %s\n", oneResult.lt.x, oneResult.lt.y,
        oneResult.rb.x, oneResult.rb.y, oneResult.result_text.c_str());

        detectResults.emplace_back(oneResult);
    }
    //如果是host侧,则数据是从device拷贝过来的,拷贝使用的内存要释放
    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t*)detectData);
        delete[]((uint8_t*)boxNum);
    }

    //将推理结果和图像发给presenter server显示
    SendImage(detectResults, frame);

    return SUCCESS;
}

void* ObjectDetect::GetInferenceOutputItem(uint32_t& itemDataSize,
                                           aclmdlDataset* inferenceOutput,
                                           uint32_t idx) {

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
    if (runMode_ == ACL_HOST) {
        data = Utils::CopyDataDeviceToLocal(dataBufferDev, bufferSize);
        if (data == nullptr) {
            ERROR_LOG("Copy inference output to host failed");
            return nullptr;
        }
    }
    else {
        data = dataBufferDev;
    }

    itemDataSize = bufferSize;
    return data;
}

void ObjectDetect::EncodeImage(vector<uint8_t>& encodeImg,
                                  cv::Mat& origImg) {
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = 95;//default(95) 0-100
    //jpeg图片必须序列化proto消息才能发送
    cv::imencode(".jpg", origImg, encodeImg, param);
}

Result ObjectDetect::SendImage(vector<DetectionResult>& detectionResults,
                                  cv::Mat& origImg) {
    vector<uint8_t> encodeImg;
    EncodeImage(encodeImg, origImg);

    ImageFrame imageParam;
    imageParam.format = ImageFormat::kJpeg;
    imageParam.width = origImg.cols;
    imageParam.height = origImg.rows;
    imageParam.size = encodeImg.size();
    imageParam.data = reinterpret_cast<uint8_t*>(encodeImg.data());
    imageParam.detection_results = detectionResults;
    //将检测到的物体框信息和帧图片发给presenter server显示
    PresenterErrorCode errorCode = PresentImage(channel_, imageParam);
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("PresentImage failed %d", static_cast<int>(errorCode));
        return FAILED;
    }

    return SUCCESS;
}

void ObjectDetect::DestroyResource()
{
    //模型实例占用的acl资源必须在acl退出前释放,否则会报abort
    model_.DestroyResource();

    aclError ret;
    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
    aclrtFree(imageDataBuf_);
    aclrtFree(imageInfoBuf_);

    delete channel_;
}
