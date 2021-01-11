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
#include <iostream>

#include "acl/acl.h"
#include "atlas_model.h"
#include "object_detect.h"
using namespace std;

namespace {
 const static std::vector<std::string> ssdLabel = { "background", "face"};

const uint32_t kBBoxDataBufId = 1;
const uint32_t kBoxNumDataBufId = 0;
uint32_t gSendNum = 0;


enum BBoxIndex { EMPTY = 0, LABEL,SCORE,TOPLEFTX,TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY};
}

static const std::string g_binFile = "output/flag.bin";

ObjectDetect::ObjectDetect(const char* modelPath,
                           uint32_t modelWidth,
                           uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
 modelHeight_(modelHeight), model_(modelPath), isInited_(false){
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

AtlasError ObjectDetect::InitResource() {
    /* 1. ACL初始化 */
    char aclConfigPath[32] = {'\0'};
    aclInit(aclConfigPath);

    /* 2. 运行管理资源申请,包括Device、Context、Stream */
    aclrtSetDevice(0);
    // create stream
    aclError ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl create stream failed\n");
        return ATLAS_ERROR_CREATE_STREAM;
    }
    ATLAS_LOG_INFO("create stream success");

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed\n");
        return ATLAS_ERROR_GET_RUM_MODE;
    }

    return ATLAS_OK;
}

AtlasError ObjectDetect::CreateImageInfoBuffer()
{
    const float imageInfo[4] = {(float)modelWidth_, (float)modelHeight_,
    (float)modelWidth_, (float)modelHeight_};
    imageInfoSize_ = sizeof(imageInfo);

    imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                     runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ATLAS_LOG_ERROR("Copy image info to device failed\n");
        return ATLAS_ERROR_COPY_DATA;
    }

    return ATLAS_OK;
}

AtlasError ObjectDetect::Init() {
    if (isInited_) {
        ATLAS_LOG_INFO("Objectdetect instance is initied already!\n");
        return ATLAS_OK;
    }
/*
    Channel* chan = nullptr;
    PresenterErrorCode openChannelret = OpenChannelByConfig(chan, "../data/param.conf");
    if (openChannelret != PresenterErrorCode::kNone) {
        ATLAS_LOG_ERROR("Open channel failed, error %d\n", (int)openChannelret);
        return ATLAS_ERROR;
    }
    chan_.reset(chan);
    printf("Open channel ok\n");
*/
    AtlasError ret = InitResource();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init acl resource failed, error: %d", ret);
        return ret;
    }

    ret = dvpp_.InitResource(stream_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init dvpp failed\n");
        return ret;
    }

    ret = model_.Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Model init failed, error:%d", ret);
        return ret;
    }

    ret = CreateImageInfoBuffer();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create image info buf failed\n");
        return ret;
    }

    isInited_ = true;
    return ATLAS_OK;
}

AtlasError ObjectDetect::Inference(vector<InferenceOutput>& inferenceOutput,
                                   ImageData& resizedImage) {
   AtlasError ret = model_.CreateInput(resizedImage.data.get(),
                                       resizedImage.size);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create mode input dataset failed, error:%d", ret);
        return ATLAS_ERROR;
    }

    ret = model_.Execute(inferenceOutput);
    if (ret != ATLAS_OK) {
        model_.DestroyInput();
        ATLAS_LOG_ERROR("Execute model inference failed, error: %d", ret);
        return ATLAS_ERROR;
    }
    model_.DestroyInput();

    return ATLAS_OK;
}

AtlasError ObjectDetect::Postprocess(ImageData& image, 
                                     vector<InferenceOutput>& modelOutput) {
    uint32_t dataSize = 0;
    float* detectData = (float *)modelOutput[kBBoxDataBufId].data.get();
    uint32_t* boxNum = (uint32_t *)modelOutput[kBoxNumDataBufId].data.get();

    uint32_t totalBox = boxNum[0];
    vector<DetectionResult> detectResults;

    for (uint32_t i = 0; i < totalBox; i++) {
        DetectionResult oneResult;
        Point point_lt, point_rb;
        uint32_t score = uint32_t(detectData[SCORE + i*8] * 100);
        if (score < 70)
            break ;

        point_lt.x = detectData[ TOPLEFTX + i*8] * image.width;
        point_lt.y = detectData[ TOPLEFTY + i*8] * image.height;
        point_rb.x = detectData[ BOTTOMRIGHTX + i*8] * image.width;
        point_rb.y = detectData[ BOTTOMRIGHTY + i*8] * image.height;

        uint32_t objIndex = (uint32_t)detectData[LABEL + i*8];
        oneResult.lt = point_lt;
        oneResult.rb = point_rb;
        oneResult.result_text = ssdLabel[objIndex] + std::to_string(score) + "\%";
        ATLAS_LOG_INFO("%d %d %d %d %d %s\n", objIndex,point_lt.x, point_lt.y,
                        point_rb.x, point_rb.y, oneResult.result_text.c_str());

        detectResults.emplace_back(oneResult);
    }
    //send to video_encoder
    AtlasError ret = encoder_.DoVencProcess(image);
    if(ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("video encode failed\n");
        return ATLAS_ERROR;
    }

    /*
    ImageData jpgImage;
    AtlasError ret = dvpp_.JpegE(jpgImage, image);
    if(ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Convert jpeg to yuv failed\n");
        return ATLAS_ERROR;
    }

    ImageData localImage;
    if (runMode_ == ACL_HOST) {
        AtlasError ret = CopyImageToLocal(localImage, jpgImage, runMode_);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Copy jpeg image to local failed");
            return ret;
        }
        printf("copy data to local\n");
        return SendImage(chan_.get(), localImage, detectResults);
    } else {
        return SendImage(chan_.get(), jpgImage, detectResults);
    }
    */

    return ATLAS_OK;
}

AtlasError ObjectDetect::Testprocess(ImageData& image) {
    //send to video_encoder
    AtlasError ret = encoder_.DoVencProcess(image);
    if(ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("video encode failed\n");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}
/*
void ObjectDetect::SaveBinFile(const string& filename, void* data, uint32_t size) {
    FILE *outFileFp = fopen(filename.c_str(), "wb+");
    if (outFileFp == nullptr) {
        ATLAS_LOG_ERROR("Save file %s failed for open error", filename.c_str());
        return;
    }
    fwrite(data, 1, size, outFileFp);
    fflush(outFileFp);
    fclose(outFileFp);
}
*/

AtlasError ObjectDetect::Process(ImageData& image) {
    /*
    ImageData resizedImage;

    //预处理图片:读取图片,讲图片缩放到模型输入要求的尺寸
    AtlasError ret = dvpp_.Resize(resizedImage, image, modelWidth_, modelHeight_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Resize image failed");
        return ret;
    }

    //将预处理的图片送入模型推理,并获取推理结果
    vector<InferenceOutput> inferOutput;
    ret = Inference(inferOutput, resizedImage);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Inference model inference output data failed");
        return ret;
    }

    //解析推理输出,并将推理得到的物体类别和位置标记到图片上
    ret = Postprocess(image, inferOutput);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Process model inference output data failed");
        return ret;
    }
    */
    //SaveBinFile(g_binFile.c_str(), image.data.get(), image.size);

    AtlasError ret = Testprocess(image);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Test Process model inference output data failed");
        return ret;
    }
    return ATLAS_OK;
}


void ObjectDetect::DestroyResource()
{
    dvpp_.DestroyResource();
    encoder_.DestroyResource();
    model_.DestroyResource();

    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("reset device failed\n");
    }
    ATLAS_LOG_INFO("end to reset device is %d\n", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("finalize acl failed\n");
    }
    ATLAS_LOG_INFO("end to finalize acl");
    aclrtFree(imageInfoBuf_);
}

AtlasError ObjectDetect::SendImage(Channel* channel,ImageData& jpegImage,vector<DetectionResult>& detRes) {
 /*   ImageFrame frame;
    frame.format = ImageFormat::kJpeg;
    frame.width = jpegImage.width;
    frame.height = jpegImage.height;
    frame.size = jpegImage.size;
    frame.data = jpegImage.data.get();
    frame.detection_results = detRes;

    PresenterErrorCode ret = PresentImage(channel, frame);
    // send to presenter failed
    if (ret != PresenterErrorCode::kNone) {
        ATLAS_LOG_ERROR("Send JPEG image to presenter failed, error %d\n", (int)ret);
        return ATLAS_ERROR;
    }

    ATLAS_LOG_INFO("Send JPEG image to presenter success, ret %d,num =%d \n", (int)ret,gSendNum++);*/
    return ATLAS_OK;
}

AtlasError ObjectDetect::Set(uint32_t width, uint32_t height) {
    AtlasError ret = encoder_.InitResource(width, height);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init video_encoder failed\n");
        return ret;
    }
    return ATLAS_OK;
}