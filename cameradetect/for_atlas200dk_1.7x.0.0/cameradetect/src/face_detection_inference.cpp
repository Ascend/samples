#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>
#include "atlas_utils.h"
#include "face_detection.h"
#include "presenter/agent/presenter_channel.h"
#include "face_detection_inference.h"

using namespace std;

namespace {
    const uint32_t kImageInfoLength = 4;
}

FaceDetectionInference::FaceDetectionInference(Resolution& modelRes) {
    modelSize_.width = modelRes.width;
    modelSize_.height = modelRes.height;
    imageInfo_.size = 0;
    imageInfo_.data = nullptr;
}

FaceDetectionInference::~FaceDetectionInference() {
    delete dvppProc_;
    delete model_;
}

int FaceDetectionInference::Init() {
    dvppProc_ = new DvppProcess();
    model_ = new ModelProcess("../model/yolov3.om");
    postProcessThreadId_ = GetAtlasThreadIdByName("postprocess");

    float info[kImageInfoLength] = { (float)modelSize_.width, (float)modelSize_.height, (float)modelSize_.width, (float)modelSize_.height};
                                        
    uint32_t infoLen = kImageInfoLength * sizeof(float);
    void* buf = CopyDataDeviceToDevice((void*)info, infoLen);
    if (buf == nullptr) {
        ASC_LOG_ERROR("Create input of image info failed");
        return STATUS_ERROR;
    }

    imageInfo_.size = infoLen;
    imageInfo_.data = shared_ptr<void>(buf, [](void* p) { aclrtFree(p); });
    return STATUS_OK;
}

int FaceDetectionInference::ConvertFrameToJpeg(ImageData& jpeg, ImageData* yuvImage) {
    if (STATUS_OK != dvppProc_->CvtYuv420spToJpeg(&jpeg, yuvImage)) {
        ASC_LOG_ERROR("Convert yuv to jpeg failed for dvpp jpege error");
        return STATUS_ERROR;
    }
    printf("convert jpeg size %d\n", jpeg.size);
    void* data = CopyDataDeviceToNewBuf(jpeg.data.get(), jpeg.size);
    if (data == nullptr) {
        ASC_LOG_ERROR("Convert yuv to jpeg failed for copy data to host error");
        return STATUS_ERROR;
    }
    jpeg.data = shared_ptr<uint8_t>((uint8_t*)data, [](uint8_t* p) { delete[](p); });

    return STATUS_OK;
}

int FaceDetectionInference::FrameImageInference(shared_ptr<ImageData>& frame) {
    ASC_LOG_INFO("Get image to inference...");
  
    std::vector<DataBuffer> inputList;
    shared_ptr<DetectionData> detData = make_shared<DetectionData>();

    ImageData resizedImage;
    printf("process image, resize, format %d\n", frame->format);
    int ret = dvppProc_->Resize(&resizedImage, frame.get(), modelSize_);
    //SaveYuvImage(&resizedImage);
    if (ret != STATUS_OK) {
        ASC_LOG_ERROR("Resize failed");
        return STATUS_ERROR;
    }

    DataBuffer imageData;
    void* data = CopyDataDeviceToDevice(resizedImage.data.get(), resizedImage.size);

    imageData.data = shared_ptr<void>(data,  [](void* p) { aclrtFree(p); });
    imageData.size = resizedImage.size;
    inputList.emplace_back(imageData);
    inputList.emplace_back(imageInfo_);

    ret = model_->Execute(detData->output, inputList);
    if (ret != STATUS_OK) {
        ASC_LOG_ERROR("Excute mode inference failed");
        return STATUS_ERROR;
    }

    if (ConvertFrameToJpeg(detData->image, frame.get()))
        return STATUS_ERROR;

    printf("send inference result to postprocess, id %d", postProcessThreadId_);
    SendMessage(postProcessThreadId_, MSG_INFERENCE_RESULT,
                std::static_pointer_cast<void>(detData));

    return STATUS_OK;
}

int FaceDetectionInference::Process(int msgId, shared_ptr<void> msgData) {
    switch (msgId) {
        case MSG_VIDEO_FRAME_IMAGE:
        {
            printf("inference thread receive MSG_VIDEO_FRAME_IMAGE \n");
            shared_ptr<ImageData> image = static_pointer_cast<ImageData>(msgData);

            void* data = CopyDataDeviceToDvpp(image->data.get(), image->size);
            printf("FaceDetectionInference %d \n",image->size);
            image->data = SHARED_PRT_DVPP_BUF(data);
            FrameImageInference(image);
            break;
        }
        case MSG_VIDEO_DECODE_FINISH:
        {
            ASC_LOG_ERROR("All frames inference finish, exit!");
            SendMessage(postProcessThreadId_, MSG_VIDEO_DECODE_FINISH, nullptr);
            return STATUS_ERROR;
        }
        default:
        {
            ASC_LOG_ERROR("Inference thread receive unkonwn msg %d", msgId);
            break;
        }
    }

    return STATUS_OK;
}

void FaceDetectionInference::SaveJpegImage(ImageData* image) {
    static uint32_t jpg_cnt = 0;
    char filename[32];

    snprintf(filename, 32, "%d.jpg", ++jpg_cnt);

    void* buffer = CopyDataDeviceToDevice(image->data.get(), image->size);
    SaveBinFile(filename, (char*)buffer, image->size);
    aclrtFreeHost(buffer);
}

void FaceDetectionInference::SaveYuvImage(ImageData* image) {
    static uint32_t yuv_cnt = 0;
    char filename[32];

    snprintf(filename, 32, "%d.yuv", ++yuv_cnt);

    void* buffer = CopyDataDeviceToDevice(image->data.get(), image->size);
    SaveBinFile(filename, (char*)buffer, image->size);
    aclrtFreeHost(buffer);

}

