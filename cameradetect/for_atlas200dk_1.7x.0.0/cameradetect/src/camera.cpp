/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <memory>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "face_detection.h"


using namespace std;

extern "C" {
#include "driver/peripheral_api.h"
#include "camera.h"

Camera::Camera(uint32_t id, uint32_t fps, uint32_t width, 
               uint32_t height, bool isSendToDevice)
:id_(id), fps_(fps), width_(width), height_(height), 
isSendToDevice_(true), outBuf_(nullptr) {
    size_ = YUV420SP_SIZE(width_, height_);
    isAlign_ = (width%32 == 0) && (height%2 == 0);
    outBuf_ = new uint8_t[size_];
    isOpened_ = Open();
}

Camera::~Camera(){
    Close();
    free(outBuf_);
    outBuf_ = nullptr;
}

int Camera::ReadMsgProcess() {
    shared_ptr<ImageData> frame = make_shared<ImageData>();

    int status = STATUS_OK;
    status = Read(*(frame.get()));
    printf("Camera::ReadMsgProcess %d ", status);
    if (status == STATUS_OK) {
        SendMessage(inferThreadId, MSG_VIDEO_FRAME_IMAGE,
        std::static_pointer_cast < void >(frame));
    }
    else
    {
        return status;
    }
    usleep(10000);
    SendMessage(myThreadId, MSG_START_READ_VIDEO, nullptr);
    printf("SendMessage %d ", MSG_START_READ_VIDEO);
    return STATUS_OK;
}

int Camera::Process(int msgId, shared_ptr<void> msgData) {
    int  ret = STATUS_ERROR;
     if(false == isOpened())
     {
         if(STATUS_OK != Open()){
             ASC_LOG_ERROR("Open camera %d failed", id_);
             return STATUS_ERROR;
         }
     }

    if (msgId == MSG_START_READ_VIDEO) {
         ret = ReadMsgProcess();
    }

    return ret;
}


int Camera::SetProperty() {
    int ret = SetCameraProperty(id_, CAMERA_PROP_FPS, &(fps_));
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ASC_LOG_ERROR("Set camera fps failed");
        return STATUS_ERROR;
    }

    int image_format = CAMERA_IMAGE_YUV420_SP;
    ret = SetCameraProperty(id_, CAMERA_PROP_IMAGE_FORMAT, &image_format);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ASC_LOG_ERROR("Set camera image format to %d failed", image_format);
        return STATUS_ERROR;
    }

    CameraResolution resolution;
    resolution.width = width_;
    resolution.height = height_;
    ret = SetCameraProperty(id_, CAMERA_PROP_RESOLUTION,    &resolution);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ASC_LOG_ERROR("Set camera resolution failed");
        return STATUS_ERROR;
    }

    CameraCapMode mode = CAMERA_CAP_ACTIVE;
    ret = SetCameraProperty(id_, CAMERA_PROP_CAP_MODE, &mode);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ASC_LOG_ERROR("Set camera mode:%d failed", mode);
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

int Camera::Open() {

    MediaLibInit();
    CameraStatus status = QueryCameraStatus(id_);

    printf("open camera %d QueryCameraStatus %d", id_,status);
    if (status == CAMERA_STATUS_CLOSED){
        // Open Camera
        if (LIBMEDIA_STATUS_FAILED == OpenCamera(id_)) {
            ASC_LOG_ERROR("Camera%d closed, and open failed.", id_);
            return STATUS_ERROR;
        }
    }else if (status != CAMERA_STATUS_OPEN){
        ASC_LOG_ERROR("Invalid camera%d status %d", id_, status);
        return STATUS_ERROR;
    }

    //Set camera property
    if (STATUS_OK != SetProperty()) {
        CloseCamera(id_);
        ASC_LOG_ERROR("Set camera%d property failed", id_);
        return STATUS_ERROR;
    }
    isOpened_ = true;
    printf("Open camera %d success", id_);
    return STATUS_OK;
}


bool Camera::isOpened(){
    return isOpened_;
}

int Camera::Read(ImageData& output) {

    uint32_t size = 0;
    void* data =  nullptr;
    printf("Camera::Read %d \n",size_);
    if (isSendToDevice_) {
        size = size_;
        data = outBuf_;
    } else {
        size = YUV420SP_SIZE(width_, height_);
        uint8_t* buf =new uint8_t[size];
        output.data = shared_ptr<uint8_t>((uint8_t *)(buf),[](uint8_t* p) { delete[](p);});
        data = output.data.get();
    }

    if ((size == 0) || (data == nullptr)) {
        ASC_LOG_ERROR("Get image from camera %d failed for buffer is nullptr", id_);
        return STATUS_ERROR;
    }

    int ret = ReadFrameFromCamera(id_, data, (int *)&size);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ASC_LOG_ERROR("Get image from camera %d failed", id_);
        return STATUS_ERROR;
    }
    printf("Read frame size_ %d ok\n",size_);

    output.isAligned = isAlign_;
    output.width = width_;
    output.height = height_;
    output.alignWidth = isAlign_? width_ : 0;
    output.alignHeight = isAlign_? height_ : 0;
    output.size = size_;

    if (isSendToDevice_) {
       // void* dvppBuf = CopyDataDeviceToDvpp(data, size);
       // output.data = SHARED_PRT_DVPP_BUF(dvppBuf);
        output.data = SHARED_PRT_DVPP_BUF(data);
    }
    return STATUS_OK;   
}

void Camera::Close() {
    if (LIBMEDIA_STATUS_FAILED == CloseCamera(id_)) {
        ASC_LOG_ERROR("Close camera %d failed", id_);
    }
}

}
