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
#include "utils.h"


using namespace std;

extern "C" {
#include "driver/peripheral_api.h"
#include "camera.h"

Camera::Camera(uint32_t id, uint32_t fps, uint32_t width, 
               uint32_t height)
:id_(id), fps_(fps), width_(width), height_(height) {
    size_ = YUV420SP_SIZE(width_, height_);
    isAlign_ = (width%16 == 0) && (height%2 == 0);
    outBuf_ = new uint8_t[size_];
}

Camera::~Camera(){
    close(0);
    close(1);
    free(outBuf_);
    outBuf_ = nullptr;
}

Result Camera::SetProperty(int channelID) {
    int ret = SetCameraProperty(channelID, CAMERA_PROP_FPS, &(fps_));
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ERROR_LOG("Set camera %d fps failed",channelID);
        return FAILED;
    }

    int image_format = CAMERA_IMAGE_YUV420_SP;
    ret = SetCameraProperty(channelID, CAMERA_PROP_IMAGE_FORMAT, &image_format);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ERROR_LOG("Set camera image format to %d  channel %d failed",channelID, image_format);
        return FAILED;
    }

    CameraResolution resolution;
    resolution.width = width_;
    resolution.height = height_;
    ret = SetCameraProperty(channelID, CAMERA_PROP_RESOLUTION, &resolution);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ERROR_LOG("Set camera resolution failed channelID %d \n",channelID);
        return FAILED;
    }

    CameraCapMode mode = CAMERA_CAP_ACTIVE;
    ret = SetCameraProperty(channelID, CAMERA_PROP_CAP_MODE, &mode);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ERROR_LOG("Set camera mode:%d failed channel %d\n", mode,channelID);
        return FAILED;
    }

    return SUCCESS;
}

Result Camera::open(int channelID ) {

    MediaLibInit();
    CameraStatus status = QueryCameraStatus(channelID);

    if (status == CAMERA_STATUS_CLOSED){
        // Open Camera
        if (CAMERA_STATUS_OPEN != OpenCamera(channelID)) {
            ERROR_LOG("Camera%d closed, and open failed.\n", channelID);
            return FAILED;
        }
    }else if (status != CAMERA_STATUS_OPEN){
        ERROR_LOG("Invalid camera%d status %d\n", channelID, status);
        return FAILED;
    }

    //Set camera property
    if (SUCCESS != SetProperty(channelID)) {
        CloseCamera(channelID);
        ERROR_LOG("Set camera%d property failed\n", channelID);
        return FAILED;
    }
    INFO_LOG("Open camera %d success\n", channelID);
    isOpened_[channelID]=true;
    return SUCCESS;
}


bool Camera::IsOpened(int channelID){
    if(1 < channelID)
        return false;
    return isOpened_[channelID];
}

Result Camera::Read(int channelID, ImageData& output) {
    int frameSize = (int )size_;
    uint8_t* data =new uint8_t[frameSize];
    if ((frameSize == 0) || (data == nullptr)) {
        ERROR_LOG("Get image from camera %d failed for buffer is nullptr\n", id_);
        return FAILED;
    }

    int ret = ReadFrameFromCamera(channelID, data, (int *)&frameSize);
    if ((ret == LIBMEDIA_STATUS_FAILED)||((int )size_ != frameSize)) {
        ERROR_LOG("Get image from camera %d ,frameSize %d  failed", channelID,frameSize);
        delete[](data);
        return FAILED;
    }

    output.width = width_;
    output.height = height_;
    output.alignWidth = isAlign_? width_ : 0;
    output.alignHeight = isAlign_? height_ : 0;
    output.data = shared_ptr<uint8_t>(data, [](uint8_t* p) { delete[](p);});
    output.size = size_;
    return SUCCESS;
}

Result Camera::close(int channelID) {
    if (LIBMEDIA_STATUS_FAILED == CloseCamera(channelID)) {
        ERROR_LOG("Close camera %d failed\n", id_);
        return FAILED;
    }
    return SUCCESS;
}

}
