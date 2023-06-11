/**
 * ============================================================================
 *
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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
#include <cstdio>
#include <cstdarg>
#include <ctime>
#include <memory>
#include <sys/time.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include "AclLiteUtils.h"
extern "C" {
#include "peripheral_api.h"
#include "CameraCapture.h"

using namespace std;
CameraResolution gCameraResTbl[] = {
    {1920, 1080}, {1280, 720},
    {704, 576}, {704, 288},
    {352, 288}
};

CameraCapture::CameraCapture(uint32_t width, uint32_t height, uint32_t fps)
    :width_(width), height_(height),
    size_(YUV420SP_SIZE(width_, height_)), fps_(fps)
{
    MediaLibInit();
    if (IsAccessible(CAMERA_ID_0)) {
        id_ = CAMERA_ID_0;
    } else if (IsAccessible(CAMERA_ID_1)) {
        id_ = CAMERA_ID_1;
    } else {
        id_ = CAMERA_ID_INVALID;
        ACLLITE_LOG_ERROR("No camera accessable in device");
    }

    if (id_ != CAMERA_ID_INVALID)
        ACLLITE_LOG_INFO("No specified carmera id, use camera%d", id_);
}

CameraCapture::CameraCapture(uint32_t id, uint32_t width, uint32_t height, uint32_t fps)
    :id_(id), width_(width), height_(height),
    size_(YUV420SP_SIZE(width_, height_)), fps_(fps)
{
    MediaLibInit();
}

bool CameraCapture::IsAccessible(uint32_t id)
{
    CameraStatus status = QueryCameraStatus(id);
    if (status == CAMERA_STATUS_OPEN) {
        return true;
    }

    if (status == CAMERA_STATUS_CLOSED) {
        if (LIBMEDIA_STATUS_FAILED == OpenCamera(id)) {
            ACLLITE_LOG_ERROR("Open camera %d failed when test accessable", id);
            return false;
        }
        if (LIBMEDIA_STATUS_FAILED == CloseCamera(id)) {
            ACLLITE_LOG_ERROR("Close camera %d failed when test accessable", id);
            return false;
        }
        return true;
    }

    return false;
}

bool CameraCapture::IsValidWidth(int width)
{
    for (uint32_t i = 0; i < SIZEOF_ARRAY(gCameraResTbl); i++) {
        if (gCameraResTbl[i].width == width)
            return true;
    }

    return false;
}

bool CameraCapture::IsValidHeight(int height)
{
    for (uint32_t i = 0; i < SIZEOF_ARRAY(gCameraResTbl); i++) {
        if (gCameraResTbl[i].height == height)
            return true;
    }

    return false;
}

bool CameraCapture::IsValidFps(int fps)
{
    int minFps = 0;
    int maxFps = 20;
    return (fps > minFps) && (fps <= maxFps);
}

AclLiteError CameraCapture::SetProperty()
{
    int ret = SetCameraProperty(id_, CAMERA_PROP_FPS, &(fps_));
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ACLLITE_LOG_ERROR("Set camera fps failed");
        return ACLLITE_ERROR_SET_CAMERA;
    }

    CameraResolution resolution;
    resolution.width = width_;
    resolution.height = height_;

    ret = SetCameraProperty(id_, CAMERA_PROP_RESOLUTION, &resolution);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ACLLITE_LOG_ERROR("Set camera resolution failed");
        return ACLLITE_ERROR_SET_CAMERA;
    }

    CameraCapMode mode = CAMERA_CAP_ACTIVE;
    ret = SetCameraProperty(id_, CAMERA_PROP_CAP_MODE, &mode);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ACLLITE_LOG_ERROR("Set camera mode:%d failed", mode);
        return ACLLITE_ERROR_SET_CAMERA;
    }

    return ACLLITE_OK;
}

AclLiteError CameraCapture::Open()
{
    if (id_ == CAMERA_ID_INVALID) {
        ACLLITE_LOG_ERROR("No camera is accessiable");
        return ACLLITE_ERROR_CAMERA_NO_ACCESSABLE;
    }

    CameraStatus status = QueryCameraStatus(id_);
    if ((status == CAMERA_NOT_EXISTS) || (status == CAMERA_STATUS_UNKOWN)) {
        ACLLITE_LOG_ERROR("Camera %d status is error %d", id_, status);
        return ACLLITE_ERROR_CAMERA_NO_ACCESSABLE;
    }

    if ((status != CAMERA_STATUS_OPEN) &&
        (LIBMEDIA_STATUS_FAILED == OpenCamera(id_))) {
        ACLLITE_LOG_ERROR("Open camera %d failed.", id_);
        return ACLLITE_ERROR_OPEN_CAMERA;
    }

    AclLiteError ret = SetProperty();
    if (ret != ACLLITE_OK) {
        Close();
        ACLLITE_LOG_ERROR("Set camera%d property failed", id_);
        return ret;
    }
    ACLLITE_LOG_INFO("Open camera %d success", id_);

    return ACLLITE_OK;
}

bool CameraCapture::IsOpened()
{
    if (id_ == CAMERA_ID_INVALID) {
        return false;
    }

    return (CAMERA_STATUS_OPEN == QueryCameraStatus(id_));
}

AclLiteError CameraCapture::Read(ImageData& image)
{
    if (id_ == CAMERA_ID_INVALID) {
        return ACLLITE_ERROR_CAMERA_NO_ACCESSABLE;
    }
    
    int size = (int)size_;
    void* buffer = nullptr;
    aclError aclRet = acldvppMalloc(&buffer, size);
    if ((aclRet != ACL_SUCCESS) || (buffer == nullptr)) {
        ACLLITE_LOG_ERROR("Malloc dvpp memory failed, error:%d", aclRet);
        return ACLLITE_ERROR_MALLOC_DVPP;
    }

    int ret = ReadFrameFromCamera(id_, buffer, &size);
    if ((ret == LIBMEDIA_STATUS_FAILED) || (size != (int)size_)) {
        ACLLITE_LOG_ERROR("Get image from camera %d failed, size %d", id_, size);
        return ACLLITE_ERROR_READ_CAMERA_FRAME;
    }
   
    image.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    image.width = width_;
    image.height = height_;
    image.alignWidth = width_;
    image.alignHeight = height_;
    image.size = (uint32_t)size_;
    image.data = SHARED_PTR_DVPP_BUF(buffer);

    return ACLLITE_OK;
}

uint32_t CameraCapture::Get(StreamProperty key)
{
    uint32_t value = 0;

    switch (key) {
        case FRAME_WIDTH:
            value = width_;
            break;
        case FRAME_HEIGHT:
            value = height_;
            break;
        case VIDEO_FPS:
            value = fps_;
            break;
        default:
            ACLLITE_LOG_ERROR("Unsurpport property %d to get for camera", key);
            break;
    }

    return value;
}

AclLiteError CameraCapture::Set(StreamProperty key, int value)
{
    AclLiteError ret = ACLLITE_OK;

    switch (key) {
        case FRAME_WIDTH:
            {
                if (IsValidWidth(value)) {
                    width_ = value;
                } else {
                    ret = ACLLITE_ERROR_INVALID_PROPERTY_VALUE;
                }
                break;
            }
        case FRAME_HEIGHT:
            {
                if (IsValidHeight(value)) {
                    height_ = value;
                } else {
                    ret = ACLLITE_ERROR_INVALID_PROPERTY_VALUE;
                }
                break;
            }
        case VIDEO_FPS:
            {
                if (IsValidFps(value)) {
                    fps_ = value;
                } else {
                    ret = ACLLITE_ERROR_INVALID_PROPERTY_VALUE;
                }
                break;
            }
        default:
            {
                ret = ACLLITE_ERROR_UNSURPPORT_PROPERTY;
                ACLLITE_LOG_ERROR("Unsurpport property %d to set for camera", key);
                break;
            }
    }

    return ret;
}

AclLiteError CameraCapture::Close()
{
    if (id_ == CAMERA_ID_INVALID) {
        return ACLLITE_ERROR_CAMERA_NO_ACCESSABLE;
    }

    if (LIBMEDIA_STATUS_FAILED == CloseCamera(id_)) {
        ACLLITE_LOG_ERROR("Close camera %d failed", id_);
    }

    return ACLLITE_OK;
}
}
