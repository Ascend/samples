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
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "acllite_utils.h"
#include "camera.h"

using namespace std;

extern "C" {
#include "peripheral_api.h"
#include "camera.h"
CameraManager g_CameraMgr;

void HwInit() {
    if (!g_CameraMgr.hwInited) {
        MediaLibInit();
        g_CameraMgr.hwInited = 1;
    }   
}

int CameraInit(int id, int fps, int width, int height) {   
    Camera& cap = CAMERA(id);
    cap.frameSize = YUV420SP_SIZE(width, height);
    cap.id = id;
    cap.fps = fps;
    cap.width = width;
    cap.height = height;
    cap.inited = true;

    return ACLLITE_OK;
}

int ConfigCamera(int id, int fps, int width, int height) {
    int ret = SetCameraProperty(id, CAMERA_PROP_FPS, &fps);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ACLLITE_LOG_ERROR("Set camera fps failed");
        return ACLLITE_ERROR;
    }

    CameraResolution resolution;
    resolution.width = width;
    resolution.height = height;
    ret = SetCameraProperty(id, CAMERA_PROP_RESOLUTION, &resolution);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ACLLITE_LOG_ERROR("Set camera resolution failed");
        return ACLLITE_ERROR;
    }

    CameraCapMode mode = CAMERA_CAP_ACTIVE;
    ret = SetCameraProperty(id, CAMERA_PROP_CAP_MODE, &mode);
    if (ret == LIBMEDIA_STATUS_FAILED) {
        ACLLITE_LOG_ERROR("Set camera mode:%d failed", mode);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

int OpenCameraEx(int id, int fps, int width, int height) {
    if ((id < 0) || (id >= CAMERA_NUM)) {
        ACLLITE_LOG_ERROR("Open camera failed for invalid id %d", id);
        return ACLLITE_ERROR;
    }

    HwInit();

    CameraStatus status = QueryCameraStatus(id);
    if (status == CAMERA_STATUS_CLOSED){
        // Open Camera
        if (LIBMEDIA_STATUS_FAILED == OpenCamera(id)) {
            ACLLITE_LOG_ERROR("Camera%d closed, and open failed.", id);
            return ACLLITE_ERROR;
        }
    } else if (status != CAMERA_STATUS_OPEN) {
        //如果摄像头状态不是close状态也不是open状态,则认为摄像头异常
        ACLLITE_LOG_ERROR("Invalid camera%d status %d", id, status);
        return ACLLITE_ERROR;
    }

    //Set camera property
    if (ACLLITE_OK != ConfigCamera(id, fps, width, height)) {
        CloseCamera(id);
        ACLLITE_LOG_ERROR("Set camera%d property failed", id);
        return ACLLITE_ERROR;
    }

    if (!CAMERA(id).inited) {
        CameraInit(id, fps, width, height);
    }

    ACLLITE_LOG_INFO("Open camera %d success", id);

    return ACLLITE_OK;
}

int ReadCameraFrame(int id, CameraOutput& frame) {
    int size = CAMERA(id).frameSize;
    void* data = nullptr;
    auto aclRet = acldvppMalloc(&data, size);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl malloc dvpp data failed, dataSize %d, error %d", 
                      size, aclRet);
        return ACLLITE_ERROR;
    }

    int ret = ReadFrameFromCamera(id, (void*)data, (int *)&size);
    if ((ret == LIBMEDIA_STATUS_FAILED) || 
        (size != CAMERA(id).frameSize)) {
        acldvppFree(data);
        ACLLITE_LOG_ERROR("Get image from camera %d failed, size %d", id, size);
        return ACLLITE_ERROR;
    }
    frame.size = size;
    frame.data = (uint8_t*)data;

    return ACLLITE_OK;    
}

int CloseCameraEx(int cameraId) {
    if (LIBMEDIA_STATUS_FAILED == CloseCamera(cameraId)) {
        ACLLITE_LOG_ERROR("Close camera %d failed", cameraId);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

}
