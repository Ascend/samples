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
#pragma once

#include "atlas_utils.h"
#include "atlas_videocapture.h"

#define CAMERA_NUM     (2)
#define CAMERA(i) (g_CameraMgr.cap[i])


class Camera : public AtlasVideoCapBase {
public:
    Camera(uint32_t width, uint32_t height, uint32_t fps);
    Camera(uint32_t id, uint32_t width, uint32_t height, uint32_t fps);
    ~Camera(){};

    bool IsOpened();

    AtlasError Read(ImageData& frame);
    AtlasError Close();

    AtlasError Open();  
    bool IsAccessible(uint32_t id);
    AtlasError Set(StreamProperty key, int value);
    uint32_t Get(StreamProperty key);

private:
    bool IsValidWidth(int width);
    bool IsValidHeight(int height);
    bool IsValidFps(int fps);
    AtlasError SetProperty();
    CameraId GetOneAccessableSlot();

private:
    uint32_t id_;
    uint32_t width_;
    uint32_t height_;
    uint32_t size_;
    uint32_t fps_;
};




