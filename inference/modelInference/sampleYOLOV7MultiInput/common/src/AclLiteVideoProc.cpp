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
#include "AclLiteVideoProc.h"
#include "VideoCapture.h"
#include "VideoWriter.h"
#ifdef ENABLE_BOARD_CAMARE
#include "CameraCapture.h"
#endif

using namespace std;

AclLiteVideoProc::AclLiteVideoProc():cap_(nullptr)
{
#ifdef ENABLE_BOARD_CAMARE
    int capWidth = 1280;
    int capHeight = 720;
    int capFps = 15;
    cap_ = new CameraCapture(capWidth, capHeight, capFps);
    Open();
#endif
}

AclLiteVideoProc::AclLiteVideoProc(uint32_t cameraId, uint32_t width,
                                   uint32_t height, uint32_t fps):cap_(nullptr)
{
#ifdef ENABLE_BOARD_CAMARE
    cap_ = new CameraCapture(cameraId, width, height, fps);
    Open();
#endif
}

AclLiteVideoProc::AclLiteVideoProc(const string& videoPath, int32_t deviceId, aclrtContext context)
{
    cap_ = new VideoCapture(videoPath, deviceId, context);
    Open();
}

AclLiteVideoProc::AclLiteVideoProc(VencConfig& vencConfig, aclrtContext context)
{
    cap_ = new VideoWriter(vencConfig, context);
    Open();
}

AclLiteVideoProc::~AclLiteVideoProc()
{
    if (cap_ != nullptr) {
        Close();
        delete cap_;
        cap_ = nullptr;
    }
}

bool AclLiteVideoProc::IsOpened()
{
    if (cap_ != nullptr) {
        return cap_->IsOpened();
    } else {
        return false;
    }
}

AclLiteError AclLiteVideoProc::Set(StreamProperty key, uint32_t value)
{
    if (cap_ != nullptr) {
        return cap_->Set(key, value);
    } else {
        return ACLLITE_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}

uint32_t AclLiteVideoProc::Get(StreamProperty key)
{
    if (cap_ != nullptr) {
        return cap_->Get(key);
    } else {
        return 0;
    }
}

AclLiteError AclLiteVideoProc::Read(ImageData& frame)
{
    if (cap_ != nullptr) {
        return cap_->Read(frame);
    } else {
        return ACLLITE_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}

AclLiteError AclLiteVideoProc::Close()
{
    if (cap_ != nullptr) {
        return cap_->Close();
    } else {
        return ACLLITE_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}

AclLiteError AclLiteVideoProc::Open()
{
    if (cap_ != nullptr) {
        return cap_->Open();
    } else {
        return ACLLITE_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}
