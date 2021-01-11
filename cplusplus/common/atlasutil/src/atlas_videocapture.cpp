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
#include "atlas_utils.h"
#include "atlas_videocapture.h"

#ifdef ENABLE_BOARD_CAMARE
#include "camera.h"
#endif

#include "video_decode.h"


using namespace std;

AtlasVideoCapture::AtlasVideoCapture():cap_(nullptr) {
#ifdef ENABLE_BOARD_CAMARE
    cap_ = new Camera(1280, 720, 20);
    Open();
#endif
}

AtlasVideoCapture::AtlasVideoCapture(uint32_t cameraId, uint32_t width, 
                                     uint32_t height, uint32_t fps)
:cap_(nullptr) {
#ifdef ENABLE_BOARD_CAMARE
    cap_ = new Camera(cameraId, width, height, fps);
    Open();
#endif
}

AtlasVideoCapture::~AtlasVideoCapture() {
    if (cap_ != nullptr) {
        Close();
        delete cap_;
        cap_ = nullptr;
    }    
}

AtlasVideoCapture::AtlasVideoCapture(const string& videoPath, aclrtContext context){
    cap_ = new VideoDecode(videoPath, context);
    Open();
}

bool AtlasVideoCapture::IsOpened() {
    if (cap_ != nullptr) {
        return cap_->IsOpened();
    } else {
        return false;
    }
}

AtlasError AtlasVideoCapture::Set(StreamProperty key, uint32_t value) {
    if (cap_ != nullptr) {
        return cap_->Set(key, value);
    } else {
        return ATLAS_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}

uint32_t AtlasVideoCapture::Get(StreamProperty key) {
    if (cap_ != nullptr) {
        return cap_->Get(key);
    } else {
        return 0;
    }
}

AtlasError AtlasVideoCapture::Read(ImageData& frame) {
    if (cap_ != nullptr) {
        return cap_->Read(frame);
    } else {
        return ATLAS_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}

AtlasError AtlasVideoCapture::Close() {
    if (cap_ != nullptr) {
        return cap_->Close();
    } else {
        return ATLAS_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}

AtlasError AtlasVideoCapture::Open() {
    if (cap_ != nullptr) {
        return cap_->Open();
    } else {
        return ATLAS_ERROR_UNSURPPORT_VIDEO_CAPTURE;
    }
}



