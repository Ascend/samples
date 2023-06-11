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

#ifndef VIDEO_ENCODE_H
#define VIDEO_ENCODE_H

#include <dirent.h>
#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <thread>
#include "ThreadSafeQueue.h"
#include "VencHelper.h"
#include "AclLiteVideoProc.h"

class VideoWriter : public AclLiteVideoCapBase {
public:
    /**
     * @brief VideoWriter constructor
     */
    VideoWriter(VencConfig& vencConfig, aclrtContext context = nullptr);

    /**
     * @brief VideoWriter destructor
     */
    ~VideoWriter();
    bool IsOpened();
    uint32_t Get(StreamProperty key);
    AclLiteError Set(StreamProperty key, int value);
    AclLiteError Read(ImageData& image);
    AclLiteError SetAclContext();
    AclLiteError Close();
    AclLiteError Open();
    void DestroyResource();

private:
    AclLiteError InitResource();
    AclLiteError SetImageFormat(uint32_t format);
    AclLiteError SetStreamFormat(uint32_t format);

private:
    bool isReleased_;
    VencStatus status_;
    aclrtContext context_;
    aclrtRunMode runMode_;
    VencHelper* dvppVenc_;
    VencConfig vencInfo_;
};

#endif
