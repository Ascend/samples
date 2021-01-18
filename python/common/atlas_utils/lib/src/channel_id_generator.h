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

#ifndef _DECODE_CHANNEL_H_
#define _DECODE_CHANNEL_H_

#include <unistd.h>
#include <mutex>

#define VIDEO_CHANNEL_MAX  16
#define INVALID_CHANNEL_ID -1


class ChannelIdGenerator {
public:
    ChannelIdGenerator() {
        for (int i = 0; i < VIDEO_CHANNEL_MAX; i++) {
            channelId_[i] = INVALID_CHANNEL_ID;
        }
    }
    ~ChannelIdGenerator(){};
 
    int GenerateChannelId(void) {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        for (int i = 0; i < VIDEO_CHANNEL_MAX; i++) {
            if (channelId_[i] == INVALID_CHANNEL_ID) {
                channelId_[i] = i;
                return i;
            }
        }

        return INVALID_CHANNEL_ID;
    }

    void ReleaseChannelId(int channelId) {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        if ((channelId >= 0) && (channelId < VIDEO_CHANNEL_MAX)) {
            channelId_[channelId] = INVALID_CHANNEL_ID;
        }
    }

private:
    int channelId_[VIDEO_CHANNEL_MAX];  
    mutable std::mutex mutex_lock_;
};



#endif /* VIDEO_DECODE_H_ */
