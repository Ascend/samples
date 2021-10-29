/**
 *  Copyright [2021] Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */

#ifndef __SAMPLE_LOOPER__
#define __SAMPLE_LOOPER__

#include "sample_api.h"
#include "sample_comm.h"

#ifdef HMEV_PLATFORM_SDK

class MsgHandler {
public:
    MsgHandler() {}

    virtual ~MsgHandler() {}

    virtual int32_t handle_message()
    {
        return HMEV_SUCCESS;
    }
    int32_t m_what = 0;
    void* m_obj;
    bool m_quit = false;
};

class HmevLooper {
public:
    HmevLooper(int32_t size, int32_t timeout);

    HmevLooper(int32_t size);

    HmevLooper(const char* looperName, int32_t size, int32_t timeout);

    HmevLooper(const char* looperName, int32_t size);

    virtual ~HmevLooper();

    int32_t add_fd(int32_t fd, MsgHandler* msg);

    int32_t remove_fd(int32_t fd);

    int32_t quit();

    int32_t do_quit();

    pthread_t m_thread;

private:
    int32_t create_epoll(int32_t size);

    static void* trampoline(void* p);

    void loop();

    std::string m_name;
    int32_t m_epoll_fd; // guarded by mLock but only modified on the looper thread
    int32_t m_epoll_size;
    int32_t m_timeout;
    bool m_polling;
    std::map <int32_t, std::shared_ptr<MsgHandler>> m_msg_map;
    mutable std::mutex m_lock;
};

#endif
#endif
