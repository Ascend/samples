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

#include "sample_looper.h"

#ifdef HMEV_PLATFORM_SDK

extern RunMode g_run_mode;
extern aclrtContext g_context;

#ifndef TEMP_FAILURE_RETRY
// Used to retry syscalls that can return EINTR.
#define TEMP_FAILURE_RETRY(exp) ({        \
    typeof (exp) rc;                      \
    do {                                  \
        rc = (exp);                       \
    } while (rc == -1 && errno == EINTR); \
    rc; })
#endif

HmevLooper::HmevLooper(int32_t size, int32_t timeout)
    : m_name("looper"), m_epoll_size(size + 2), m_timeout(timeout) // 2:2 more size
{
    create_epoll(m_epoll_size);
}

HmevLooper::HmevLooper(int32_t size) : m_name("looper"), m_epoll_size(size + 2), m_timeout(-1) // 2:2 more size
{
    create_epoll(m_epoll_size);
}

HmevLooper::HmevLooper(const char* looperName, int32_t size, int32_t timeout)
    : m_name(looperName), m_epoll_size(size + 2), m_timeout(timeout) // 2:2 more size
{
    create_epoll(m_epoll_size);
}

HmevLooper::HmevLooper(const char* looperName, int32_t size)
    : m_name(looperName), m_epoll_size(size + 2), m_timeout(-1) // 2:2 more size
{
    create_epoll(m_epoll_size);
}

HmevLooper::~HmevLooper()
{
    bool polling = true;
    {
        std::lock_guard<std::mutex> guardLock(m_lock);
        polling = m_polling;
    }
    if (polling) {
        quit();
    }
}

int32_t HmevLooper::create_epoll(int32_t size)
{
    int32_t ret;

    std::lock_guard<std::mutex> guardLock(m_lock);

    hi_mpi_sys_create_epoll(10, &m_epoll_fd);
    m_polling = true;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    ret = pthread_create(&m_thread, &attr, trampoline, this);
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "pthread_create failed");
        m_thread = 0;
        return HMEV_FAILURE;
    }

    return HMEV_SUCCESS;
}

void *HmevLooper::trampoline(void* p)
{
    ((HmevLooper*)p)->loop();
    return NULL;
}

// through epoll_wait monitor the fd of each encoding channel. If there is an encoding completion event,
// it can be captured and executed
void HmevLooper::loop()
{
    aclError aclRet = aclrtSetCurrentContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "set current context failed:%d", aclRet);
        return;
    }

    prctl(PR_SET_NAME, m_name.c_str(), 0, 0, 0);
    HMEV_HISDK_PRT(DEBUG, "m_epoll_fd:%d looper timeout %d", m_epoll_fd, m_timeout);
    hi_dvpp_epoll_event events[m_epoll_size];
    int32_t eventCount = 0;
    int32_t ret = 0;
    while (m_polling) {
        memset(events, 0, sizeof(events));

        ret = hi_mpi_sys_wait_epoll(m_epoll_fd, events, m_epoll_size, m_timeout, &eventCount);
        for (int i = 0; i < eventCount; i++) {
            hi_dvpp_epoll_event* ev = &events[i];
            int32_t fd = (int32_t)(unsigned long)ev->data;
            uint32_t epollEvents = ev->events;
            if ((epollEvents & HI_DVPP_EPOLL_IN) == 0 || fd < 0) {
                HMEV_HISDK_PRT(WARN, "epoll events is not epollin, or fd is invalid");
                continue;
            }
            std::shared_ptr <MsgHandler> msg = NULL;
            {
                std::lock_guard<std::mutex> guardLock(m_lock);
                if (m_msg_map.find(fd) != m_msg_map.end()) {
                    msg = m_msg_map[fd];
                }
            }
            if (msg != NULL) {
                msg->handle_message();
                msg = NULL;
            }
        }
    }
}

int32_t HmevLooper::do_quit()
{
    std::lock_guard<std::mutex> guardLock(m_lock);
    if (m_epoll_fd >= 0) {
        HMEV_HISDK_PRT(INFO, "close epoll");
        hi_mpi_sys_close_epoll(m_epoll_fd);
        m_epoll_fd = 0;
    }

    // free all msg
    m_msg_map.clear();

    return HMEV_SUCCESS;
}

int32_t HmevLooper::quit()
{
    {
        std::lock_guard<std::mutex> guardLock(m_lock);
        if (!m_polling) {
            return HMEV_FAILURE;
        }
        m_polling = false;
    }

    void* ret = NULL;
    if (m_thread) {
        pthread_join(m_thread, &ret);
    }

    do_quit();

    return HMEV_SUCCESS;
}

int32_t HmevLooper::add_fd(int32_t fd, MsgHandler* msg)
{
    int32_t ret;
    HMEV_HISDK_CHECK_RET_EXPRESS(fd < 0, "fd is invalid");

    std::lock_guard<std::mutex> guardLock(m_lock);
    HMEV_HISDK_CHECK_RET_EXPRESS(m_epoll_fd < 0, "mEpollFd is invalid");
    auto iter = m_msg_map.find(fd);
    HMEV_HISDK_CHECK_RET_EXPRESS(iter != m_msg_map.end(), "fd already added");
    std::shared_ptr <MsgHandler> msgHandler(msg);
    m_msg_map[fd] = msgHandler;

    hi_dvpp_epoll_event event;
    event.events = HI_DVPP_EPOLL_IN;
    event.data = (void*)(unsigned long)(fd);
    ret = hi_mpi_sys_ctl_epoll(m_epoll_fd, HI_DVPP_EPOLL_CTL_ADD, fd, &event);
    HMEV_HISDK_PRT(DEBUG, "mEpollFd:%d fd:%d", m_epoll_fd, fd);
    HMEV_HISDK_CHECK_RET_EXPRESS(ret == -1, "Could not add fd to epoll instance");
    return HMEV_SUCCESS;
}

int32_t HmevLooper::remove_fd(int32_t fd)
{
    int32_t ret;
    HMEV_HISDK_CHECK_RET_EXPRESS(fd < 0, "fd is invalid");

    std::lock_guard<std::mutex> guardLock(m_lock);
    HMEV_HISDK_CHECK_RET_EXPRESS(m_epoll_fd < 0, "mEpollFd is invalid");
    auto iter = m_msg_map.find(fd);
    HMEV_HISDK_CHECK_RET_EXPRESS(iter == m_msg_map.end(), "fd not exists");
    m_msg_map.erase(iter);

    ret = hi_mpi_sys_ctl_epoll(m_epoll_fd, HI_DVPP_EPOLL_CTL_DEL, fd, NULL);
    HMEV_HISDK_CHECK_RET_EXPRESS(ret != 0, "Could not remove fd from epoll instance");
    return HMEV_SUCCESS;
}

#endif
