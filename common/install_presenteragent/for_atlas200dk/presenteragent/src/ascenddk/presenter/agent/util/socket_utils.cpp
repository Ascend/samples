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

#include "ascenddk/presenter/agent/util/socket_utils.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <signal.h>
#include <unistd.h>

#include "ascenddk/presenter/agent/util/logging.h"

namespace {

// no flag is need for now
const int kSocketFlagNone = 0;
  
// socket closed
const int kSocketClosed = 0;

// connect timeout
const int kDefaultTimeoutInSec = 3;

const int kSocketSuccess = 0;

// indicating invalid socket file descriptor
const int kSocketFdNull = -1;

const int kReuseAddress = 1;

}

namespace ascend {
namespace presenter {
namespace socketutils {

// set blocking mode
void SetNonBlocking(int socket, bool nonblocking) {
  // get original mask
  long mask = fcntl(socket, F_GETFL, NULL);
  if (nonblocking) {
    mask |= O_NONBLOCK;  // set nonblocking
  } else {
    mask &= ~O_NONBLOCK;  // unset nonblocking
  }
  (void) fcntl(socket, F_SETFL, mask);
}

bool SetSockAddr(const char *host_ip, uint16_t port, sockaddr_in &addr) {
  // valid port is 1~65535
  if (port == 0) {
    AGENT_LOG_ERROR("Invalid port: %d", port);
    return false;
  }

  // convert host and port to sockaddr
  memset(&addr, 0, sizeof(addr));
 
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host_ip, &addr.sin_addr) <= 0) {
    AGENT_LOG_ERROR("Invalid host IP: %s", host_ip);
    return false;
  }

  return true;
}

void SetSocketReuseAddr(int socket) {
  // set reuse address
  int so_reuse = kReuseAddress;
  int ret = setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &so_reuse,
                       sizeof(so_reuse));
  if (ret != kSocketSuccess) {
    AGENT_LOG_WARN("set socket opt SO_REUSEADDR failed");
  }
}

void SetSocketTimeout(int socket, int timeout_in_sec) {
  // initialize timeout
  timeval timeout = { timeout_in_sec, 0 };

  // set write timeout
  int ret = setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                       sizeof(timeout));
  if (ret != kSocketSuccess) {
    AGENT_LOG_WARN("set socket opt SO_SNDTIMEO failed");
  }

  // set read timeout
  ret = setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  if (ret != kSocketSuccess) {
    AGENT_LOG_WARN("set socket opt SO_RCVTIMEO failed");
  }
}

int CreateSocket() {
  return ::socket(AF_INET, SOCK_STREAM, 0);
}

int Connect(int socket, const sockaddr_in& addr) {
  // Ignore SIGPIPE signals
  signal(SIGPIPE, SIG_IGN);

  // set nonblocking and connect with timeout
  SetNonBlocking(socket, true);

  // do connect
  int ret = ::connect(socket, (sockaddr*) &addr, sizeof(addr));
  if (ret < 0) {
    if (errno != EINPROGRESS) {
      AGENT_LOG_ERROR("connect() error: %s", strerror(errno));
      return kSocketError;
    }

    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(socket, &fdset);
    // connect timeout = 3 seconds
    timeval tv = { kDefaultTimeoutInSec, 0 };
    int select_ret = select(socket + 1, NULL, &fdset, NULL, &tv);
    if (select_ret < 0) {  // error
      AGENT_LOG_ERROR("select() error: %s", strerror(errno));
      return kSocketError;
    }

    if (select_ret == 0) {  // no FD is ready
      AGENT_LOG_ERROR("select() timeout");
      return kSocketError;
    }

    int so_error = kSocketError;
    socklen_t len = sizeof(so_error);
    getsockopt(socket, SOL_SOCKET, SO_ERROR, &so_error, &len);
    if (so_error != kSocketSuccess) {
      AGENT_LOG_ERROR("getsockopt() error: %d", so_error);
      return kSocketError;
    }
  }

  // reset to blocking mode
  SetNonBlocking(socket, false);
  return kSocketSuccess;
}

int ReadN(int socket, char *buffer, int size) {
  int received_cnt = 0;
  // keep reading until nReceived == size
  while (received_cnt < size) {
    char *write_ptr = buffer + received_cnt;
    int ret = ::recv(socket, write_ptr, size - received_cnt, kSocketFlagNone);  // [false alarm]: will never write over size
    if (ret == kSocketError) {
      if (errno == EAGAIN || errno == EINTR) {
        AGENT_LOG_INFO("recv() timeout. error = %s", strerror(errno));
        return kSocketTimeout;
      }

      AGENT_LOG_ERROR("recv() error. error = %s", strerror(errno));
      return kSocketError;
    }
    
    if (ret == kSocketClosed) {
      AGENT_LOG_ERROR("socket closed");
      return kSocketError;
    }

    received_cnt += ret;
  }

  return received_cnt;
}

int WriteN(int socket, const char *data, int size) {
  int sent_cnt = 0;
  // keep reading until nReceived == size
  while (sent_cnt < size) {
    const char *read_ptr = data + sent_cnt;
    int ret = ::send(socket, read_ptr, size - sent_cnt, kSocketFlagNone);
    if (ret == kSocketError) {
      AGENT_LOG_ERROR("send() error. errno = %s", strerror(errno));
      return kSocketError;
    }
    
    if (ret == kSocketClosed) {
      AGENT_LOG_ERROR("socket closed");
      return kSocketError;
    }

    sent_cnt += ret;
  }

  return sent_cnt;
}

void CloseSocket(int &socket) {
  if (socket >= 0) {
    (void) close(socket);
    socket = kSocketFdNull;
  }
}

} /* namespace sockutil */
} /* namespace presenter */
} /* namespace ascend */

