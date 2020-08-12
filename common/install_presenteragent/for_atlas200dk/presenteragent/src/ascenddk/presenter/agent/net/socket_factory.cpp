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

#include "ascenddk/presenter/agent/net/socket_factory.h"

#include <cstring>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "ascenddk/presenter/agent/errors.h"
#include "ascenddk/presenter/agent/util/logging.h"
#include "ascenddk/presenter/agent/util/socket_utils.h"

using namespace std;

namespace ascend {
namespace presenter {

// anonymous namespace for constants
namespace {

// Default Socket Timeout
const int kDefaultTimeoutInSec = 3;

} /* anonymous namespace */

PresenterErrorCode SocketFactory::GetErrorCode() const {
  return error_code_;
}

void SocketFactory::SetErrorCode(PresenterErrorCode error_code) {
  this->error_code_ = error_code;
}

// common function for creating a socket with given hostIp and port
int SocketFactory::CreateSocket(const string& host_ip, uint16_t port) {
  // parse address
  sockaddr_in addr;
  if (!socketutils::SetSockAddr(host_ip.c_str(), port, addr)) {
    AGENT_LOG_ERROR("Invalid address: %s:%d", host_ip.c_str(), port);
    SetErrorCode(PresenterErrorCode::kInvalidParam);
    return socketutils::kSocketError;
  }

  // create socket file descriptor
  int sock = socketutils::CreateSocket();
  if (sock == socketutils::kSocketError) {
    AGENT_LOG_ERROR("socket() error: %s", strerror(errno));
    SetErrorCode(PresenterErrorCode::kConnection);
    return socketutils::kSocketError;
  }

  // reuse address
  socketutils::SetSocketReuseAddr(sock);

  // set timeout
  socketutils::SetSocketTimeout(sock, kDefaultTimeoutInSec);

  // do connect
  if (socketutils::Connect(sock, addr) == socketutils::kSocketError) {
    if (errno == EINVAL) {
      SetErrorCode(PresenterErrorCode::kInvalidParam);
    } else {
      SetErrorCode(PresenterErrorCode::kConnection);
    }

    AGENT_LOG_ERROR("Failed to connect to server: %s:%u", host_ip.c_str(), port);

    // connect failed, close socket
    (void) close(sock);
    return socketutils::kSocketError;
  }

  // connect successfully
  SetErrorCode(PresenterErrorCode::kNone);
  AGENT_LOG_INFO("Connected to server %s:%d, socket file descriptor = %d",
                 host_ip.c_str(), port, sock);
  return sock;
}

} /* namespace presenter */
} /* namespace ascend */
