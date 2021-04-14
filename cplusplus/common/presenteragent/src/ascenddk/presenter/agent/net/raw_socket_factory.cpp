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

#include "ascenddk/presenter/agent/net/raw_socket_factory.h"

#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "ascenddk/presenter/agent/util/logging.h"
#include "ascenddk/presenter/agent/util/socket_utils.h"

using std::string;

namespace ascend {
namespace presenter {

RawSocketFactory::RawSocketFactory(const string& host_ip, uint16_t port)
    : host_ip_(host_ip),
      port_(port) {
}

// overrided method of Create()
RawSocket* RawSocketFactory::Create() {
  // create a socket and connect to server
  int sock = CreateSocket(host_ip_, port_);
  if (sock == socketutils::kSocketError) {
    return nullptr;
  }

  // No error, create RawSocket and return
  RawSocket *ret = RawSocket::New(sock);
  if (ret == nullptr) {
    (void) close(sock);
    SetErrorCode(PresenterErrorCode::kBadAlloc);
  }

  return ret;
}

} /* namespace presenter */
} /* namespace ascend */
