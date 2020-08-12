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

#include "ascenddk/presenter/agent/net/socket.h"

#include "ascenddk/presenter/agent/util/logging.h"
#include "ascenddk/presenter/agent/util/socket_utils.h"

namespace ascend {
namespace presenter {

PresenterErrorCode Socket::Send(const char *data, int size) {
  int ret = DoSend(data, size);
  if (ret == socketutils::kSocketError) {
    return PresenterErrorCode::kConnection;
  }

  // check size of sent data
  if (ret < size) {
    AGENT_LOG_ERROR("Socket::Send() error, expect %d bytes, but sent %d", size,
                    ret);
    return PresenterErrorCode::kConnection;
  }

  AGENT_LOG_DEBUG("Socket::Send() succeeded, size = %d", size);
  return PresenterErrorCode::kNone;
}

PresenterErrorCode Socket::Recv(char *buffer, int size) {
  int ret = DoRecv(buffer, size);
  if (ret == socketutils::kSocketError) {
    return PresenterErrorCode::kConnection;
  } else if (ret == socketutils::kSocketTimeout) {
    return PresenterErrorCode::kSocketTimeout;
  }

  // check size of received data
  if (ret < size) {
    AGENT_LOG_ERROR("Socket::Recv() error, expect %d bytes, but received %d",
                    size, ret);
    return PresenterErrorCode::kConnection;
  }

  AGENT_LOG_DEBUG("Socket::Recv() succeeded, size = %d", size);
  return PresenterErrorCode::kNone;
}

} /* namespace presenter */
} /* namespace ascend */

