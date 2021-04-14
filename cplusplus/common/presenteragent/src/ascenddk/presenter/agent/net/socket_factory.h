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

#ifndef ASCENDDK_PRESENTER_AGENT_NET_SOCKET_FACTORY_H_
#define ASCENDDK_PRESENTER_AGENT_NET_SOCKET_FACTORY_H_

#include "ascenddk/presenter/agent/net/socket.h"

#include <string>
#include <memory>


namespace ascend {
namespace presenter {

/**
 * Abstract SocketFactory for creating Socket
 * Subclasses implement Create() to return concrete instance
 */
class SocketFactory {
 public:

  /**
   * Destructor
   */
  virtual ~SocketFactory() = default;

  /**
   * @brief Create instance of Socket, If NULL is returned,
   *        invoke GetErrorCode() for error code
   * @return pointer of Socket
   */
  virtual Socket* Create() = 0;

  /**
   * @brief Get error code
   */
  PresenterErrorCode GetErrorCode() const;

 protected:

  /**
   * @brief create a socket and connect to server
   * @param [in] host_ip              host IP
   * @param [in] port                 port
   * @return socket file descriptor, if SOCKET_ERROR(-1) is returned,
   *         invoke GetErrorCode() for error code
   */
  int CreateSocket(const std::string& host_ip, std::uint16_t port);

  /**
   * @brief Set error code
   * @param[in] error_code             error code
   */
  void SetErrorCode(PresenterErrorCode error_code);

 private:
  PresenterErrorCode error_code_ = PresenterErrorCode::kNone;
};

}
}

#endif /* ASCENDDK_PRESENTER_AGENT_NET_SOCKET_FACTORY_H_ */
