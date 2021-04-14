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

#ifndef ASCENDDK_PRESENTER_AGENT_NET_RAW_SOCKET_H_
#define ASCENDDK_PRESENTER_AGENT_NET_RAW_SOCKET_H_

#include "ascenddk/presenter/agent/errors.h"
#include "ascenddk/presenter/agent/net/socket.h"

namespace ascend {
namespace presenter {

/**
 * RawSocket, the data is not encrypted
 */
class RawSocket : public Socket {
 public:
  /**
   * @brief Factory method
   * @param [in] socket               socket file descriptor
   */
  static RawSocket* New(int socket);

  /**
   * @brief Constructor
   * @param [in] socket               socket file descriptor
   */
  explicit RawSocket(int socket);

  // Disable copy constructor and assignment operator
  RawSocket(const RawSocket& other) = delete;
  RawSocket& operator=(const RawSocket& other) = delete;

  /**
   * @brief Destructor
   */
  virtual ~RawSocket();

 protected:

  /**
   * @brief Read bytes from socket
   * @param [in] buffer               receive buffer
   * @param [in] size                 expected bytes
   * @return bytes received. -1 of read failed
   */
  virtual int DoRecv(char *buffer, int size) override;

  /**
   * @brief Write bytes to socket
   * @param [in] data                 bytes to send
   * @param [in] size                 size of data
   * @return bytes sent. -1 if send failed
   */
  virtual int DoSend(const char *data, int size) override;

 private:
  int socket_;
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_NET_RAW_SOCKET_H_ */
