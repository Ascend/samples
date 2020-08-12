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

#ifndef ASCENDDK_PRESENTER_AGENT_CONNECTION_CONNECTION_H_
#define ASCENDDK_PRESENTER_AGENT_CONNECTION_CONNECTION_H_

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <google/protobuf/message.h>

#include "ascenddk/presenter/agent/codec/message_codec.h"
#include "ascenddk/presenter/agent/errors.h"
#include "ascenddk/presenter/agent/net/socket_factory.h"

namespace ascend {
namespace presenter {

/**
 * Connection between agent and server
 * provide protobuf based interface
 */
class Connection {
 public:
  static Connection* New(Socket* socket);
  ~Connection() = default;

  /**
   * @brief Send a protobuf Message to presenter server
   * @param [in] message        protobuf message
   * @return PresenterErrorCode
   */
  PresenterErrorCode SendMessage(const ::google::protobuf::Message& message);

  /**
   * @brief Send a Message to presenter server
   * @param [in] message        PartialMessageWithTlvs
   * @return PresenterErrorCode
   */
  PresenterErrorCode SendMessage(const PartialMessageWithTlvs& message);

  /**
   * @brief Receive a message from presenter server
   * @param [out] message       response message
   * @return PresenterErrorCode
   */
  PresenterErrorCode ReceiveMessage(
      std::unique_ptr<::google::protobuf::Message>& message);

 private:
  PresenterErrorCode DoSendMessage(const ::google::protobuf::Message& message,
                                   const std::vector<Tlv>& tlv_list);

 private:
  Connection(Socket* socket);

  /**
   * @brief Send tlv in protobuf format to server
   * @param [out] message       response message
   * @return PresenterErrorCode
   */
  PresenterErrorCode SendTlvList(const std::vector<Tlv>& tlv_list);

  // max size of received message
  static const int kBufferSize = 1024;

  std::unique_ptr<Socket> socket_;

  char recv_buf_[kBufferSize] = { 0 };

  std::mutex mtx_;

  MessageCodec codec_;
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_CONNECTION_CONNECTION_H_ */
