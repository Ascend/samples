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

#ifndef ASCENDDK_PRESENTER_AGENT_CHANNEL_H_
#define ASCENDDK_PRESENTER_AGENT_CHANNEL_H_

#include <string>
#include <cstdint>
#include <vector>
#include <memory>

#include "ascenddk/presenter/agent/errors.h"

namespace google {
namespace protobuf {
class Message;
}
}

namespace ascend {
namespace presenter {

/**
 * TLV
 */
struct Tlv {
  int tag;
  int length;
  const char* value;
};

/**
 * When a message has large fields, this can be used to avoid
 * unnecessary memory copies
 */
struct PartialMessageWithTlvs {
  const google::protobuf::Message* message;
  std::vector<Tlv> tlv_list;
};

/**
 * Deal with channel initialization
 */
class InitChannelHandler {
 public:
  InitChannelHandler() = default;
  virtual ~InitChannelHandler() = default;

  /**
   * @brief Create initialize request
   * @return initialize request
   */
  virtual google::protobuf::Message* CreateInitRequest() = 0;

  /**
   * @brief check the response
   * @param [in] response             response
   * @return check result
   */
  virtual bool CheckInitResponse(const google::protobuf::Message& response) = 0;
};

/**
 * @brief General channel
 */
class Channel {
 public:
  virtual ~Channel() = default;

  /**
   * @brief Open channel
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode Open() = 0;

  /**
   * @brief send message to server
   * @param [in] message              message
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(
      const google::protobuf::Message& message) = 0;

  /**
   * @brief send message to server
   * @param [in] message              message
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(
      const PartialMessageWithTlvs& message) = 0;

  /**
   * @brief send message to server and read the response
   * @param [in] message              message
   * @pararm [out] response           response
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(
      const google::protobuf::Message& message,
      std::unique_ptr<google::protobuf::Message>& response) = 0;

  /**
   * @brief send message to server and read the response
   * @param [in] message              message
   * @pararm [out] response           response
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(
      const PartialMessageWithTlvs& message,
      std::unique_ptr<google::protobuf::Message>& response) = 0;

  /**
   * @brief recevice a response
   * @param [out] response            response
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode ReceiveMessage(
      std::unique_ptr<google::protobuf::Message>& response) = 0;

  /**
   * @brief Get the description of the channel, can be used for logging
   * @return description
   */
  virtual const std::string& GetDescription() const = 0;
};

/**
 * Channel Factory
 */
class ChannelFactory {
 public:
  /**
   * @brief create a channel
   * @param [in] host_ip                host IP of server
   * @param [in] port                   port of server
   * @return pointer to channel
   */
  static Channel* NewChannel(const std::string& host_ip, uint16_t port);

  /**
   * @brief create a channel
   * @param [in] host_ip                host IP of server
   * @param [in] port                   port of server
   * @param [in] handler                init handler
   * @return pointer to channel
   */
  static Channel* NewChannel(const std::string& host_ip, uint16_t port,
                             std::shared_ptr<InitChannelHandler> handler);
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_CHANNEL_H_ */
