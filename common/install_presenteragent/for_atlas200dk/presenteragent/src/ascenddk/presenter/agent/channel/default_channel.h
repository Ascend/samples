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

#ifndef ASCENDDK_PRESENTER_AGENT_CHANNEL_DEFAULT_CHANNEL_H_
#define ASCENDDK_PRESENTER_AGENT_CHANNEL_DEFAULT_CHANNEL_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "ascenddk/presenter/agent/connection/connection.h"
#include "ascenddk/presenter/agent/channel.h"

namespace ascend {
namespace presenter {

/**
 * Default channel implementation
 */
class DefaultChannel : public Channel {
 public:

  /**
   * @brief create a channel
   * @param [in] host_ip                host IP of server
   * @param [in] port                   port of server
   * @param [in] handler                init handler
   * @return pointer to channel
   */
  static DefaultChannel* NewChannel(
      const std::string& host_ip, uint16_t port,
      std::shared_ptr<InitChannelHandler> handler);

  virtual ~DefaultChannel();

  /**
   * @brief Open channel
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode Open() override;

  /**
   * @brief send message to server
   * @param [in] message              message
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(
      const google::protobuf::Message& message) override;

  /**
   * @brief send message to server
   * @param [in] message              message
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(const PartialMessageWithTlvs& message)
      override;

  /**
   * @brief send message to server and read the response
   * @param [in] message              message
   * @pararm [out] response           response
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(
      const google::protobuf::Message& message,
      std::unique_ptr<google::protobuf::Message>& response) override;

  /**
   * @brief send message to server and read the response
   * @param [in] message              message
   * @pararm [out] response           response
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode SendMessage(
      const PartialMessageWithTlvs& message,
      std::unique_ptr<google::protobuf::Message>& response) override;

  /**
   * @brief recevice a response
   * @param [out] response            response
   * @return PresenterErrorCode
   */
  virtual PresenterErrorCode ReceiveMessage(
      std::unique_ptr<google::protobuf::Message>& response) override;

  /**
   * @brief set InitChannelHandler
   * @param [in] handler              handler
   */
  void SetInitChannelHandler(std::shared_ptr<InitChannelHandler> handler);

  /**
   * @brief get InitChannelHandler
   * @return InitChannelHandler
   */
  std::shared_ptr<const InitChannelHandler> GetInitChannelHandler();

  /**
   * @brief set description
   * @param [in] desc              description
   */
  void SetDescription(const std::string& desc);

  /**
   * @brief Get the description of the channel, can be used for logging
   * @return description
   */
  const std::string& GetDescription() const override;

 private:
  /**
   * @brief constructor
   * @param [in] socket_factory     socket factory
   */
  DefaultChannel(std::shared_ptr<SocketFactory> socket_factory);

  /**
   * @brief handle channel initialization process
   */
  PresenterErrorCode HandleInitialization(
      const google::protobuf::Message& message);

  /**
   * @brief Start heartbeat thread
   */
  void StartHeartbeatThread();

  /**
   * @brief Task to keep the channel alive
   */
  void KeepAlive();

  /**
   * @brief Send heartbeat message to server
   */
  void SendHeartbeat();

 private:
  std::shared_ptr<SocketFactory> socket_factory_;
  std::shared_ptr<InitChannelHandler> init_channel_handler_;
  std::unique_ptr<Connection> conn_;

  // indicating whether the socket is valid
  std::atomic_bool open_;
  // indicating whether channel is valid
  std::atomic_bool disposed_;

  std::mutex mtx_;
  std::condition_variable cv_shutdown_;
  std::unique_ptr<std::thread> heartbeat_thread_;

  std::string description_;
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_CHANNEL_DEFAULT_CHANNEL_H_ */
