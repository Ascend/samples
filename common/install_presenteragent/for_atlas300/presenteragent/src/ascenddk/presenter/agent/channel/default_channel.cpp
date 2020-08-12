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

#include <chrono>
#include <cstring>
#include <functional>
#include <netinet/in.h>
#include <sstream>
#include <cstddef>

#include "proto/presenter_message.pb.h"

#include "ascenddk/presenter/agent/channel/default_channel.h"
#include "ascenddk/presenter/agent/net/raw_socket_factory.h"
#include "ascenddk/presenter/agent/util/logging.h"

using namespace std;
using namespace google::protobuf;

namespace {
const int HEARTBEAT_INTERVAL = 1500;  // 1.5s
}

namespace ascend {
namespace presenter {

DefaultChannel* DefaultChannel::NewChannel(
    const std::string& host_ip, uint16_t port,
    std::shared_ptr<InitChannelHandler> handler) {
  DefaultChannel *channel = nullptr;
  std::shared_ptr<SocketFactory> fac(
      new (std::nothrow) RawSocketFactory(host_ip, port));
  if (fac != nullptr) {
    channel = new (std::nothrow) DefaultChannel(fac);
    if (channel != nullptr && handler != nullptr) {
      channel->SetInitChannelHandler(handler);
    }
  }

  return channel;
}

DefaultChannel::DefaultChannel(std::shared_ptr<SocketFactory> socket_factory)
    : socket_factory_(socket_factory),
      open_(false),
      disposed_(false) {
}

DefaultChannel::~DefaultChannel() {
  disposed_ = true;
  if (heartbeat_thread_ != nullptr) {
    heartbeat_thread_->join();
  }
}

void DefaultChannel::SetInitChannelHandler(
    std::shared_ptr<InitChannelHandler> handler) {
  init_channel_handler_ = handler;
}

shared_ptr<const InitChannelHandler> DefaultChannel::GetInitChannelHandler() {
  return init_channel_handler_;
}

PresenterErrorCode DefaultChannel::HandleInitialization(
    const Message& message) {
  // send init request
  PresenterErrorCode error_code = conn_->SendMessage(message);
  if (error_code != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("Failed to send init request, %d", error_code);
    return error_code;
  }

  // receive init response
  unique_ptr<Message> resp;
  error_code = conn_->ReceiveMessage(resp);
  if (error_code != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("Failed to send init response, %d", error_code);
    return error_code;
  }

  // check response
  if (!init_channel_handler_->CheckInitResponse(*resp)) {
    AGENT_LOG_ERROR("App check response failed");
    return PresenterErrorCode::kAppDefinedError;
  }

  return PresenterErrorCode::kNone;
}

PresenterErrorCode DefaultChannel::Open() {
  //check request generation before connection
  unique_ptr<Message> message;
  if (init_channel_handler_ != nullptr) {
    message.reset(init_channel_handler_->CreateInitRequest());
    if (message == nullptr) {
      AGENT_LOG_ERROR("App create init request failed");
      return PresenterErrorCode::kAppDefinedError;
    }
  }

  Socket* sock = socket_factory_->Create();
  PresenterErrorCode error_code = socket_factory_->GetErrorCode();
  if (error_code != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("Failed to create socket, %d", error_code);
    return error_code;
  }

  Connection* conn = Connection::New(sock);
  if (conn == nullptr) {
    delete sock;
    return PresenterErrorCode::kBadAlloc;
  }
  this->conn_.reset(conn);

  //perform init process
  if (message != nullptr) {
    error_code = HandleInitialization(*message);
    if (error_code != PresenterErrorCode::kNone) {
      conn_.reset(nullptr);
      return error_code;
    }
  }

  open_ = true;
  // prevent from starting multiple thread
  if (heartbeat_thread_ == nullptr) {
    StartHeartbeatThread();
  }

  return PresenterErrorCode::kNone;
}

void DefaultChannel::StartHeartbeatThread() {
  this->heartbeat_thread_.reset(
      new (nothrow) thread(bind(&DefaultChannel::KeepAlive, this)));

  if (heartbeat_thread_ != nullptr) {
    AGENT_LOG_INFO("heartbeat thread started");
  }
}

void DefaultChannel::KeepAlive() {
  chrono::milliseconds heartbeatInterval(HEARTBEAT_INTERVAL);
  while (!disposed_) {
    SendHeartbeat();

    // interruptable wait
    unique_lock<mutex> lock(mtx_);
    cv_shutdown_.wait_for(lock, heartbeatInterval,
                         [this]() {return disposed_.load();});
  }

  AGENT_LOG_DEBUG("heartbeat thread ended");
}

void DefaultChannel::SendHeartbeat() {
  // reopen channel if disconnected
  if (!open_) {
    if (Open() != PresenterErrorCode::kNone) {
      return;
    }
  }

  // construct a heartbeat message then send it
  proto::HeartbeatMessage heartbeat_msg;
  SendMessage(heartbeat_msg);
}

PresenterErrorCode DefaultChannel::SendMessage(const Message& message) {
  PartialMessageWithTlvs msg;
  string msg_name = message.GetDescriptor()->full_name();
  AGENT_LOG_DEBUG("To send message: %s", msg_name.c_str());
  msg.message = &message;
  return SendMessage(msg);
}

PresenterErrorCode DefaultChannel::SendMessage(
    const PartialMessageWithTlvs& message) {
  if (!open_) {
    AGENT_LOG_ERROR("Channel is not open, send message failed");
    return PresenterErrorCode::kConnection;
  }

  PresenterErrorCode errorCode = PresenterErrorCode::kOther;
  try {
    errorCode = conn_->SendMessage(message);
    //connect error, set is_open to false, enable retry
    if (errorCode == PresenterErrorCode::kConnection) {
      open_ = false;
    }
  } catch (std::exception &e) {  // protobuf may throw FatalException
    AGENT_LOG_ERROR("Protobuf error: %s", e.what());
    open_ = false;
  }

  return errorCode;
}

PresenterErrorCode DefaultChannel::ReceiveMessage(
    unique_ptr<Message>& message) {
  AGENT_LOG_DEBUG("To receive message");
  if (!open_) {
    AGENT_LOG_ERROR("Channel is not open, receive message failed");
    return PresenterErrorCode::kConnection;
  }

  PresenterErrorCode error_code = PresenterErrorCode::kOther;
  try {
    error_code = conn_->ReceiveMessage(message);
    // connect error and codec error, set is_open to false, enable retry
    if (error_code == PresenterErrorCode::kConnection
        || error_code == PresenterErrorCode::kCodec) {
      open_ = false;
    }

  } catch (std::exception &e) {  // protobuf may throw FatalException
    AGENT_LOG_ERROR("Protobuf error: %s", e.what());
    open_ = false;
  }

  return error_code;
}

PresenterErrorCode DefaultChannel::SendMessage(
    const google::protobuf::Message& message,
    std::unique_ptr<google::protobuf::Message> &response) {
  string msg_name = message.GetDescriptor()->full_name();
  AGENT_LOG_DEBUG("To send message: %s", msg_name.c_str());
  PresenterErrorCode error_code = SendMessage(message);
  if (error_code == PresenterErrorCode::kNone) {
    error_code = ReceiveMessage(response);
  }

  return error_code;
}

PresenterErrorCode DefaultChannel::SendMessage(
    const PartialMessageWithTlvs& message,
    std::unique_ptr<google::protobuf::Message> &response) {
  string msg_name = message.message->GetDescriptor()->full_name();
  AGENT_LOG_DEBUG("To send message: %s", msg_name.c_str());
  PresenterErrorCode error_code = SendMessage(message);
  if (error_code == PresenterErrorCode::kNone) {
    error_code = ReceiveMessage(response);
  }

  return error_code;
}

const std::string& DefaultChannel::GetDescription() const {
  return this->description_;
}

void DefaultChannel::SetDescription(const std::string& desc) {
  this->description_ = desc;
}

}
/* namespace presenter */
} /* namespace ascend */
