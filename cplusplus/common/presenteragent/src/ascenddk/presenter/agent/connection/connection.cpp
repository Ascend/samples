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

#include "ascenddk/presenter/agent/connection/connection.h"

#include <netinet/in.h>
#include <sstream>

#include "ascenddk/presenter/agent/codec/message_codec.h"
#include "ascenddk/presenter/agent/net/raw_socket_factory.h"
#include "ascenddk/presenter/agent/util/byte_buffer.h"
#include "ascenddk/presenter/agent/util/logging.h"
#include "ascenddk/presenter/agent/util/mem_utils.h"


namespace {
  const uint32_t kMaxPacketSize = 1024 * 1024 * 10; //10MB
}

namespace ascend {
namespace presenter {

using google::protobuf::Message;
using namespace std;

Connection::Connection(Socket* socket)
    : socket_(socket) {
}

Connection* Connection::New(Socket* socket) {
  if (socket == nullptr) {
    AGENT_LOG_ERROR("socket is null");
    return nullptr;
  }

  return new (nothrow) Connection(socket);
}

PresenterErrorCode Connection::SendTlvList(const std::vector<Tlv>& tlv_list) {
  if (tlv_list.empty()) {
    return PresenterErrorCode::kNone;
  }

  for (auto it = tlv_list.begin(); it != tlv_list.end(); ++it) {
    SharedByteBuffer tlv_buf = codec_.EncodeTagAndLength(*it);
    if (tlv_buf.IsEmpty()) {
      AGENT_LOG_ERROR("Failed to encode TLV");
      return PresenterErrorCode::kCodec;
    }

    //send tag and length
    PresenterErrorCode error_code = socket_->Send(tlv_buf.Get(),
                                                  tlv_buf.Size());
    if (error_code != PresenterErrorCode::kNone) {
      AGENT_LOG_ERROR("Failed to send TLV tag and length");
      return error_code;
    }

    //send value
    error_code = socket_->Send(it->value, it->length);
    if (error_code != PresenterErrorCode::kNone) {
      AGENT_LOG_ERROR("Failed to send TLV value");
      return error_code;
    }
  }

  return PresenterErrorCode::kNone;
}

PresenterErrorCode Connection::SendMessage(
    const PartialMessageWithTlvs& proto_message) {
  if (proto_message.message == nullptr) {
    AGENT_LOG_ERROR("message is null");
    return PresenterErrorCode::kInvalidParam;
  }
  // lock for encoding and sending
  unique_lock<mutex> lock(mtx_);

  const char* msg_name = proto_message.message->GetDescriptor()->name().c_str();
  SharedByteBuffer buffer = codec_.EncodeMessage(proto_message);
  if (buffer.IsEmpty()) {
    AGENT_LOG_ERROR("Failed to encode message: %s", msg_name);
    return PresenterErrorCode::kCodec;
  }

  // send message
  PresenterErrorCode error_code = socket_->Send(buffer.Get(), buffer.Size());
  // if send success and has more to send..
  if (error_code != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("Failed to send message: %s", msg_name);
    return error_code;
  }

  return SendTlvList(proto_message.tlv_list);
}

PresenterErrorCode Connection::SendMessage(const Message& message) {
  PartialMessageWithTlvs msg;
  msg.message = &message;
  return SendMessage(msg);
}

PresenterErrorCode Connection::ReceiveMessage(
    unique_ptr<::google::protobuf::Message>& message) {
  // read 4 bytes header
  char *buf = recv_buf_;
  PresenterErrorCode error_code = socket_->Recv(
      buf, MessageCodec::kPacketLengthSize);

  if (error_code == PresenterErrorCode::kSocketTimeout) {
    AGENT_LOG_INFO("Read message header timeout");
    return PresenterErrorCode::kSocketTimeout;
  }

  if (error_code != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("Failed to read message header");
    return error_code;
  }

  // parse length
  uint32_t total_size = ntohl(*((uint32_t*) buf));

  // read the remaining data
  uint32_t remaining_size = total_size - MessageCodec::kPacketLengthSize;
  if (remaining_size == 0 || remaining_size > kMaxPacketSize) {
    AGENT_LOG_ERROR("received malformed message, size field = %u", total_size);
    return PresenterErrorCode::kCodec;
  }

  int pack_size = static_cast<int>(remaining_size);
  unique_ptr<char[]> unique_buf; // ensure release allocated buffer
  if (remaining_size > kBufferSize) {
    buf = memutils::NewArray<char>(remaining_size);
    if (buf == nullptr) {
      return PresenterErrorCode::kBadAlloc;
    }

    unique_buf.reset(memutils::NewArray<char>(remaining_size));
  }

  // packSize must be within [1, MAX_PACKET_SIZE],
  // Recv() can not cause buffer overflow
  error_code = socket_->Recv(buf, pack_size);
  if (error_code != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("Failed to read whole message");
    return PresenterErrorCode::kConnection;
  }

  // Decode message
  Message* msg = codec_.DecodeMessage(buf, pack_size);
  if (msg == nullptr) {
    return PresenterErrorCode::kCodec;
  }

  message.reset(msg);
  string name = message->GetDescriptor()->name();
  AGENT_LOG_DEBUG("Message received, name = %s", name.c_str());
  return PresenterErrorCode::kNone;
}

} /* namespace presenter */
} /* namespace ascend */
