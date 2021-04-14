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

#include "ascenddk/presenter/agent/codec/message_codec.h"

#include <string>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

#include "ascenddk/presenter/agent/util/logging.h"

using namespace google::protobuf;
using namespace google::protobuf::io;
using namespace std;

namespace {
const int kMessageNameLengthSize = sizeof(uint8_t);

// protobuf tag size
const int kTagSize = 1;

// protobuf string/bytes wire type
const int kProtoStringWireType = 0x2;

// max buffer size for varint32
static const int kMaxVarint32Bytes = 5;

// for calc tag
const int kTagShift = 3;

// empty string
const string kEmptyStr = "";
}

namespace ascend {
namespace presenter {

// make protobuf tag, tag should less than 15
static uint8_t MakeTag(int tag) {
  return static_cast<uint8_t>(tag) << kTagShift | kProtoStringWireType;
}

static string ConvertToVarint32(uint32_t value) {
  // Zero-length field should not be serialized
  if (value == 0) {
    return kEmptyStr;
  }

  char buf[kMaxVarint32Bytes];
  ArrayOutputStream arr(buf, kMaxVarint32Bytes);
  CodedOutputStream os(&arr);
  os.WriteVarint32(value);
  // os.ByteCount() <= kMaxVarint32Bytes
  return string(buf, os.ByteCount());
}

SharedByteBuffer MessageCodec::EncodeTagAndLength(const Tlv& tlv) {
  string varlen = ConvertToVarint32(tlv.length);
  if (varlen.empty()) {
    AGENT_LOG_ERROR("length is 0");
    return SharedByteBuffer();
  }

  uint32_t size = kTagSize + varlen.size();
  SharedByteBuffer result = SharedByteBuffer::Make(size);
  if (result.IsEmpty()) {
    return result;
  }

  ByteBufferWriter buffer(result.GetMutable(), size);
  // put tag
  buffer.PutUInt8(MakeTag(tlv.tag));
  // put var length
  buffer.PutString(string(varlen.c_str(), varlen.size()));
  return result;
}

SharedByteBuffer MessageCodec::EncodeMessage(
    const google::protobuf::Message& message) {
  PartialMessageWithTlvs msg;
  msg.message = &message;
  return EncodeMessage(msg);
}

SharedByteBuffer MessageCodec::EncodeMessage(
    const PartialMessageWithTlvs& msg) {
  if (msg.message == nullptr) {
    return SharedByteBuffer();
  }

  const Message& message = *(msg.message);
  vector<Tlv> tlv_list = msg.tlv_list;

  string name = message.GetDescriptor()->full_name();
  uint32_t msg_size = static_cast<uint32_t>(message.ByteSize());
  uint8_t msg_name_size = static_cast<uint8_t>(name.size());

  // calc total size
  uint32_t encode_size = kPacketLengthSize + kMessageNameLengthSize;
  encode_size += msg_name_size + msg_size;

  uint32_t total_size = encode_size;
  // if has additional field
  if (!tlv_list.empty()) {
    for (auto it = tlv_list.begin(); it != tlv_list.end(); ++it) {
      string varlen = ConvertToVarint32(it->length);
      total_size = total_size + kTagSize + varlen.size() + it->length;
    }
  }

  SharedByteBuffer encode_buffer = SharedByteBuffer::Make(encode_size);
  if (encode_buffer.IsEmpty()) {
    return encode_buffer;
  }

  // serialize message
  ByteBufferWriter buffer(encode_buffer.GetMutable(), encode_size);
  buffer.PutUInt32(total_size);
  buffer.PutUInt8(msg_name_size);
  buffer.PutString(name);
  // serialization may fail if any of the required field is not set,
  // in which case, a empty buffer is returned
  if (!buffer.PutMessage(message)) {
    return SharedByteBuffer();
  }

  return encode_buffer;
}

// Generate message prototype by name for parsing
static Message* NewMessageByName(const string& name) {
  const Descriptor* descriptor = DescriptorPool::generated_pool()
      ->FindMessageTypeByName(name);
  if (descriptor != nullptr) {
    const Message* prototype =
        MessageFactory::generated_factory()->GetPrototype(descriptor);
    if (descriptor != nullptr) {
      return prototype->New();
    }
  }

  return nullptr;
}

Message* MessageCodec::DecodeMessage(const char* data, int size) {
  if (size < kMessageNameLengthSize) {
    AGENT_LOG_ERROR("Insufficient data for message name length field");
    return nullptr;
  }

  // wrap message data with Reader
  ByteBufferReader buffer(data, size);

  // read message name length
  uint8_t msg_name_length = buffer.ReadUInt8();
  if (buffer.RemainingBytes() < msg_name_length) {
    AGENT_LOG_ERROR(
        "Insufficient data for name field, expect %d, but remain %d",
        msg_name_length, buffer.RemainingBytes());
    return nullptr;
  }

  // read message name
  string name = buffer.ReadString(msg_name_length);
  // get message prototype by name
  Message* message = NewMessageByName(name);
  if (message == nullptr) {
    AGENT_LOG_ERROR("Unsupported message, name = %s", name.c_str());
    return nullptr;
  }

  // parse message
  if (!buffer.ReadMessage(buffer.RemainingBytes(), *message)) {
    AGENT_LOG_ERROR("Failed to parse message, name = %s", name.c_str());
    delete message;
    return nullptr;
  }

  return message;
}

} /* namespace presenter */
} /* namespace ascend */

