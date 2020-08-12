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

#ifndef ASCENDDK_PRESENTER_AGENT_CODEC_MESSAGE_CODEC_H_
#define ASCENDDK_PRESENTER_AGENT_CODEC_MESSAGE_CODEC_H_

#include <cstdint>
#include <google/protobuf/message.h>

#include "ascenddk/presenter/agent/channel.h"
#include "ascenddk/presenter/agent/util/byte_buffer.h"

namespace ascend {
namespace presenter {

/**
 * MessageCodec for encoding and decoding message
 *
 * A message has the following structure
 *    --------------------------------------------------------------------
 *    |Field Name          |  Size(bytes)   |    Type                     |
 *    --------------------------------------------------------------------
 *    |total message len   |       4        |    uint32                   |
 *    |-------------------------------------------------------------------
 *    |message name len    |       1        |    uint8                    |
 *    |-------------------------------------------------------------------
 *    |message name        |  Var. max 255  |  String, NO terminated '\0' |
 *    |-------------------------------------------------------------------
 *    |message body        |      Var.      |  Bytes. Encoded by protobuf |
 *    --------------------------------------------------------------------
 */
class MessageCodec {
 public:
  // size of channel message total length
  static const int kPacketLengthSize = sizeof(uint32_t);

  /**
   * @brief Encode the message to a ByteBuffer
   * @param [in] message              message
   * @return ByteBuffer. Empty if encode failed
   */
  SharedByteBuffer EncodeMessage(const google::protobuf::Message& message);

  /**
   * @brief Encode the message to a ByteBuffer
   * @param [in] message              message
   * @return ByteBuffer. Empty if encode failed
   */
  SharedByteBuffer EncodeMessage(const PartialMessageWithTlvs& message);

  /**
   * @brief Encode the tag and length to a ByteBuffer
   * @param [in] Tlv                  Tlv
   * @return ByteBuffer. Empty if encode failed
   */
  SharedByteBuffer EncodeTagAndLength(const Tlv& tlv);

  /**
   * @brief Decode the message from buffer
   * @param [in] data                 data buffer
   * @param [in] size                 data size
   * @return Message. NULL if decode failed
   */
  google::protobuf::Message* DecodeMessage(const char* data, int size);

};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_CODEC_MESSAGE_CODEC_H_ */
