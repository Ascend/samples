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

#ifndef ASCENDDK_PRESENTER_AGENT_UTIL_BYTE_BUFFER_H_
#define ASCENDDK_PRESENTER_AGENT_UTIL_BYTE_BUFFER_H_

#include <cstdint>
#include <string>
#include <memory>

#include <google/protobuf/message.h>

namespace ascend {
namespace presenter {

class SharedByteBuffer {
 public:
  static SharedByteBuffer Make(std::uint32_t size);

  /**
   * @brief constructor
   * @param [in] buf      buffer
   * @param [in] size     size of buffer
   */
  SharedByteBuffer(char* buf, std::uint32_t size);

  /**
   * @brief constructor
   */
  SharedByteBuffer();
  ~SharedByteBuffer() = default;

  /**
   * @brief Get buffer
   * @return buffer
   */
  const char* Get() const;

  /**
   * @brief Get buffer
   * @return buffer
   */
  char* GetMutable() const;

  /**
   * @brief Get size of buffer
   * @return size of buffer
   */
  std::uint32_t Size() const;

  /**
   * @brief Check whether the buffer is empty
   * @return true: empty, false: non-empty
   */
  bool IsEmpty() const;

 private:
  std::shared_ptr<char> buf_;
  std::uint32_t size_;
};

/**
 * a helper data structure that holds a byte array and byte array size
 * Note: ByteBuffer does NOT own the underlying data
 */
class ByteBuffer {
 public:
  /**
   * @brief constructor
   * @param [in] buf      buffer
   * @param [in] size     size of buffer
   */
  ByteBuffer(const char* buf, std::uint32_t size);

  /**
   * @brief constructor
   */
  ByteBuffer();
  ~ByteBuffer() = default;

  /**
   * @brief Get buffer
   * @return buffer
   */
  const char* Get() const;

  /**
   * @brief Get size of buffer
   * @return size of buffer
   */
  std::uint32_t Size() const;

  /**
   * @brief Check whether the buffer is empty
   * @return true: empty, false: non-empty
   */
  bool IsEmpty() const;

 private:
  const char *buf;
  std::uint32_t size;
};

/**
 * help to write kinds of data to a buffer
 * after all write operation is finished, call GetBuffer to get the buffer
 */
class ByteBufferWriter {
 public:
  /**
   * @brief constructor
   * @param [out] buf   output buffer
   * @param [in] size   size of buffer
   */
  ByteBufferWriter(char* buf, int size);
  ~ByteBufferWriter();

  // Disable copy constructor and assignment operator
  ByteBufferWriter(const ByteBufferWriter&) = delete;
  ByteBufferWriter& operator=(const ByteBufferWriter&) = delete;

  // Write methods
  // caller must make sure that there is sufficient memory is buffer

  /**
   * @brief write an uint8 integer to buffer
   * @param [in] value      value
   */
  void PutUInt8(std::uint8_t value);

  /**
   * @brief write an uint32 integer to buffer
   * @param [in] value      value
   */
  void PutUInt32(std::uint32_t value);

  /**
   * @brief write an String value to buffer, '\0' is excluded
   * @param [in] value      string
   */
  void PutString(const std::string& value);

  /**
   * @brief write an protobuf message to buffer
   * @param [in] msg        protobuf message
   * @return true: success; false: serialization failure
   */
  bool PutMessage(const ::google::protobuf::Message& msg);

  /**
   * @brief Finish writing and wrap the data to ByteBuffer
   * @return ByteBuffer
   */
  ByteBuffer GetBuffer();

 private:
  /**
   * @brief put bytes to buffer
   * @param [in] data        buffer of data
   * @param [in] size        size of buffer
   */
  void PutBytes(const void* data, size_t size);

  char *buf_;
  char *w_ptr_;
  const char* const end_;
};

/**
 * help to read kinds of data from a buffer
 * ByteBufferReader will NOT take the ownership of the buffer,
 * so it is the caller's responsibility to free the memory of the buffer
 */
class ByteBufferReader {
 public:
  /**
   * @brief constructor
   * @param [in] buf          buffer
   * @param [in] size         size of buffer
   */
  ByteBufferReader(const char* buf, int size);
  ~ByteBufferReader() = default;

  // Disable copy constructor and assignment operator
  ByteBufferReader(const ByteBufferReader&) = delete;
  ByteBufferReader& operator=(ByteBufferReader) = delete;

  /**
   * @brief read an uint8 integer from buffer
   * @return uint8 integer
   */
  std::uint8_t ReadUInt8();

  /**
   * @brief read an uint32 integer from buffer
   * @return uint32 integer
   */
  std::uint32_t ReadUInt32();

  /**
   * @brief read an string from buffer
   * @return string
   */
  std::string ReadString(int size);

  /**
   * @brief read an protobuf message from buffer
   * @param [in]  size          size of the message
   * @param [out] message
   * @return true: success, false: parse message failed
   */
  bool ReadMessage(int size, ::google::protobuf::Message &message);

  /**
   * @brief get the size of buffer available for reading
   * @return remaining bytes for reading
   */
  int RemainingBytes();

 private:
  const char* r_ptr_;
  const char* const end_;
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_UTIL_BYTE_BUFFER_H_ */
