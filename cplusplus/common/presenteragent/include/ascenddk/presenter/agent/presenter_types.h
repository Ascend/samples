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
#ifndef ASCENDDK_PRESENTER_AGENT_DATA_TYPES_H_
#define ASCENDDK_PRESENTER_AGENT_DATA_TYPES_H_

#include <string>
#include <cstdint>
#include <vector>

namespace ascend {
namespace presenter {

/**
 * ContentType
 */
enum class ContentType {
  // Image
  kImage = 0,

  // Video
  kVideo = 1,

  // Reserved content type, do not use this
  kReserved = 127,
};

/**
 * ImageFormat
 */
enum class ImageFormat {
  // JPEG
  kJpeg = 0,

  // Reserved format, do not use this
  kReserved = 127,
};

/**
 * OpenChannelParam
 */
struct OpenChannelParam {
  std::string host_ip;
  std::uint16_t port;
  std::string channel_name;
  ContentType content_type;
};

struct Point {
    std::uint32_t x;
    std::uint32_t y;
};

struct DetectionResult {
    Point lt;   //The coordinate of left top point
    Point rb;   //The coordinate of the right bottom point
    std::string result_text;  // Face:xx%
};
/**
 * ImageFrame
 */
struct ImageFrame {
  ImageFormat format;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t size;
  unsigned char *data;
  std::vector<DetectionResult> detection_results;
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_DATA_TYPES_H_ */
