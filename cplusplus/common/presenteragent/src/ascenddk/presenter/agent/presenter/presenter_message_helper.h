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

#ifndef SRC_CHANNEL_PRESENTERMESSAGEHELPER_H_
#define SRC_CHANNEL_PRESENTERMESSAGEHELPER_H_

#include <memory>

#include "ascenddk/presenter/agent/errors.h"
#include "ascenddk/presenter/agent/presenter_types.h"
#include "proto/presenter_message.pb.h"

namespace ascend {
namespace presenter {

/**
 * Helper class for Presenter Messages
 */
class PresenterMessageHelper {
 public:
  // helper class, constructor/destructor is not needed
  PresenterMessageHelper() = delete;
  ~PresenterMessageHelper() = delete;

  /**
   * @brief create OpenChannelRequest
   * @param [out] request         request to set the properties
   * @param [in] channelName      channel name
   * @param [in] contentType      content type
   * @return Shared pointer of OpenChannelRequest. nullptr is returned if
   *         any of the parameters is invalid
   */
  static PresenterErrorCode CreateOpenChannelRequest(
      proto::OpenChannelRequest& request, const std::string& channel_name,
      ContentType content_type);

  /**
   * @brief create PresentImageRequest
   * @param [out] request         request to set the properties
   * @param [in] image            image
   * @return true: success, false: failure
   */
  static bool InitPresentImageRequest(proto::PresentImageRequest& request,
                                      const ImageFrame& image);

  /**
   * @brief Check OpenChannelResponse
   * @param [in] msg              Open Channel Response
   * @return PresenterErrorCode
   */
  static PresenterErrorCode CheckOpenChannelResponse(
      const ::google::protobuf::Message& msg);

  /**
   * @brief Check PresentImageResponse
   * @param [in] msg              Present Image Response
   * @return PresenterErrorCode
   */
  static PresenterErrorCode CheckPresentImageResponse(
      const ::google::protobuf::Message& msg);

 private:
  /**
   * @brief Translate OpenChannelErrorCode
   * @param [in] errorCode        Error code
   * @return PresenterErrorCode
   */
  static PresenterErrorCode TranslateErrorCode(
      proto::OpenChannelErrorCode error_code);

  /**
   * @brief Translate PresentDataErrorCode
   * @param [in] errorCode        Error code
   * @return PresenterErrorCode
   */
  static PresenterErrorCode TranslateErrorCode(
      proto::PresentDataErrorCode error_code);
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* SRC_CHANNEL_PRESENTERMESSAGEHELPER_H_ */
