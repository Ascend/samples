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

#include "ascenddk/presenter/agent/presenter/presenter_channel_init_handler.h"

#include "ascenddk/presenter/agent/presenter/presenter_message_helper.h"
#include "ascenddk/presenter/agent/util/logging.h"

using google::protobuf::Message;

namespace ascend {
namespace presenter {

PresentChannelInitHandler::PresentChannelInitHandler(
    const OpenChannelParam& param)
    : param_(param) {
}

google::protobuf::Message* PresentChannelInitHandler::CreateInitRequest() {
  proto::OpenChannelRequest *req =
      new (std::nothrow) proto::OpenChannelRequest();
  if (req != nullptr) {
    error_code_ = PresenterMessageHelper::CreateOpenChannelRequest(
        *req, param_.channel_name, param_.content_type);

    if (error_code_ != PresenterErrorCode::kNone) {
      delete req;
      return nullptr;
    }
  }

  return req;
}

bool PresentChannelInitHandler::CheckInitResponse(const Message& response) {
  error_code_ = PresenterMessageHelper::CheckOpenChannelResponse(response);
  if (error_code_ != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("OpenChannel failed, error = %d", error_code_);
  }
  return error_code_ == PresenterErrorCode::kNone;
}

PresenterErrorCode PresentChannelInitHandler::GetErrorCode() const {
  return error_code_;
}

} /* namespace presenter */
} /* namespace ascend */
