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

#ifndef ASCENDDK_PRESENTER_AGENT_PRESENTER_PRESENTER_CHANNEL_INIT_HANDLER_H_
#define ASCENDDK_PRESENTER_AGENT_PRESENTER_PRESENTER_CHANNEL_INIT_HANDLER_H_

#include <google/protobuf/message.h>

#include "ascenddk/presenter/agent/channel.h"
#include "ascenddk/presenter/agent/errors.h"
#include "ascenddk/presenter/agent/presenter_types.h"

namespace ascend {
namespace presenter {

/**
 * Presenter Channel Init Handler
 */
class PresentChannelInitHandler : public InitChannelHandler {
 public:
  /**
   * @brief Constructor
   * @param [in] param          Open channel parameter
   */
  PresentChannelInitHandler(const OpenChannelParam& param);

  /**
   * @brief Create OpenChannelRequest
   * @return OpenChannelRequest
   */
  google::protobuf::Message* CreateInitRequest() override;

  /**
   * @brief Check OpenChannelResponse
   * @param [in] response       response
   * @return check result
   */
  bool CheckInitResponse(const google::protobuf::Message& response) override;

  /**
   * @brief Get ErrorCode
   * @return PresenterErrorCode
   */
  PresenterErrorCode GetErrorCode() const;

 private:
  OpenChannelParam param_;
  PresenterErrorCode error_code_ = PresenterErrorCode::kOther;
};

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_PRESENTER_PRESENTER_CHANNEL_INIT_HANDLER_H_ */
