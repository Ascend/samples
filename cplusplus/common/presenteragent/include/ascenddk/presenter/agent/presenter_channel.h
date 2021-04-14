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

#ifndef ASCENDDK_PRESENTER_AGENT_PRESENTER_CHANNEL_H_
#define ASCENDDK_PRESENTER_AGENT_PRESENTER_CHANNEL_H_

#include "ascenddk/presenter/agent/channel.h"
#include "ascenddk/presenter/agent/errors.h"
#include "ascenddk/presenter/agent/presenter_types.h"

namespace ascend {
namespace presenter {

/**
 * @brief Open a channel to presenter server
 * @param [out] channel       channel must be a NULL pointer,
 *                            and it will point to an opened channel if
 *                            open successfully
 * @param [in]  param         parameters for opening a channel
 * @return PresenterErrorCode
 */
PresenterErrorCode OpenChannel(Channel *&channel,
                               const OpenChannelParam &param);
/**
 * @brief Open a channel to presenter server by config file
 * @param [out] channel       channel must be a NULL pointer,
 *                            and it will point to an opened channel if
 *                            open successfully
 * @configFile [in]  param    config file of channel configuration
 * @return PresenterErrorCode
 */
PresenterErrorCode OpenChannelByConfig(Channel*& channel,
	                                   const char* configFile);

/**
 * @brief Send the image to server for display through the given channel
 * @param [in] channel        the channel to send the image with
 * @param [in] image          the image to display
 * @return PresenterErrorCode
 */
PresenterErrorCode PresentImage(Channel *channel, const ImageFrame &image);

/**
 * @brief Send the image message to server for display through the given channel
 * @param [in] channel        the channel to send the image with
 * @param [in] message          a protobuf message
 * @return PresenterErrorCode
 */
PresenterErrorCode SendMessage(Channel *channel,
                               const google::protobuf::Message& message);

} /* namespace presenter */
} /* namespace ascend */

#endif /* ASCENDDK_PRESENTER_AGENT_PRESENTER_CHANNEL_H_ */
