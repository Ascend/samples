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

#include "ascenddk/presenter/agent/presenter_channel.h"

#include <memory>
#include <sstream>

#include "ascenddk/presenter/agent/channel/default_channel.h"
#include "ascenddk/presenter/agent/net/raw_socket_factory.h"
#include "ascenddk/presenter/agent/presenter/presenter_channel_init_handler.h"
#include "ascenddk/presenter/agent/presenter/presenter_message_helper.h"
#include "ascenddk/presenter/agent/util/logging.h"
#include "ascenddk/presenter/agent/util/parse_config.h"

using namespace std;
using namespace google::protobuf;

namespace ascend {
namespace presenter {

PresenterErrorCode CreateChannel(Channel *&channel,
                                 const OpenChannelParam &param) {
  std::shared_ptr<PresentChannelInitHandler> handler = make_shared<
      PresentChannelInitHandler>(param);

  DefaultChannel *ch = DefaultChannel::NewChannel(param.host_ip, param.port, handler);
  if (ch == nullptr) {
    AGENT_LOG_ERROR("Channel new() error");
    return PresenterErrorCode::kBadAlloc;
  }

  // OpenChannelParam to string
  std::stringstream ss;
  ss << "PresenterChannelImpl: {";
  ss << "server: " << param.host_ip << ":" << param.port;
  ss << ", channel: " << param.channel_name;
  ss << ", content_type: " << static_cast<int>(param.content_type);
  ss << "}";
  ch->SetDescription(ss.str());
  channel = ch;
  return PresenterErrorCode::kNone;
}

PresenterErrorCode OpenChannel(Channel *&channel,
                               const OpenChannelParam &param) {

  // If the channel is not NULL, we cannot know whether it is actually
  // point to something. We cannot be sure whether it is safe to simply
  // delete that, so a kPresenterErrorInvalidParams will be returned
  if (channel != nullptr) {
    AGENT_LOG_ERROR("channel is not NULL");
    return PresenterErrorCode::kInvalidParam;
  }

  // allocate channel object
  PresenterErrorCode error_code = CreateChannel(channel, param);
  if (error_code != PresenterErrorCode::kNone) {
    return error_code;
  }

  string channelDesc = channel->GetDescription();

  // Try Open Channel
  AGENT_LOG_INFO("To Open channel: %s", channelDesc.c_str());

  error_code = channel->Open();

  // If failed, the channel object need to be released
  if (error_code != PresenterErrorCode::kNone) {
    if (error_code == PresenterErrorCode::kAppDefinedError) {
      DefaultChannel *ch = dynamic_cast<DefaultChannel*>(channel);
      error_code = dynamic_pointer_cast<const PresentChannelInitHandler>(
          ch->GetInitChannelHandler())->GetErrorCode();
    }

    AGENT_LOG_ERROR("OpenChannel Failed, channel = %s, error_code = %d",
                    channelDesc.c_str(), error_code);
    delete channel;
    channel = nullptr;
    return error_code;
  }

  AGENT_LOG_INFO("Channel opened, channel = %s", channelDesc.c_str());
  return PresenterErrorCode::kNone;
}

PresenterErrorCode OpenChannelByConfig(Channel*& channel,
									   const char* configFile) {
	map<string, string> config;
	ReadConfig(config, configFile);

	OpenChannelParam param;
	map<string, string>::const_iterator mIter = config.begin();
	for (; mIter != config.end(); ++mIter) {
		if (mIter->first == "presenter_server_ip") {
			param.host_ip = mIter->second;
                        AGENT_LOG_INFO("presenter_server_ip config string:%s", mIter->second.c_str());
                }
		else if (mIter->first == "presenter_server_port") {
			param.port = std::stoi(mIter->second);
                        AGENT_LOG_INFO("presenter_server_port config string:%s", mIter->second.c_str());
                }
		else if (mIter->first == "channel_name") {
			param.channel_name = mIter->second;
                        AGENT_LOG_INFO("channel_name config string:%s", mIter->second.c_str());
                }
		else if (mIter->first == "content_type") {
			param.content_type = static_cast<ContentType>(std::stoi(mIter->second));
                        AGENT_LOG_INFO("content_type config string:%s", mIter->second.c_str());
                        printf("content_type config string:%s\n", mIter->second.c_str());
                }
	}

	return OpenChannel(channel, param);
}

PresenterErrorCode PresentImage(Channel *channel, const ImageFrame &image) {
  if (channel == nullptr) {
    AGENT_LOG_ERROR("channel is NULL");
    return PresenterErrorCode::kInvalidParam;
  }

  proto::PresentImageRequest req;
  if (!PresenterMessageHelper::InitPresentImageRequest(req, image)) {
    return PresenterErrorCode::kInvalidParam;
  }

  Tlv tlv;
  tlv.tag = proto::PresentImageRequest::kDataFieldNumber;
  tlv.length = image.size;
  tlv.value = reinterpret_cast<char *>(image.data);

  PartialMessageWithTlvs message;
  message.message = &req;
  message.tlv_list.push_back(tlv);

  std::unique_ptr<Message> recv_message;
  PresenterErrorCode error_code = channel->SendMessage(message, recv_message);
  if (error_code != PresenterErrorCode::kNone) {
    AGENT_LOG_ERROR("Failed to present image, error = %d", error_code);
    return error_code;
  }

  return PresenterMessageHelper::CheckPresentImageResponse(*recv_message);
}

PresenterErrorCode SendMessage(
        Channel *channel, const google::protobuf::Message& message) {
    if (channel == nullptr) {
        AGENT_LOG_ERROR("channel is NULL");
        return PresenterErrorCode::kInvalidParam;
    }

    unique_ptr<google::protobuf::Message> resp;
    PresenterErrorCode error_code = channel->SendMessage(message, resp);
    if (error_code != PresenterErrorCode::kNone) {
        AGENT_LOG_ERROR("Failed to present image, error = %d", error_code);
        return error_code;
    }

    return PresenterMessageHelper::CheckPresentImageResponse(*resp);
}

}
}

