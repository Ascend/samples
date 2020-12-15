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

#ifndef PRESENTER_CHANNELS_H
#define PRESENTER_CHANNELS_H

#include "painting_message.pb.h"
#include "acl/acl.h"
#include <mutex>
#include <string>
#include <cstdint>
#include "utils.h"

#include "ascenddk/presenter/agent/presenter_types.h"
#include "ascenddk/presenter/agent/channel.h"
#include "ascenddk/presenter/agent/presenter_channel.h"

struct PresenterServerParams {
  // ip of presenter server
  std::string host_ip;
  // port of presenter server
  std::uint16_t port;
  // name of registered app
  std::string app_id;
  // type of registered app
  std::string app_type;
};

class PresenterChannels {
public:
  static PresenterChannels& GetInstance() {
    static PresenterChannels instance;
    return instance;
  }

  void Init(const PresenterServerParams& param) {
    param_ = param;
  }

  ascend::presenter::Channel* GetChannel() {
    if (intf_channel_ != nullptr) {
      return intf_channel_.get();
    }

    // create agent channel by host_ip and port
    ascend::presenter::ChannelFactory channel_factory;
    ascend::presenter::Channel *agent_channel = channel_factory.NewChannel(
        param_.host_ip, param_.port);

    //open present channel
    ascend::presenter::PresenterErrorCode present_open_err =
        agent_channel->Open();
    if (present_open_err != ascend::presenter::PresenterErrorCode::kNone) {
      return nullptr;
    }

    // register app to presenter server
    PaintingMessage::RegisterApp app_register;
    app_register.set_id(param_.app_id);
    app_register.set_type(param_.app_type);

    // construct responded protobuf Message
    std::unique_ptr < google::protobuf::Message > response;

    // send registered request to server
    ascend::presenter::PresenterErrorCode present_register_err = agent_channel
        ->SendMessage(app_register, response);
    if (present_register_err != ascend::presenter::PresenterErrorCode::kNone) {
      return nullptr;
    }

    // get responded Message and judge result
    PaintingMessage::CommonResponse* register_response =
        dynamic_cast<PaintingMessage::CommonResponse*>(response
            .get());
    if (register_response == nullptr) {
      return nullptr;
    }
    PaintingMessage::ErrorCode register_err =
        register_response->ret();
    if (register_err != PaintingMessage::kErrorNone) {
      return nullptr;
    }

    intf_channel_.reset(agent_channel);

    return intf_channel_.get();
  }

  ascend::presenter::Channel* GetPresenterChannel() {
    // channel already exist, return it
    if (presenter_channel_ != nullptr) {
      return presenter_channel_.get();
    }

    // channel not exist, open it
    ascend::presenter::Channel *ch = nullptr;
    ascend::presenter::OpenChannelParam param;
    param.host_ip = param_.host_ip;
    param.port = param_.port;
    param.channel_name = param_.app_id;
    param.content_type = ascend::presenter::ContentType::kVideo;
    ascend::presenter::PresenterErrorCode error_code =
        ascend::presenter::OpenChannel(ch, param);

    // open channel failed
    if (error_code != ascend::presenter::PresenterErrorCode::kNone) {
//      INFO_LOG( "Open channel failed! %d", error_code);
      INFO_LOG( "Open channel failed!");

      return nullptr;
    }

    // open channel successfully, set it to private parameter
    presenter_channel_.reset(ch);
    return presenter_channel_.get();
  }

private:

  // intf channel for face register
  std::unique_ptr<ascend::presenter::Channel> intf_channel_;

  // presenter channel for camera data
  std::unique_ptr<ascend::presenter::Channel> presenter_channel_;

  // channel params
  PresenterServerParams param_;
};

#endif
