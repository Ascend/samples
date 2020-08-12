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

#include "ascenddk/presenter/agent/presenter/presenter_message_helper.h"

#include "ascenddk/presenter/agent/util/logging.h"

using std::string;

namespace ascend {
namespace presenter {

PresenterErrorCode PresenterMessageHelper::CreateOpenChannelRequest(
        proto::OpenChannelRequest& request, const string& channel_name,
        ContentType content_type) {
    // check channel name
    if (channel_name.empty()) {
        AGENT_LOG_ERROR("channel name is empty");
        return PresenterErrorCode::kInvalidParam;
    }

    request.set_channel_name(channel_name);

    // set content type
    if (content_type == ContentType::kImage) {
        request.set_content_type(proto::kChannelContentTypeImage);
    } else if (content_type == ContentType::kVideo) {
        request.set_content_type(proto::kChannelContentTypeVideo);
    } else {
        AGENT_LOG_ERROR("Unsupported content type: %d", content_type);
        return PresenterErrorCode::kInvalidParam;
    }

    return PresenterErrorCode::kNone;
}

bool PresenterMessageHelper::InitPresentImageRequest(
        proto::PresentImageRequest& request, const ImageFrame& image) {
    if (image.format == ImageFormat::kJpeg) {
        request.set_format(proto::kImageFormatJpeg);
    } else {  // other formats is not supported
        AGENT_LOG_ERROR("Unsupported image format: %d", image.format);
        return false;
    }

    // data can not be NULL
    if (image.data == nullptr) {
        AGENT_LOG_ERROR("Image data is NULL");
        return false;
    }

    // size should greater than 0
    if (image.size == 0) {
        AGENT_LOG_ERROR("Image data size is 0");
        return false;
    }

    request.set_width(image.width);
    request.set_height(image.height);

    // set the rectangle attr
    proto::Rectangle_Attr *rectangle_attr = nullptr;
    for (uint32_t i = 0; i < image.detection_results.size(); i++)
    {
	rectangle_attr = request.add_rectangle_list();
	rectangle_attr->mutable_left_top()-> set_x(image.detection_results[i].lt.x);
	rectangle_attr->mutable_left_top()-> set_y(image.detection_results[i].lt.y);
	rectangle_attr->mutable_right_bottom()->set_x(image.detection_results[i].rb.x);
        rectangle_attr->mutable_right_bottom()->set_y(image.detection_results[i].rb.y);
	rectangle_attr->set_label_text(image.detection_results[i].result_text);
    }
    // image.data may be too large to affect performance, so it is not set here
    return true;
}

PresenterErrorCode PresenterMessageHelper::TranslateErrorCode(
        proto::OpenChannelErrorCode error_code) {
    switch (error_code) {
        case proto::kOpenChannelErrorNone:
            return PresenterErrorCode::kNone;
        case proto::kOpenChannelErrorChannelAlreadyOpened:
            return PresenterErrorCode::kChannelAlreadyOpened;
        case proto::kOpenChannelErrorNoSuchChannel:
            return PresenterErrorCode::kNoSuchChannel;
        default:
            return PresenterErrorCode::kServerReturnedUnknownError;
    }
}

PresenterErrorCode PresenterMessageHelper::CheckOpenChannelResponse(
        const ::google::protobuf::Message& msg) {

    string msg_name = msg.GetDescriptor()->full_name();
    // check response
    if (msg_name != proto::OpenChannelResponse::descriptor()->full_name()) {
        AGENT_LOG_ERROR("expecting OpenChannelResponse, but received %s",
                     msg_name.c_str());
        return PresenterErrorCode::kOther;
    }

    const proto::OpenChannelResponse& resp =
            static_cast<const proto::OpenChannelResponse&>(msg);
    return TranslateErrorCode(resp.error_code());
}

PresenterErrorCode PresenterMessageHelper::TranslateErrorCode(
        proto::PresentDataErrorCode error_code) {
    if (error_code == proto::kPresentDataErrorNone) {
        return PresenterErrorCode::kNone;
    }

    AGENT_LOG_ERROR("Present Image failed. error code = %d", error_code);

    if (error_code == proto::kPresentDataErrorUnsupportedFormat ||
            error_code == proto::kPresentDataErrorUnsupportedType) {
        return PresenterErrorCode::kInvalidParam;
    }

    return PresenterErrorCode::kServerReturnedUnknownError;
}

PresenterErrorCode PresenterMessageHelper::CheckPresentImageResponse(
        const ::google::protobuf::Message& msg) {
    // check response
    string msg_name = msg.GetDescriptor()->full_name();

    // if the received message is not of the desired type
    if (msg_name != proto::PresentImageResponse::descriptor()->full_name()) {
        AGENT_LOG_ERROR("expecting PresentImageResponse, but received %s",
                     msg_name.c_str());
        return PresenterErrorCode::kOther;
    }

    const proto::PresentImageResponse& resp =
            static_cast<const proto::PresentImageResponse&>(msg);
    return TranslateErrorCode(resp.error_code());
}

} /* namespace presenter */
} /* namespace ascend */
