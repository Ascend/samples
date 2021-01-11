/**
 * ============================================================================
 *
 * Copyright (C) 2018-2020, Hisilicon Technologies Co., Ltd. All Rights Reserved.
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

#include "face_post_process.h"
#include "utils.h"

#include <memory>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <string>
#include "resource_load.h"

using namespace std;
using namespace google::protobuf;

Result FacePostProcess::CheckSendMessageRes(
    const PresenterErrorCode &error_code) {
    if (error_code == PresenterErrorCode::kNone) {

        return SUCCESS;
    }

    INFO_LOG("send message to presenter server failed. error=%d", int(error_code));
    return FAILED;
}

//static ImageData jpgImage;
Result FacePostProcess::GetOriginPic(
    const shared_ptr<FaceRecognitionInfo> &image_handle, ImageData &jpgImage, facial_recognition::FrameInfo &frame_info) {
    if(image_handle->frame.image_source == 0) {
	    Result ret = ResourceLoad::GetInstance().GetDvpp().CvtYuv420spToJpeg(jpgImage, image_handle->org_img);
	    // failed, no need to send to presenter
	    if(ret == FAILED)
	    {
            ERROR_LOG("Failed to convert NV12 to JPEG, skip this frame.");
            return FAILED;
        }

        frame_info.set_image(string(reinterpret_cast < char* > (jpgImage.data.get()), jpgImage.size));
    }
    return SUCCESS;
}
static int memcount = 0;
Result FacePostProcess::SendFeature(
    const shared_ptr<FaceRecognitionInfo> &info) {
    // get channel for send feature (data from camera)
    Channel *channel = PresenterChannels::GetInstance().GetPresenterChannel();
    if (channel == nullptr) {
        ERROR_LOG("get channel for send FrameInfo failed.");
        return FAILED;
    }

    // front engine deal failed, skip this frame
    if (info->err_info.err_code != AppErrorCode::kNone) {
        INFO_LOG("front engine dealing failed, skip this frame, err_code=%d, err_msg=%s",
        int(info->err_info.err_code), info->err_info.err_msg.c_str());
        return FAILED;
    }

    ImageData jpgImage;

    facial_recognition::FrameInfo frame_info;
    if(GetOriginPic(info, jpgImage, frame_info) != SUCCESS){
        info->err_info.err_code = AppErrorCode::kRecognition;
        info->err_info.err_msg = "Get the original pic failed";

        ERROR_LOG("Engine handle filed, err_code=%d, err_msg=%s",
        int(info->err_info.err_code),
        info->err_info.err_msg.c_str());
    }

    // 2. repeated FaceFeature
    vector<FaceImage> face_imgs = info->face_imgs;
    facial_recognition::FaceFeature *feature = nullptr;
    for (int i = 0; i < face_imgs.size(); i++) {
        // every face feature
        feature = frame_info.add_feature();
        // box
        feature->mutable_box()->set_lt_x(face_imgs[i].rectangle.lt.x);
        feature->mutable_box()->set_lt_y(face_imgs[i].rectangle.lt.y);
        feature->mutable_box()->set_rb_x(face_imgs[i].rectangle.rb.x);
        feature->mutable_box()->set_rb_y(face_imgs[i].rectangle.rb.y);

        INFO_LOG("position is (%d,%d),(%d,%d)",face_imgs[i].rectangle.lt.x,face_imgs[i].rectangle.lt.y,face_imgs[i].rectangle.rb.x,face_imgs[i].rectangle.rb.y);

        // vector
        for (int j = 0; j < face_imgs[i].feature_vector.size(); j++) {
            feature->add_vector(face_imgs[i].feature_vector[j]);
        }
    }

    // send frame information to presenter server
    unique_ptr<Message> resp;
    PresenterErrorCode error_code;
    error_code = channel->SendMessage(frame_info, resp);

    return CheckSendMessageRes(error_code);
    //return SUCCESS;
}

Result FacePostProcess::ReplyFeature(
    const shared_ptr<FaceRecognitionInfo> &info) {
      // get channel for reply feature (data from register)
    Channel *channel = PresenterChannels::GetInstance().GetChannel();
    if (channel == nullptr) {
        ERROR_LOG("get channel for send FaceResult failed.");
        return FAILED;
    }

    // generate FaceResult
    facial_recognition::FaceResult result;
    result.set_id(info->frame.face_id);
    unique_ptr<Message> resp;
    INFO_LOG("FacePostProcess::ReplyFeature begin.");
    // 1. front engine dealing failed, send error message
    if (info->err_info.err_code != AppErrorCode::kNone) {
        INFO_LOG("front engine dealing failed, reply error response to server");

        result.mutable_response()->set_ret(facial_recognition::kErrorOther);
        result.mutable_response()->set_message(info->err_info.err_msg);

        // send
        PresenterErrorCode error_code = channel->SendMessage(result, resp);
        return CheckSendMessageRes(error_code);
    }
    // 2. dealing success, need set FaceFeature
    result.mutable_response()->set_ret(facial_recognition::kErrorNone);
    vector<FaceImage> face_imgs = info->face_imgs;
    facial_recognition::FaceFeature *face_feature = nullptr;
    for (int i = 0; i < face_imgs.size(); i++) {
        // every face feature
        face_feature = result.add_feature();
        // box
        face_feature->mutable_box()->set_lt_x(face_imgs[i].rectangle.lt.x);
        face_feature->mutable_box()->set_lt_y(face_imgs[i].rectangle.lt.y);
        face_feature->mutable_box()->set_rb_x(face_imgs[i].rectangle.rb.x);
        face_feature->mutable_box()->set_rb_y(face_imgs[i].rectangle.rb.y);

        // vector
        for (int j = 0; j < face_imgs[i].feature_vector.size(); j++) {
            face_feature->add_vector(face_imgs[i].feature_vector[j]);
        }
    }
    PresenterErrorCode error_code = channel->SendMessage(result, resp);
    INFO_LOG("FacePostProcess::ReplyFeature end.");
    return CheckSendMessageRes(error_code);
}

Result FacePostProcess::Process(const std::shared_ptr<FaceRecognitionInfo> &image_handle) {
	
    // deal data from camera
    if (image_handle->frame.image_source == 0) {

        return SendFeature(image_handle);
    }

    // deal data from register
    ReplyFeature(image_handle);
    INFO_LOG( "post process dealing data from register end.");
    return SUCCESS;
}
