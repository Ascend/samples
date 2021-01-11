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

#ifndef FACE_POST_PROCESS_H_
#define FACE_POST_PROCESS_H_

#include "face_recognition_params.h"

#include <vector>
#include <stdint.h>

#include "facial_recognition_message.pb.h"
#include "ascenddk/presenter/agent/presenter_channel.h"
#include "presenter_channels.h"

using namespace ascend::presenter;

class FacePostProcess {
public:

  /**
   * @brief: engine processor which override HIAI engine
   *         inference every image, and then send data to post process
   * @param [in]: input size
   * @param [in]: output size
   */
	Result Process(const std::shared_ptr<FaceRecognitionInfo> &info);

private:

  /**
   * @brief: check send message to presenter server result
   * @param [in]: send message result
   * @return: Result
   */
    Result CheckSendMessageRes(
        const ascend::presenter::PresenterErrorCode &error_code);
  /**
   * @brief: send feature to presenter server
   * @param [in]: FaceRecognitionInfo
   * @return: Result
   */
    Result SendFeature(const std::shared_ptr<FaceRecognitionInfo> &info);

  /**
   * @brief: reply feature to presenter server
   * @param [in]: FaceRecognitionInfo
   * @return: Result
   */
    Result ReplyFeature(const std::shared_ptr<FaceRecognitionInfo> &info);
		

    Result GetOriginPic(
    const shared_ptr<FaceRecognitionInfo> &image_handle, ImageData &jpgImage, facial_recognition::FrameInfo &frame_info);

};

#endif
