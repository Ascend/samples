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

#include "face_register.h"
#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <regex>

#include<iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/types_c.h"
#include<opencv2/core/core.hpp>

#include "face_recognition_params.h"
#include "resource_load.h"

using namespace cv;
using namespace std;
using namespace ascend::presenter;
using namespace ascend::presenter::facial_recognition;


bool FaceRegister::Init() {
    // create agent channel and register app
    Channel* agent_channel = PresenterChannels::GetInstance().GetChannel();
    if (agent_channel == nullptr) {
        ERROR_LOG("register app failed.");
        return false;
    }

    return true;
}

static struct timespec time1 = {0, 0};
static struct timespec time2 = {0, 0};

bool FaceRegister::DoRegisterProcess() {
    // get agent channel
    Channel* agent_channel = PresenterChannels::GetInstance().GetChannel();
    if (agent_channel == nullptr) {
        ERROR_LOG("get agent channel to send failed.");
        return false;
    }

    bool ret = true;
    while (1) {
        // construct registered request Message and read message
        unique_ptr < google::protobuf::Message > response_rec;
        PresenterErrorCode agent_ret = agent_channel->ReceiveMessage(response_rec);
        if (agent_ret == PresenterErrorCode::kNone) {
            FaceInfo* face_register_req = (FaceInfo*) (response_rec.get());
            if (face_register_req == nullptr) {
                ERROR_LOG("[DoRegisterProcess]face_register_req is nullptr");
                continue;
            }
            clock_gettime(CLOCK_REALTIME, &time1);
            // get face id and image
            uint32_t face_image_size = face_register_req->image().size();
            const char* face_image_buff = face_register_req->image().c_str();
            int face_id_size = face_register_req->id().size();
            INFO_LOG("face_id = %s,face_image_size = %d,face_id_size = %d",
                      face_register_req->id().c_str(), face_image_size,
                      face_id_size);

            // ready to send registered info to next component
            shared_ptr < FaceRecognitionInfo > pobj =
                make_shared<FaceRecognitionInfo>();

            // judge size of face id
            if (face_id_size > kMaxFaceIdLength) {
                ERROR_LOG("[DoRegisterProcess]length of face_id beyond range");
                pobj->err_info.err_code = AppErrorCode::kRegister;
                pobj->err_info.err_msg = "length of face_id beyond range";
                // send description information to next engine
                ResourceLoad::GetInstance().SendNextModelProcess("FaceRegister", pobj);

                continue;
            }

            // use dvpp convert jpg image to yuv
            ImageData jpgImage;
            int32_t ch = 0;
            void * buffer = nullptr;
   
            acldvppJpegGetImageInfo(face_image_buff, face_image_size,
                 &(jpgImage.width), &(jpgImage.height), &ch);


            aclError aclRet = acldvppMalloc(&buffer, face_image_size*2);
            jpgImage.data.reset((uint8_t*)buffer, [](uint8_t* p) { acldvppFree((void *)p); });
            jpgImage.size = face_image_size;

            aclrtMemcpy(jpgImage.data.get(), face_image_size, face_image_buff, face_image_size, ACL_MEMCPY_DEVICE_TO_DEVICE);

            //INFO_LOG("[DoRegisterProcess]jpgImage.width %d, jpgImage.height %d jpgImage.size %d", jpgImage.width, jpgImage.height, jpgImage.size);

            ret = ResourceLoad::GetInstance().GetDvpp().CvtJpegToYuv420sp(pobj->org_img, jpgImage);
            // failed, no need to send to presenter
            if(ret == FAILED)
            {
                ERROR_LOG("[DoRegisterProcess]fail to convert jpg to yuv");
                pobj->err_info.err_code = AppErrorCode::kRegister;
                pobj->err_info.err_msg = "fail to convert jpg to yuv";
                // send description information to next component
                ResourceLoad::GetInstance().SendNextModelProcess("FaceRegister", pobj);
                continue;
            }
        
            // 1 indicate the image from register
            pobj->frame.image_source = 1;
            pobj->frame.face_id = face_register_req->id();
            pobj->frame.org_img_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
            // true indicate the image is aligned
            pobj->frame.img_aligned = true;

            pobj->err_info.err_code = AppErrorCode::kNone;

            INFO_LOG("[DoRegisterProcess]jpgImage.width %d, jpgImage.height %d yuvImage_.alignWidth %d yuvImage_.alignHeight %d",
            pobj->org_img.width, pobj->org_img.height, pobj->org_img.alignWidth, pobj->org_img.alignHeight);

            ResourceLoad::GetInstance().SendNextModelProcess("FaceRegister", pobj);
            clock_gettime(CLOCK_REALTIME, &time2);
            cout << "FaceRegister::DoRegisterProcess costtime is: " << (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;
        } else {
            usleep(kSleepInterval);
        }
    }
}


bool FaceRegister::Process()
{
	if (!Init()) {
        ERROR_LOG("Init failed.");
    }
    if (!DoRegisterProcess()) {
        ERROR_LOG("DoRegisterProcess failed.");
    }
    return true;
}
