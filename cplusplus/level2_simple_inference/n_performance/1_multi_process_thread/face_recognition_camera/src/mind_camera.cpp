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

#include "mind_camera.h"

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <cstring>
#include <chrono>

#include "resource_load.h"

extern "C" {
#include "driver/peripheral_api.h"
}

using namespace std;

namespace {
    // initial value of frameId
    const uint32_t kInitFrameId = 0;

}

MindCamera::MindCamera() {
    config_ = nullptr;
    frame_id_ = kInitFrameId;
    exit_flag_ = CAMERADATASETS_INIT;
    InitConfigParams();
}

MindCamera::~MindCamera() {
}

string MindCamera::CameraDatasetsConfig::ToString() const {
    stringstream log_info_stream("");
    log_info_stream << "fps:" << this->fps << ", camera:" << this->channel_id
    << ", image_format:" << this->image_format
    << ", resolution_width:" << this->resolution_width
    << ", resolution_height:" << this->resolution_height;

    return log_info_stream.str();
}

bool MindCamera::Init() {
    INFO_LOG("[CameraDatasets] start init!");
    if (config_ == nullptr) {
        config_ = make_shared<CameraDatasetsConfig>();
    }

    config_->fps = 10;
    config_->image_format = CommonParseParam("YUV420SP");
    config_->channel_id = CommonParseParam("Channel-1");
    ParseImageSize("1280x720", config_->resolution_width,
    //ParseImageSize("352x288", config_->resolution_width,
    config_->resolution_height);
    bool ret = true;
    bool failed_flag = (config_->image_format == PARSEPARAM_FAIL
    || config_->channel_id == PARSEPARAM_FAIL
    || config_->resolution_width == 0 || config_->resolution_height == 0);

    if (failed_flag) {
        string msg = config_->ToString();
        msg.append(" config data failed");
        ret = false;
    }

    INFO_LOG("[CameraDatasets] end init!");
    return ret;
}

void MindCamera::InitConfigParams() {
    params_.insert(pair<string, string>("Channel-1", IntToString(CAMERAL_1)));
    params_.insert(pair<string, string>("Channel-2", IntToString(CAMERAL_2)));
    params_.insert(
    pair<string, string>("YUV420SP", IntToString(CAMERA_IMAGE_YUV420_SP)));
}

string MindCamera::IntToString(int value) {
    char msg[MAX_VALUESTRING_LENGTH] = { 0 };
    // MAX_VALUESTRING_LENGTH ensure no error occurred
    sprintf(msg, "%d", value);
    string ret = msg;

    return ret;
}

int MindCamera::CommonParseParam(const string& val) const {
    map<string, string>::const_iterator iter = params_.find(val);
    if (iter != params_.end()) {
        return atoi((iter->second).c_str());
    }

    return PARSEPARAM_FAIL;
}

void MindCamera::SplitString(const string& source, vector<string>& tmp,
const string& obj) {
    string::size_type pos1 = 0;
    string::size_type pos2 = source.find(obj);

    while (string::npos != pos2) {
        tmp.push_back(source.substr(pos1, pos2 - pos1));
        pos1 = pos2 + obj.size();
        pos2 = source.find(obj, pos1);
    }

    if (pos1 != source.length()) {
        tmp.push_back(source.substr(pos1));
    }
}

void MindCamera::ParseImageSize(const string& val, int& width,
int& height) const {
    vector<string> tmp;
    SplitString(val, tmp, "x");

    // val is not a format of resolution ratio(*x*),correct should have 2 array
    // in this wrong case,set width and height zero
    if (tmp.size() != 2) {
        width = 0;
        height = 0;
    } else {
        width = atoi(tmp[0].c_str());
        height = atoi(tmp[1].c_str());
    }
}

MindCamera::CameraOperationCode MindCamera::PreCapProcess() {
    MediaLibInit();

    CameraStatus status = QueryCameraStatus(config_->channel_id);
    if (status != CAMERA_STATUS_CLOSED) {
        ERROR_LOG("[CameraDatasets] PreCapProcess.QueryCameraStatus "
        "{status:%d} failed.",
        status);
        return kCameraNotClosed;
    }

    // Open Camera
    int ret = OpenCamera(config_->channel_id);
    // return 0 indicates failure
    if (ret == 0) {
        ERROR_LOG("[CameraDatasets] PreCapProcess OpenCamera {%d} "
        "failed.",
        config_->channel_id);
        return kCameraOpenFailed;
    }

    // set fps
    ret = SetCameraProperty(config_->channel_id, CAMERA_PROP_FPS,
    &(config_->fps));
    // return 0 indicates failure
    if (ret == 0) {
        ERROR_LOG("[CameraDatasets] PreCapProcess set fps {fps:%d} "
        "failed.",
        config_->fps);
        return kCameraSetPropertyFailed;
    }

    // set image format
    ret = SetCameraProperty(config_->channel_id, CAMERA_PROP_IMAGE_FORMAT,
    &(config_->image_format));
    // return 0 indicates failure
    if (ret == 0) {
        ERROR_LOG("[CameraDatasets] PreCapProcess set image_fromat "
        "{format:%d} failed.",
        config_->image_format);
        return kCameraSetPropertyFailed;
    }

    // set image resolution.
    CameraResolution resolution;
    resolution.width = config_->resolution_width;
    resolution.height = config_->resolution_height;
    ret = SetCameraProperty(config_->channel_id, CAMERA_PROP_RESOLUTION,
    &resolution);
    // return 0 indicates failure
    if (ret == 0) {
        ERROR_LOG("[CameraDatasets] PreCapProcess set resolution "
        "{width:%d, height:%d } failed.",
        config_->resolution_width, config_->resolution_height);
        return kCameraSetPropertyFailed;
    }

    // set work mode
    CameraCapMode mode = CAMERA_CAP_ACTIVE;
    ret = SetCameraProperty(config_->channel_id, CAMERA_PROP_CAP_MODE, &mode);
    // return 0 indicates failure
    if (ret == 0) {
        ERROR_LOG("[CameraDatasets] PreCapProcess set cap mode {mode:%d}"
        " failed.", mode);
        return kCameraSetPropertyFailed;
    }

    return kCameraOk;
}

shared_ptr<FaceRecognitionInfo> MindCamera::CreateBatchImageParaObj() {
    shared_ptr<FaceRecognitionInfo> pObj = make_shared<FaceRecognitionInfo>();

    pObj->frame.image_source = 0;
    // handle one image frame every time
    pObj->frame.channel_id = config_->channel_id;
    pObj->frame.frame_id = frame_id_++;
    pObj->frame.timestamp = time(nullptr);
    pObj->org_img.width = config_->resolution_width;
    pObj->org_img.height = config_->resolution_height;
    pObj->org_img.alignWidth = ALIGN_UP16(config_->resolution_width);
    pObj->org_img.alignHeight = ALIGN_UP2(config_->resolution_height);
    // YUV size in memory is width*height*3/2
    pObj->org_img.size = config_->resolution_width * config_->resolution_height
    * 3 / 2;

    void * buffer = nullptr;
    aclError aclRet = acldvppMalloc(&buffer, pObj->org_img.size);
    pObj->org_img.data.reset((uint8_t*)buffer, [](uint8_t* p) { acldvppFree((void *)p); });
    return pObj;
}

static struct timespec time1 = {0, 0};
static struct timespec time2 = {0, 0};

bool MindCamera::DoCapProcess() {
    CameraOperationCode ret_code = PreCapProcess();
    if (ret_code == kCameraSetPropertyFailed) {
        CloseCamera(config_->channel_id);

        ERROR_LOG("[CameraDatasets] DoCapProcess.PreCapProcess failed");
        return false;
    } else if((ret_code == kCameraOpenFailed) || (ret_code == kCameraNotClosed)) {

        ERROR_LOG("[CameraDatasets] DoCapProcess.PreCapProcess failed");
        return false;
    } 

    // set procedure is running.
    SetExitFlag(CAMERADATASETS_RUN);

    bool ret = true;
    int read_ret = 0;
    int read_size = 0;
    bool read_flag = false;
    void * buffer = nullptr;
    int size = config_->resolution_width * config_->resolution_height
    * 3 / 2;
    int numcount = 100;
    while (GetExitFlag() == CAMERADATASETS_RUN) {
        clock_gettime(CLOCK_REALTIME, &time1);
        //
        shared_ptr<FaceRecognitionInfo> p_obj = CreateBatchImageParaObj();
        uint8_t* p_data = p_obj->org_img.data.get();
        read_size = (int) p_obj->org_img.size;

        // do read frame from camera, readSize maybe changed when called
        read_ret = ReadFrameFromCamera(config_->channel_id, (void*) p_data, &read_size);

        // indicates failure when readRet is 1
        read_flag = ((read_ret == 1) && (read_size == (int) p_obj->org_img.size));

        if (!read_flag) {
            ERROR_LOG("[CameraDatasets] readFrameFromCamera failed "
            "{camera:%d, ret:%d, size:%d, expectsize:%d} ",
            config_->channel_id, read_ret, read_size,
            (int )p_obj->org_img.size);
            break;
        }

        //INFO_LOG("MindCamera DoCapProcess  width %d, height %d, al w %d h %d size %d\n", p_obj->org_img.width, p_obj->org_img.height, p_obj->org_img.alignWidth, p_obj->org_img.alignHeight, p_obj->org_img.size);
        ResourceLoad::GetInstance().SendNextModelProcess("MindCamera", p_obj);
        clock_gettime(CLOCK_REALTIME, &time2);

        numcount --;
    }

    // close camera
    CloseCamera(config_->channel_id);

    if (ret != true) {
        return false;
    }

    return true;
}

void MindCamera::SetExitFlag(int flag) {
    TLock lock(mutex_);
    exit_flag_ = flag;
}

int MindCamera::GetExitFlag() {
    TLock lock(mutex_);
    return exit_flag_;
}

bool MindCamera::Process() {
    INFO_LOG("[CameraDatasets] start process!");
    Init();
    DoCapProcess();
    INFO_LOG("[CameraDatasets] end process!");
    return true;
}


