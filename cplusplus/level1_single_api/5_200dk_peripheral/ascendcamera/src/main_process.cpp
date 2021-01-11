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

#include "main_process.h"

#include <time.h>
#include <unistd.h>
#include <stdio.h>

#include "camera.h"
#include "output_info_process.h"
#include "ascend_camera_common.h"
#include "acl/acl.h"

using namespace std;

namespace ascend {
namespace ascendcamera {
MainProcess::MainProcess():deviceId_(0), context_(nullptr), stream_(nullptr) {
  // constuct a instance used to control the whole process.
  control_object_.ascend_camera_paramter = nullptr;
  control_object_.camera = nullptr;
  control_object_.dvpp_process = nullptr;
  control_object_.output_process = nullptr;
  control_object_.loop_flag = kNoNeedLoop;

  // initialization debugging info
  debug_info_.total_frame = 0;
  debug_info_.queue_max_length = 0;
}

MainProcess::~MainProcess() {
  // release all
  if (control_object_.ascend_camera_paramter != nullptr) {
    delete control_object_.ascend_camera_paramter;
    control_object_.ascend_camera_paramter = nullptr;
  }

  if (control_object_.camera != nullptr) {
    delete control_object_.camera;
    control_object_.camera = nullptr;
  }

  if (control_object_.dvpp_process != nullptr) {
    delete control_object_.dvpp_process;
    control_object_.dvpp_process = nullptr;
  }

  if (control_object_.output_process != nullptr) {
    delete control_object_.output_process;
    control_object_.dvpp_process = nullptr;
  }

  DestroyResource();
}

int MainProcess::OutputInstanceInit(int width, int height) {
  int ret = kMainProcessOk;

  OutputInfoPara output_para;
  if (control_object_.ascend_camera_paramter->GetMediaType() == kImage) {
    output_para.presenter_para.content_type =
        ascend::presenter::ContentType::kImage;
  } else if (control_object_.ascend_camera_paramter->GetMediaType() == kVideo) {
    output_para.presenter_para.content_type =
        ascend::presenter::ContentType::kVideo;
  }

  string str = control_object_.ascend_camera_paramter->GetOutputFile();
  // if user input "-o",it means output to stdout or save file to local
  if (!str.empty()) {
    // ouput to stdout or save file to local.
    output_para.mode = (str == "-") ? kOutputToStdout : kOutputToLocal;
    output_para.path = (str == "-") ? "" : str;
    output_para.width = width;
    output_para.height = height;
    control_object_.output_process = new OutputInfoProcess(output_para);

    // open output channel
    ret = control_object_.output_process->OpenOutputChannel();
  } else {  // if user input "-s",it means ouput to presenter.
    string str = control_object_.ascend_camera_paramter->GetOutputPresenter();
    if (!str.empty()) {
      // x.x.x.x:yyy/zzz/zz
      // x.x.x.x--ip   yyy--port   zzz/zz--channelName
      output_para.presenter_para.host_ip = str.substr(
          kFirstIndex, str.find(kPortIpSeparator, kFirstIndex));

      output_para.presenter_para.port = stoi(
          str.substr(
              str.find(kPortIpSeparator, kFirstIndex) + 1,
              str.find_first_of(KPortChannelSeparator, kFirstIndex)
                  - str.find(kPortIpSeparator, kFirstIndex) - 1));

      output_para.presenter_para.channel_name = str.substr(
          str.find_first_of(KPortChannelSeparator, kFirstIndex) + 1,
          str.length() - str.find_first_of(KPortChannelSeparator, kFirstIndex));

      output_para.mode = kOutputToPresenter;
      output_para.path = "";
      output_para.width = width;
      output_para.height = height;
      control_object_.output_process = new OutputInfoProcess(output_para);

      // open output channel
      ret = control_object_.output_process->OpenOutputChannel();
    }
  }

  if (ret != kMainProcessOk) {
    control_object_.output_process->PrintErrorInfo(ret);
  }
  return ret;
}

void MainProcess::DvppInstanceInit(int width, int height) {
  // 1. In mode of video to presenter server, it need picture mode.
  // 2. The user points to picture.
  string str = control_object_.ascend_camera_paramter->GetOutputPresenter();
  bool is_need_convert_to_jpg = (!str.empty())
      && (control_object_.ascend_camera_paramter->GetMediaType() == kVideo);

  // instance(convert to jpg)
  if (is_need_convert_to_jpg
      || (control_object_.ascend_camera_paramter->GetMediaType() == kImage)) {
   /* ascend::utils::DvppToJpgPara dvpp_to_jpg_para;

    dvpp_to_jpg_para.format = JPGENC_FORMAT_NV12;
    dvpp_to_jpg_para.level = kDvppToJpgQualityParameter;
    dvpp_to_jpg_para.resolution.height = height;
    dvpp_to_jpg_para.resolution.width = width;
    control_object_.dvpp_process = new ascend::utils::DvppProcess(
        dvpp_to_jpg_para);*/
    if (is_need_convert_to_jpg) {
      // if output object is presenter and the mode is video ,
      // dvpp chose picture mode and need Continuous convert yuv to jpg
      control_object_.loop_flag = kNeedLoop;
    }
  }
}

void MainProcess::CameraInstanceInit(int width, int height) {
  CameraPara camera_para;

  // camera instance paramter
  camera_para.fps = control_object_.ascend_camera_paramter->GetFps();
  camera_para.capture_obj_flag = control_object_.ascend_camera_paramter
      ->GetMediaType();
  camera_para.channel_id = control_object_.ascend_camera_paramter
      ->GetCameraChannel();
  camera_para.image_format = CAMERA_IMAGE_YUV420_SP;
  camera_para.timeout = control_object_.ascend_camera_paramter->GetTimeout();
  camera_para.resolution.width = width;
  camera_para.resolution.height = height;

  // camera instance
  control_object_.camera = new Camera(camera_para);
}

int MainProcess::Init(int argc, char *argv[]) {

  int ret = InitResource();
  if (ret != SUCCESS) {
    ERROR_LOG("Init acl resource failed\n");
    return FAILED;
  }
  control_object_.dvpp_process = new DvppProcess();
  ret= control_object_.dvpp_process->InitResource(stream_);
  if (ret != SUCCESS) {
    ERROR_LOG("Init acl resource failed\n");
    return FAILED;
  }



  control_object_.ascend_camera_paramter = new AscendCameraParameter();
  if (!(control_object_.ascend_camera_paramter->Init(argc, argv))) {
    return kParaCheckError;
  }

  // verify ascendcamera_parameter parameters
  if (!(control_object_.ascend_camera_paramter->Verify())) {
    return kParaCheckError;
  }

  int width = control_object_.ascend_camera_paramter->GetImageWidth();
  int height = control_object_.ascend_camera_paramter->GetImageHeight();

    control_object_.Image.width = width;
    control_object_.Image.height = height;

  // camera instance
  CameraInstanceInit(width, height);
  // dvpp controller instance
  DvppInstanceInit(width, height);
  // output instance
  ret = OutputInstanceInit(width, height);
  if (ret != kMainProcessOk) {
    return ret;
  }

  return ret;
}

int MainProcess::DvppAndOutputProc(CameraOutputPara *output_para,
                                  DvppProcess *dvpp, ImageData &image,
                                  OutputInfoProcess *output_info_process) {

  // DVPP convert to jpg
  ImageData jpgImage;
  
  image.data = output_para->data;
  cout << "stream0 = "  << stream_ << endl;
  Result rett = dvpp->CvtYuv420spToJpeg(jpgImage, image);
    if(rett== FAILED) {
        ERROR_LOG("Convert jpeg to yuv failed\n");
        return FAILED;
    }

  // output to channel
  int ret = output_info_process->SendToChannel(jpgImage.data.get(),
                                           jpgImage.size);
  if (ret != kMainProcessOk) {
    output_info_process->PrintErrorInfo(ret);
  }

  return ret;
}

int MainProcess::DoOnce() {
  CameraOutputPara output_para;

  // get a frame from camera
  int ret = control_object_.camera->CaptureCameraInfo(&output_para);
  if (ret != kMainProcessOk) {
    control_object_.camera->PrintErrorInfo(ret);
    return ret;
  }

  ret = DvppAndOutputProc(&output_para, control_object_.dvpp_process, control_object_.Image,
                         control_object_.output_process);
  if (ret != kMainProcessOk) {
    return ret;
  }
  return ret;
}

int MainProcess::Run() {

  if ((control_object_.camera == nullptr)
      || (control_object_.ascend_camera_paramter == nullptr)
      || (control_object_.dvpp_process == nullptr)
      || (control_object_.output_process == nullptr)) {
    return kParaCheckError;
  }

  // init driver of camera and open camera.
  int ret = control_object_.camera->InitCamera();
  if (ret != kCameraInitOk) {
    control_object_.camera->PrintErrorInfo(ret);
    return ret;
  } else {  // print to terminal
    cerr << "[INFO] Success to open camera["
         << control_object_.camera->GetChannelId() << "],and start working."
         << endl;
    ASC_LOG_INFO("Success to open camera[%d],and start working.",
                 control_object_.camera->GetChannelId());
  }

  int timeout = control_object_.camera->GetUserTimeout();

  // get begin time
  struct timespec begin_time = { 0, 0 };
  struct timespec current_time = { 0, 0 };
  long long running_time = 0;
  clock_gettime(CLOCK_MONOTONIC, &begin_time);
  do {
    // get current time
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    // get running time
    running_time = (current_time.tv_sec - begin_time.tv_sec) * kSecToMillisec
        + current_time.tv_nsec / kMillSecToNanoSec
        - begin_time.tv_nsec / kMillSecToNanoSec;
    // get and deal with a frame from camera.
    ret = DoOnce();
    if (ret != kMainProcessOk) {
      cerr << "[ERROR] Failed to complete the task." << endl;
      ASC_LOG_ERROR("Failed to complete the task.");
      break;
    }

    // we exit the loop if The running time more than the specified time.
    if ((running_time > (long long) timeout * kSecToMillisec)
        && (timeout != 0)) {
      cerr << "[INFO] Success to complete the task." << endl;
      ASC_LOG_ERROR("Success to complete the task.");
      break;
    }
    // record debugging info.
    debug_info_.total_frame++;
    if (debug_info_.total_frame % kDebugRecordFrameThresHold == 0) {
      ASC_LOG_INFO("total frame = %ld,running time = %lldms,"
          "Queue Maximum length = %d.", debug_info_.total_frame, running_time,
          debug_info_.queue_max_length);
    }
  } while (control_object_.loop_flag == kNeedLoop);
  ExitProcess(ret);

  return ret;
}

void MainProcess::ExitProcess(int ret) {
  // close camera.
  if (control_object_.camera != nullptr) {
    CloseCamera(control_object_.camera->GetChannelId());
    // close camera
    cerr << "[INFO] Close camera [" << control_object_.camera->GetChannelId()
         << "]." << endl;
    ASC_LOG_INFO("close camera[%d].", control_object_.camera->GetChannelId());
  }

  // close the output channel.
  if (control_object_.output_process != nullptr) {
    control_object_.output_process->CloseChannel();
  }
}
Result MainProcess::InitResource() {
    // ACL init
    const char* aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed\n");
        return FAILED;
    }
    INFO_LOG("acl init success\n");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl open device %d failed\n", deviceId_);
        return FAILED;
    }
    INFO_LOG("open device %d success\n", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed\n");
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create stream failed\n");
        return FAILED;
    }
    INFO_LOG("create stream success");
    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed\n");
        return FAILED;
    }
    return SUCCESS;
}
void MainProcess::DestroyResource()
{
    aclError ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed\n");
    }
    INFO_LOG("end to reset device is %d\n", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed\n");
    }
    INFO_LOG("end to finalize acl");
    //aclrtFree(imageInfoBuf_);
}
}
}
