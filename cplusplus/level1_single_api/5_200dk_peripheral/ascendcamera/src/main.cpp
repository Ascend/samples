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
#include "ascend_camera_common.h"

#include <cstdint>
#include <cstdio>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "sample_venc.h"
#include "utils.h"
#include "camera.h"


int main(int argc, char *argv[]) {

  //根据输入参数，若无参数，则开始以H265格式录制视频，存到本地
  if((argc < 2) || (argv[1] == nullptr)) {

    bool bSetChannelId = false;
    int channelId = 1;  //camera port

    //获取camera channel id
    if(bSetChannelId)
    {
      string channelName = string(argv[1]);
      Utils::GetChannelID(channelName, channelId);
      if(0xFF == channelId){
        printf("channelId = %d  ERROR \n", channelId);
        return FAILED;
      }
    }

    Camera_265  cameraDevice(channelId, cameraFps_, inputWidth_, inputHeight_);
    if(false == cameraDevice.IsOpened(channelId))
    {
      if (cameraDevice.Open(channelId)) {
        ERROR_LOG("Failed to open channelId =%d.", channelId);
        return FAILED;
      }
    }

    /* 1. ACL初始化 */
    char aclConfigPath[32] = {'\0'};
    aclError ret_test = aclInit(aclConfigPath);

    /* 2. 运行管理资源申请,包括Device、Context、Stream */
    ret_test = aclrtSetDevice(deviceId_);
    ret_test = aclrtCreateContext(&context_, deviceId_);
    ret_test = aclrtCreateStream(&stream_);

    /* 3. Vdec 资源初始化 */
    // create threadId
    int createThreadErr = pthread_create(&threadId_, nullptr, ThreadFunc, nullptr);
    (void)aclrtSubscribeReport(static_cast<uint64_t>(threadId_), stream_);

    setupVencDesc(inputWidth_, inputHeight_);

    string fileName;

    //将文件同步传送到本地服务器中
    fileName = "./output/" + getTime();

    if(enType_ == H265_MAIN_LEVEL)
    {
      fileName += ".h265";
    }
    else
    {
      fileName += ".h264";
    }

    outFileFp_ = fopen(fileName.c_str(), "ab");
    if(outFileFp_ == nullptr)
    {
      ERROR_LOG("Failed to open  file %s.", fileName.c_str());
      return FAILED;
    }

    //设置录制帧数，当前为50帧
    int restLen = 50;
    //逐张图片编码
    ImageData image;

    while(restLen)
    {
      cameraDevice.Read(channelId, image);
      if (image.data == nullptr) {
        ERROR_LOG("Read image %d failed", channelId);
        return FAILED;
      }
      INFO_LOG("Camera image width %d, Camera image height %d ", image.alignWidth, image.alignHeight);
      acldvppSetPicDescData(encodeInputDesc_, reinterpret_cast<void *>(image.data.get()));
      acldvppSetPicDescSize(encodeInputDesc_, image.size);
      // 执行视频码流编码，编码每帧数据后，系统自动调用callback回调函数将编码后的数据写入文件，再及时释放相关资源
      ret_test = aclvencSendFrame(vencChannelDesc_, encodeInputDesc_, nullptr, vencFrameConfig_, nullptr);
      restLen = restLen - 1;
    }
    if(true == cameraDevice.IsOpened(channelId))
    {
      INFO_LOG("camera device is opened");
      if (cameraDevice.Close(channelId)) {
        ERROR_LOG("Failed to Close channelId =%d.", channelId);
        return FAILED;
      }
    }
    aclvencDestroyFrameConfig(vencFrameConfig_);

    ret_test = aclvencDestroyChannel(vencChannelDesc_);
    aclvencDestroyChannelDesc(vencChannelDesc_);
    vencChannelDesc_ = nullptr;
    (void)aclrtUnSubscribeReport(static_cast<uint64_t>(threadId_), stream_);

    // destory thread
    runFlag = false;
    void *res = nullptr;
    int joinThreadErr = pthread_join(threadId_, &res);

    fclose(outFileFp_);

    ret_test = aclrtDestroyStream(stream_);
    stream_ = nullptr;
    ret_test = aclrtDestroyContext(context_);
    context_ = nullptr;
    ret_test = aclrtResetDevice(deviceId_);
    ret_test = aclFinalize();

    INFO_LOG("Execute sample success");
    //录制成功
    return SUCCESS;
  }

  //若有运行参数，则会执行以下函数。Jepg格式的图片或视频
  int ret = ascend::ascendcamera::kMainProcessOk;
  ascend::ascendcamera::MainProcess main_process;

  // initialization
  ret = main_process.Init(argc, argv);
  if (ret != ascend::ascendcamera::kMainProcessOk) {
    return ret;
  }

  // begin to work
  ret = main_process.Run();
  if (ret != ascend::ascendcamera::kMainProcessOk) {
    return ret;
  }

  return ascend::ascendcamera::kMainProcessOk;

}