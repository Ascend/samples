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

#ifndef FACE_REGISTER_ENGINE_H_
#define FACE_REGISTER_ENGINE_H_
#include <iostream>
#include <string>
#include <dirent.h>
#include <memory>
#include <unistd.h>
#include <vector>
#include <stdint.h>
#include "face_recognition_params.h"
#include "presenter_channels.h"

//#define INPUT_SIZE 1
//#define OUTPUT_SIZE 1

const int32_t kMaxFaceIdLength = 50;

// sleep interval when queue full (unit:microseconds)
const __useconds_t kSleepInterval = 200000;

// function of dvpp returns success
const int kDvppOperationOk = 0;

// IP regular expression
const std::string kIpRegularExpression =
    "^(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|[1-9])\\."
        "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)\\."
        "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)\\."
        "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)$";

// port number range
const int32_t kPortMinNumber = 1024;
const int32_t kPortMaxNumber = 49151;

// app name regular expression
const std::string kAppNameRegularExpression = "[a-zA-Z0-9_]{3,20}";

// presenter server ip
const std::string kPresenterServerIP = "presenter_server_ip";

// presenter server port
const std::string kPresenterServerPort = "presenter_server_port";

// app name
const std::string kAppName = "app_name";

// app type
const std::string kAppType = "facial_recognition";

class FaceRegister {
public:
  /**
   * @brief   constructor
   */
    FaceRegister() {
    }

  /**
   * @brief   destructor
   */
    ~FaceRegister() = default;

    /**
   * @brief  Process
   */
    bool Process();

private:
  // Private implementation a member variable,
  // which is used to cache the input queue
  //hiai::MultiTypeQueue input_que_;
  
  /**
   * @brief  init config of face register by aiConfig
   * @return  success --> true ; fail --> false
   */
    bool Init();

  /**
   * @brief  handle registered image and send to next engine
   * @return  success --> true ; fail --> false
   */
    bool DoRegisterProcess();
};

#endif
