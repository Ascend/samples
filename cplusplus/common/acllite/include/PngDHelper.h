/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File AclLiteImageProc.h
* Description: handle dvpp process
*/
#ifndef PNGD_HELPER_H
#define PNGD_HELPER_H
#pragma once
#include <cstdint>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "AclLiteUtils.h"

class PngDHelper {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    PngDHelper(aclrtStream &stream,  acldvppChannelDesc *dvppChannelDesc);

    /**
    * @brief Destructor
    */
    ~PngDHelper();

    /**
    * @brief dvpp global init
    * @return result
    */
    AclLiteError InitResource();

    /**
    * @brief init dvpp output para
    * @param [in] modelInputWidth: model input width
    * @param [in] modelInputHeight: model input height
    * @return result
    */
    AclLiteError InitOutputPara(int modelInputWidth, int modelInputHeight);

    /**
    * @brief set pngd input
    * @param [in] inDevBuffer: device buffer of input pic
    * @param [in] inDevBufferSize: device buffer size of input pic
    * @param [in] inputWidth:width of pic
    * @param [in] inputHeight:height of pic
    */
    void SetInput4PngD(uint8_t* inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight);
    AclLiteError InitDecodeOutputDesc(ImageData& inputImage);
    /**
    * @brief gett dvpp output
    * @param [in] outputBuffer: pointer which points to dvpp output buffer
    * @param [out] outputSize: output size
    */
    void GetOutput(void **outputBuffer, int &outputSize);
    AclLiteError Process(ImageData& dest, ImageData& src);

private:
    void DestroyDecodeResource();
    void DestroyResource();
    void DestroyOutputPara();

private:
    aclrtStream stream_;
    void* decodeOutBufferDev_; // decode output buffer
    acldvppPicDesc *decodeOutputDesc_; // decode output desc
    acldvppChannelDesc *dvppChannelDesc_;
};
#endif