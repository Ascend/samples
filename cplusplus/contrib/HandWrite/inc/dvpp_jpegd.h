/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File dvpp_process.h
* Description: handle dvpp process
*/
#pragma once
#include <cstdint>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "utils.h"


/**
 * DvppProcess
 */
class DvppJpegD {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    DvppJpegD(aclrtStream &stream,  acldvppChannelDesc *dvppChannelDesc);

    /**
    * @brief Destructor
    */
    ~DvppJpegD();

    /**
    * @brief dvpp global init
    * @return result
    */
    Result InitResource();

    /**
    * @brief init dvpp output para
    * @param [in] modelInputWidth: model input width
    * @param [in] modelInputHeight: model input height
    * @return result
    */
    Result InitOutputPara(int modelInputWidth, int modelInputHeight);

    /**
    * @brief set jpegd input
    * @param [in] inDevBuffer: device buffer of input pic
    * @param [in] inDevBufferSize: device buffer size of input pic
    * @param [in] inputWidth:width of pic
    * @param [in] inputHeight:height of pic
    */
    void SetInput4JpegD(uint8_t* inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight);
    Result InitDecodeOutputDesc(ImageData& inputImage);
    /**
    * @brief gett dvpp output
    * @param [in] outputBuffer: pointer which points to dvpp output buffer
    * @param [out] outputSize: output size
    */
    void GetOutput(void **outputBuffer, int &outputSize);
    Result Process(ImageData& dest, ImageData& src);
   /**
    * @brief release encode resource
    */
    void DestroyEncodeResource();

private:
    void DestroyDecodeResource();
    void DestroyResource();
    void DestroyOutputPara();

    aclrtStream stream_;
    acldvppChannelDesc *dvppChannelDesc_;


    void* decodeOutBufferDev_; // decode output buffer
    acldvppPicDesc *decodeOutputDesc_; //decode output desc


    uint8_t *inDevBuffer_;  // input pic dev buffer
    uint32_t inDevBufferSizeD_; // input pic size for decode

    void *vpcOutBufferDev_; // vpc output buffer
    uint32_t vpcOutBufferSize_;  // vpc output size
};

