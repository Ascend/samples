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

* File utils.h
* Description: handle file operations
*/
#ifndef ACLLITE_VIDEO_PROC_H
#define ACLLITE_VIDEO_PROC_H
#pragma once

#include <unistd.h>
#include "AclLiteUtils.h"
#include "AclLiteVideoCapBase.h"

class AclLiteVideoProc {
public:
    AclLiteVideoProc();
    AclLiteVideoProc(uint32_t cameraId, uint32_t width = 1280,
                      uint32_t height = 720, uint32_t fps = 15);
    AclLiteVideoProc(const std::string& videoPath, int32_t deviceId = 0,
                      aclrtContext context = nullptr);
    AclLiteVideoProc(VencConfig& vencConfig,
                     aclrtContext context = nullptr);
    ~AclLiteVideoProc();

    bool IsOpened();
    /**
    * @brief Set the actual value of the corresponding attribute according to the key value
    * @param [in]: key: enumeration type StreamProperty, see the AclLiteVideoCapBase class for details
    * @param [in]: Value: attribute value
    * @return AclLiteError: ACLLITE_ OK: Setting succeeded
    * Other: Setting failed
    */
    AclLiteError Set(StreamProperty key, uint32_t value);
    /**
    * @brief Get the actual value of the corresponding attribute
    * according to the key value
    * @param [in]: key: enumeration type StreamProperty,
    * see the AclLiteVideoCapBase class for details
    * @return Property Value
    */
    uint32_t Get(StreamProperty key);
    /**
    * @brief Acquiring a frame of image data to be processed;
    * According to the constructor, there are three scenarios:
    * 1. One frame data read from the camera;
    * 2. One frame data read from video file/rtsp stream;
    * 3. One frame data processed by dvpp venc.
    * @param [in]: frame: input image data and attributes
    * @return AclLiteError: ACLLITE_ OK: Read successfully
    * Other: ACLLITE_ OK: Reading failed
    */
    AclLiteError Read(ImageData& frame);
    /**
    * @brief Stop reading picture data
    * @param [in]: None
    * @return AclLiteError: ACLLITE_ OK: Close successfully
    * Other: ACLLITE_ OK: Close failed
    */
    AclLiteError Close();

private:
    AclLiteError Open();
 
private:
    AclLiteVideoCapBase* cap_;
};
#endif