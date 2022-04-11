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

* File utils.h
* Description: handle file operations
*/
#pragma once

#include "AclLiteError.h"
#include "AclLiteType.h"

#define RTSP_TRANS_UDP ((uint32_t)0)
#define RTSP_TRANS_TCP ((uint32_t)1)

enum StreamProperty {   
    FRAME_WIDTH = 1,
    FRAME_HEIGHT = 2,
    VIDEO_FPS = 3,
    OUTPUT_IMAGE_FORMAT = 4,
    RTSP_TRANSPORT = 5,
    STREAM_FORMAT = 6
};

class AclLiteVideoCapBase {
public:
    AclLiteVideoCapBase(){}
    virtual ~AclLiteVideoCapBase(){}; 
    virtual bool IsOpened() = 0;
    virtual AclLiteError Set(StreamProperty key, uint32_t value) { return ACLLITE_OK; }
    virtual uint32_t Get(StreamProperty key) { return 0; }
    virtual AclLiteError Read(ImageData& frame) = 0;
    virtual AclLiteError Close() = 0;
    virtual AclLiteError Open() = 0;    
};