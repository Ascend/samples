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


#include <unistd.h>
#include "atlas_utils.h"
#include "atlas_videocap_base.h"



class AtlasVideoCapture {
public:
    AtlasVideoCapture();

    AtlasVideoCapture(uint32_t cameraId, uint32_t width = 1280, 
                      uint32_t height = 720, uint32_t fps = 20);

    AtlasVideoCapture(const string& videoPath, aclrtContext context = nullptr);

    bool IsOpened();

    AtlasError Set(StreamProperty key, uint32_t value);
    uint32_t Get(StreamProperty key);

    AtlasError Read(ImageData& frame);

    AtlasError Close();

private:
    AtlasError Open();
 
private:
    AtlasVideoCapBase* cap;    
};


