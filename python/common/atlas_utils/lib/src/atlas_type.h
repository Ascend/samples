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
#include <string>

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

enum MemoryType {
    MEMORY_NORMAL = 0,
    MEMORY_HOST,
    MEMORY_DEVICE,
    MEMORY_DVPP,    
    MEMORY_INVALID_TYPE
};

enum CopyDirection {
    TO_DEVICE = 0,
    TO_HOST,
    INVALID_COPY_DIRECT
};

struct DataBuf {
    uint32_t size;
    uint8_t* data;
};

struct ImageData {
    acldvppPixelFormat format;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t alignWidth = 0;
    uint32_t alignHeight = 0;
    uint32_t size = 0;
    uint8_t* data = nullptr;
};

struct FrameData {
    bool isFinished = false;
    uint32_t frameId = 0;
    uint32_t size = 0;
    void* data = nullptr;
};

struct Resolution {
    uint32_t width = 0;
    uint32_t height = 0;
};

struct Rect {
    uint32_t ltX = 0;
    uint32_t ltY = 0;
    uint32_t rbX = 0;
    uint32_t rbY = 0;
};

struct BBox {
    Rect rect;
    uint32_t score = 0;
    string text;
};

struct AtlasMessage {
    int dest;
    int msgId;
    void* data;
};

struct DataInfo {
    void* data;
    uint32_t size;
};

struct InferenceOutput {
    shared_ptr<void> data = nullptr;
    uint32_t size;
};

struct VideoInfo {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
};