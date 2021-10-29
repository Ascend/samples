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
#include "atlas_error.h"

using namespace std;

class AclDevice {
public:
    AclDevice();
    AclDevice(int32_t devId, const string& aclConfigPath);   
    ~AclDevice();

    AtlasError Init();
    void Release();

public:
    int32_t deviceId;
    string aclConfig;
    aclrtRunMode runMode;

private:   
    bool isReleased_;
};