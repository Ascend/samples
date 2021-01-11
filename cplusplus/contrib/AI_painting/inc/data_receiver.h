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

* File data_receiver.h
* Description: receive data
*/

#ifndef AIPAINTING_ATLAS200DK_V2_DATA_RECEIVER_H
#define AIPAINTING_ATLAS200DK_V2_DATA_RECEIVER_H


#include <iostream>
#include <string>
#include <dirent.h>
#include <memory>
#include <unistd.h>
#include <vector>
#include <stdint.h>

#include "presenter_channels.h"
#include "utils.h"

class  DataReceiver {
public:
    /**
     * @brief   constructor
     */
     DataReceiver() {}

    /**
     * @brief   destructor
     */
    ~ DataReceiver() = default;

    Result Init();

    Result DoReceiverProcess(void* objectData, void* layoutData);


};

#endif //AIPAINTING_ATLAS200DK_V2_DATA_RECEIVER_H
