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

* File data_receiver.cpp
* Description: receive data
*/

#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <regex>
#include "data_receiver.h"
#include "utils.h"

using namespace std;

Result DataReceiver::Init() {

    string input_file = "./param.conf";
    string presenter_server_ip = "";
    int presenter_server_port = 0;
    // get presenter server ip and port
    Utils::GetPresenterServerParam(input_file, presenter_server_ip, presenter_server_port);
    if (presenter_server_ip == "" || presenter_server_port == 0){
        ERROR_LOG("Get presenter_server ip or port failed, please check param.conf.");
        return FAILED;
    }

    PresenterServerParams register_param;
    register_param.app_id = "AIPainting";
    register_param.app_type = "display";
    register_param.host_ip = presenter_server_ip;
    register_param.port = presenter_server_port;
    PresenterChannels::GetInstance().Init(register_param);
    // create agent channel and register app
    ascend::presenter::Channel *agent_channel = PresenterChannels::GetInstance().GetChannel();
    if (agent_channel == nullptr) {
        ERROR_LOG("Register app failed.");
        return FAILED;
    }

    INFO_LOG("register app success.");

    return SUCCESS;
}

Result DataReceiver::DoReceiverProcess(void* objectData, void* layoutData) {
    // get agent channel
    ascend::presenter::Channel *agent_channel = PresenterChannels::GetInstance().GetChannel();
    if (agent_channel == nullptr) {
        ERROR_LOG("get agent channel to send failed.");
        return FAILED;
    }

    // construct registered request Message and read message
    unique_ptr<google::protobuf::Message> data_rec;
    ascend::presenter::PresenterErrorCode agent_ret = agent_channel->ReceiveMessage(data_rec);
    if (agent_ret == ascend::presenter::PresenterErrorCode::kNone) {

        PaintingMessage::DataPackage *model_input_data_package = (PaintingMessage::DataPackage * )(data_rec.get());

        if(model_input_data_package == nullptr){
            INFO_LOG("Receive data is null");
            return FAILED;
        }

        PaintingMessage::ModelInputData object_input = model_input_data_package->objectdata();
        std::string object_input_name = object_input.name();
        std::string object_input_data = object_input.data();
        INFO_LOG("object_input_name: %s", object_input_name.c_str());
        INFO_LOG("object_input_data size: %ld", object_input_data.size());


        PaintingMessage::ModelInputData layout_input = model_input_data_package->layoutdata();
        std::string layout_input_name = layout_input.name();
        std::string layout_input_data = layout_input.data();
        INFO_LOG("layout_input_name: %s", layout_input_name.c_str());
        INFO_LOG("layout_input_data size: %ld", layout_input_data.size());

        aclrtRunMode runMode;
        aclrtGetRunMode (& runMode);
        aclrtMemcpyKind policy = (runMode == ACL_HOST) ?
        ACL_MEMCPY_HOST_TO_DEVICE : ACL_MEMCPY_DEVICE_TO_DEVICE;

        aclError ret = aclrtMemcpy(objectData, object_input_data.size(),
        object_input_data.c_str(), object_input_data.size(), policy);
        ret = aclrtMemcpy(layoutData, layout_input_data.size(),
        layout_input_data.c_str(), layout_input_data.size(), policy);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("Copy resized image data to device failed.");
            return FAILED;
        }

        return SUCCESS;

    } else {
        return FAILED;
    }
}