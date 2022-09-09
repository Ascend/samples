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

* File main.cpp
* Description: dvpp sample main func
*/

#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include <sys/time.h>

#include "object_detect.h"
#include "acllite/AclLiteVideoProc.h"
#include "acllite/AclLiteError.h"
#include "acllite/AclLiteResource.h"

using namespace std;

int main(int argc, char *argv[]) {
    //init acl resource
    AclLiteResource aclDev;
    AclLiteError ret = aclDev.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    //init object detect inference
    ObjectDetect detect;
    ret = detect.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    //open camera. If CAMERA0 is accessiable,open CAMERA0, otherwise open CAMERA1
    AclLiteVideoProc cap = AclLiteVideoProc();
    if (!cap.IsOpened()) {
        ACLLITE_LOG_ERROR("Open camera failed");
        return ACLLITE_ERROR;
    }
    //read frame from camera and inference
    while (true) {
        ImageData image;
        ret = cap.Read(image);
        if (ret) {
            ACLLITE_LOG_ERROR("Read image failed, error %d", ret);
            return ACLLITE_ERROR;
        }
        
        ret = detect.Process(image);
        if (ret) {
            ACLLITE_LOG_ERROR("Inference image failed, error %d",  ret);
            return ACLLITE_ERROR;
        }
    }

    ACLLITE_LOG_INFO("Execute sample success");
    //release object detece inference resource
    detect.DestroyResource();

    return ACLLITE_OK;
}
