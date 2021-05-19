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

#include "TextRecongnize.h"
#include "atlasutil/atlas_videocapture.h"
#include "atlasutil/atlas_error.h"
#include "atlasutil/acl_device.h"

using namespace std;

int main(int argc, char *argv[]) {
    //init acl resource
    AclDevice aclDev;
    AtlasError ret = aclDev.Init();
    if (ret) {
        ATLAS_LOG_ERROR("Init resource failed, error %d", ret);
        return ATLAS_ERROR;
    }
    aclrtRunMode RunMode = aclDev.GetRunMode();   
    
    //init text recongnize
    TextRecongnize Recongnize;
    ret = Recongnize.Init();
    if (ret) {
        ATLAS_LOG_ERROR("Init resource failed, error %d", ret);
        return ATLAS_ERROR;
    }
    
    //open camera. If CAMERA0 is accessiable,open CAMERA0, otherwise open CAMERA1
    AtlasVideoCapture cap = AtlasVideoCapture("../data/data.mp4");
    if(!cap.IsOpened()) {
        ATLAS_LOG_ERROR("Open camera failed");
        return ATLAS_ERROR;
    }
    //read frame from camera and inference
    while(true) {
        ImageData image;
        ret = cap.Read(image);
        if (ret) {
            ATLAS_LOG_ERROR("Read image failed, error %d", ret);
            break;
        }
        
        ret = Recongnize.Process(image, RunMode);
        if (ret) {
            ATLAS_LOG_ERROR("Inference image failed, error %d",  ret);
            break;
        }

    }
    cap.Close();
    ATLAS_LOG_INFO("Execute sample success");
    //release text recongnize
    Recongnize.DestroyResource();
    aclDev.Release();
    return ATLAS_OK;
}
