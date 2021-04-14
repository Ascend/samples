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
* Description: AIPainting sample main func
*/

#include <cstdint>
#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include "painting_process.h"
#include "utils.h"

using namespace std;

#define ModelInputNums 2

const char* omModelPath = "../model/AIPainting_v2.om";

uint32_t ModelInputSize[ModelInputNums][4] = {{9, 1, 1, 1},
                                              {1, 256, 256, 17}};
bool g_isDevice = false;

int main(int argc, char *argv[]) {


    PaintingProcess painting(omModelPath, ModelInputSize);

    Result ret = painting.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("AIPainting init resource failed");
        return FAILED;
    }

    while(1)
    {

        Result ret = painting.Preprocess();
        if (ret != SUCCESS) {
            INFO_LOG("Continue receiving...");
        }
        else{

            aclmdlDataset* inferenceOutput = nullptr;
            ret = painting.Inference(inferenceOutput);
            if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
                ERROR_LOG("Inference model inference output data failed");
                return FAILED;
            }

            ret = painting.Postprocess(inferenceOutput);
            if (ret != SUCCESS) {
                ERROR_LOG("Process model inference output data failed");
                return FAILED;
            }
            INFO_LOG("Inference once success");
        }
    }

    return SUCCESS;
}
