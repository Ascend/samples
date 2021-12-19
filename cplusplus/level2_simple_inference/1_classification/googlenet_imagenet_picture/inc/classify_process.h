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

* File sample_process.h
* Description: handle acl resource
*/
#pragma once

#include <iostream>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <dirent.h>
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"

/**
* ClassifyProcess
*/
class ClassifyProcess {
public:
    ClassifyProcess();
    ~ClassifyProcess();

    AclLiteError Init();
    AclLiteError Process(std::vector<std::string>& fileVec, aclrtRunMode RunMode);
    void LabelClassToImage(int classIdx, const std::string& origImagePath);
    void DestroyResource();

private:
    AclLiteError Preprocess(const std::string& imageFile);
    AclLiteError Inference(std::vector<InferenceOutput>& inferOutputs);
    AclLiteError Postprocess(const std::string& origImageFile,
                       std::vector<InferenceOutput>& inferOutputs);

private:
    AclLiteModel model_;
    uint32_t inputDataSize_;
    void* inputData_;
    bool isInited_;
    bool isReleased_;
    aclrtRunMode RunMode_;
};

