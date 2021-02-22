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

#include <cstdio>
#include "acl_engine.h"

int main(int argc, char *argv[]) {
    AclEngine acl_engine;
    if (!acl_engine.ParseParams(argc, argv)) {
        printf("parse params faild.");
        acl_engine.Usage(arge[0]);
        return 1;
    }
    printf("parse completed...\n");
    
    if (acl_engine.IsHelp()) {
        acl_engine.Usage(argv[0]);
        return 0;
    }

    if (!acl_engine.Init()) {
        puts("acl engine init failed.");
        return 1;
    }
    printf("engine init completed...\n");

    if (acl_engine.Inference()) {
        puts("acl inference failed");
        return 1;
    }
    printf("inference completed...\n");
    return 0;
}