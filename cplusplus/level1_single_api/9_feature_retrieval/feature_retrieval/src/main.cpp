/**
* @file main.cpp
*
* Copyright (C) 2021. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <iostream>

#include "acl/acl.h"
#include "common.h"
#include "fv_search.h"

int main()
{
    RunFVSearch search;

    // 0. System init
    bool ret = search.Initialize(SEARCH_1_N);
    if (!ret) {
        ERROR_LOG("Init resource failed!!!");
        return -1;
    }
    INFO_LOG("Init resource success!!!");

    // 1. add feature into th library
    ret = search.Add();
    if (!ret) {
        ERROR_LOG("Add feature into the library failed!!!");
        return -1;
    }
    INFO_LOG("Add feature into the library success!!!");

    // 2.search from the library
    ret = search.Search();
    if (!ret) {
        ERROR_LOG("Search features from the library failed!!!");
        return -1;
    }
    INFO_LOG("Search features from the library success!!!");

    // 3. system finalize
    search.Finalize();
    INFO_LOG("Finalize success!!!");

    return SUCCESS;
}