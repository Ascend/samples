/**
* @file main.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "op_test.h"
#include "acl/acl.h"
#include "common.h"

int main(int argc, char *argv[])
{
    int ret =  OpTest::UnitTest::GetInstance().Run();
    if (aclrtDestroyStream(g_stream) != ACL_SUCCESS) {
        ERROR_LOG("aclrtDestroyStream failed.");
    }
    if (aclFinalize() != ACL_SUCCESS) {
        ERROR_LOG("aclFinalize failed.");
    }
    return ret;
}
