/**
* @file gemm_main.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "acl/acl.h"
#include "gemm_runner.h"
#include "common.h"

bool g_isDevice = false;
int deviceId = 0;

bool RunOp(int m, int n, int k, aclFloat16 alpha, aclFloat16 beta)
{
    GemmRunner runner(m, n, k, ACL_FLOAT16, ACL_FLOAT16);
    if (!runner.Init()) {
        return false;
    }
    INFO_LOG("Init op input success");

    runner.SetAlpha(alpha);
    runner.SetBeta(beta);

    if (!runner.PrepareInputs()) {
        return false;
    }
    INFO_LOG("Prepare op input data success");

    INFO_LOG("Matrix A:");
    runner.PrintMatrixA();

    INFO_LOG("Matrix B:");
    runner.PrintMatrixB();

    INFO_LOG("Matrix C:");
    runner.PrintMatrixC();

    // Launch gemm kernel
    if (!runner.RunOp()) {
        return false;
    }

    INFO_LOG("Matrix Output:");
    runner.PrintMatrixC();
    return runner.WriteOutput();
}

void DestoryResource()
{
    bool flag = false;
    if (aclrtResetDevice(deviceId) != ACL_SUCCESS) {
        ERROR_LOG("Reset device %d failed", deviceId);
        flag = true;
    }
    if (aclFinalize() != ACL_SUCCESS) {
        ERROR_LOG("Finalize acl failed");
        flag = true;
    }
    if (flag) {
        ERROR_LOG("Destory resource failed");
    } else {
        INFO_LOG("Destory resource success");
    }
}

bool InitResource()
{
    std::string output = "./result_files";
    if (access(output.c_str(), 0) == -1) {
        int ret = mkdir(output.c_str(), 0700);
        if (ret == 0) {
            INFO_LOG("Make output directory successfully");
        }
        else {
            ERROR_LOG("Make output directory fail");
            return false;
        }
    }

    // acl.json is dump or profiling config file
    if (aclInit("test_data/config/acl.json") != ACL_SUCCESS) {
        ERROR_LOG("Init acl failed");
        return false;
    }

    if (aclrtSetDevice(deviceId) != ACL_SUCCESS) {
        ERROR_LOG("Set device[%d] failed.", deviceId);
        (void)aclFinalize();
        return false;
    }
    INFO_LOG("Set device[%d] success", deviceId);

    aclrtRunMode runMode;
    // runMode is ACL_HOST which represents app is running in host
    // runMode is ACL_DEVICE which represents app is running in device
    if (aclrtGetRunMode(&runMode) != ACL_SUCCESS) {
        ERROR_LOG("Get run mode failed");
        DestoryResource();
        return false;
    }
    g_isDevice = (runMode == ACL_DEVICE);

    if (aclopSetModelDir("op_models") != ACL_SUCCESS) {
        ERROR_LOG("Load single op model failed");
        DestoryResource();
        return false;
    }

    return true;
}

int main()
{
    if (!InitResource()) {
        ERROR_LOG("Init resource failed");
        return FAILED;
    }
    INFO_LOG("Init resource success");

    int m = 16;
    int n = 16;
    int k = 16;
    aclFloat16 alpha = aclFloatToFloat16(2.0);
    aclFloat16 beta =  aclFloatToFloat16(1.0);
    if (!RunOp(m, n, k, alpha, beta)) {
        DestoryResource();
        return FAILED;
    }
    INFO_LOG("Run op success");

    DestoryResource();

    return SUCCESS;
}
