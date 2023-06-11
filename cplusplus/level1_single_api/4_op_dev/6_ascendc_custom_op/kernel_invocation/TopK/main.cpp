/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */
#include "data_utils.h"
#ifndef __CCE_KT_TEST__
#include "acl/acl.h"
extern void topk_custom_do(uint32_t coreDim, void* l2ctrl, void* stream,
    uint8_t* x, uint8_t* y, uint8_t* workspace, uint8_t* sync, uint8_t* param);
#else
#include "tikicpulib.h"
extern "C" __global__ __aicore__ void topk_custom(GM_ADDR x, GM_ADDR y,
    GM_ADDR workspace, GM_ADDR sync, GM_ADDR param);
#endif

int32_t main(int32_t argc, char* argv[])
{
    size_t inputSize = 2097152;
    size_t xSize = inputSize * sizeof(uint16_t);  // uint16_t represent half
    size_t ySize = inputSize * sizeof(uint16_t);  // uint16_t represent half
    size_t workspaceSize = inputSize * 8 * sizeof(uint16_t);  // uint16_t represent half
    size_t syncSize = 128 * sizeof(int32_t);
    size_t paramSize = 8 * sizeof(int32_t);
    uint32_t blockDim = 4;

#ifdef __CCE_KT_TEST__
    uint8_t *param1 = (uint8_t *)tik2::GmAlloc(xSize);
    uint8_t *param2 = (uint8_t *)tik2::GmAlloc(ySize);
    uint8_t *param3 = (uint8_t *)tik2::GmAlloc(workspaceSize);
    uint8_t *param4 = (uint8_t *)tik2::GmAlloc(syncSize);
    uint8_t *param5 = (uint8_t *)tik2::GmAlloc(paramSize);

    ReadFile("./input/input_x.bin", xSize, param1, xSize);
    ReadFile("./input/workspace.bin", workspaceSize, param3, workspaceSize);
    ReadFile("./input/sync.bin", syncSize, param4, syncSize);
    ReadFile("./input/param.bin", paramSize, param5, paramSize);

    ICPU_RUN_KF(topk_custom, blockDim, param1, param2, param3, param4, param5);

    WriteFile("./output/cpu_output.bin", param2, ySize);

    tik2::GmFree((void *)param1);
    tik2::GmFree((void *)param2);
    tik2::GmFree((void *)param3);
    tik2::GmFree((void *)param4);
    tik2::GmFree((void *)param5);
#else
    CHECK_ACL(aclInit(nullptr));
    aclrtContext context;
    int32_t deviceId = 0;
    CHECK_ACL(aclrtSetDevice(deviceId));
    CHECK_ACL(aclrtCreateContext(&context, deviceId));
    aclrtStream stream = nullptr;
    CHECK_ACL(aclrtCreateStream(&stream));

    uint8_t *param1Host;
    uint8_t *param1Device;
    CHECK_ACL(aclrtMallocHost((void**)(&param1Host), xSize));
    CHECK_ACL(aclrtMalloc((void**)&param1Device, xSize, ACL_MEM_MALLOC_HUGE_FIRST));
    ReadFile("./input/input_x.bin", xSize, param1Host, xSize);
    CHECK_ACL(aclrtMemcpy(param1Device, xSize, param1Host, xSize, ACL_MEMCPY_HOST_TO_DEVICE));

    uint8_t *param2Host;
    uint8_t *param2Device;
    CHECK_ACL(aclrtMallocHost((void**)(&param2Host), ySize));
    CHECK_ACL(aclrtMalloc((void**)&param2Device, ySize, ACL_MEM_MALLOC_HUGE_FIRST));

    uint8_t *param3Host;
    uint8_t *param3Device;
    CHECK_ACL(aclrtMallocHost((void**)(&param3Host), workspaceSize));
    CHECK_ACL(aclrtMalloc((void**)&param3Device, workspaceSize, ACL_MEM_MALLOC_HUGE_FIRST));
    ReadFile("./input/workspace.bin", workspaceSize, param3Host, workspaceSize);
    CHECK_ACL(aclrtMemcpy(param3Device, workspaceSize, param3Host, workspaceSize, ACL_MEMCPY_HOST_TO_DEVICE));

    uint8_t *param4Host;
    uint8_t *param4Device;
    CHECK_ACL(aclrtMallocHost((void**)(&param4Host), syncSize));
    CHECK_ACL(aclrtMalloc((void**)&param4Device, syncSize, ACL_MEM_MALLOC_HUGE_FIRST));
    ReadFile("./input/sync.bin", syncSize, param4Host, syncSize);
    CHECK_ACL(aclrtMemcpy(param4Device, syncSize, param4Host, syncSize, ACL_MEMCPY_HOST_TO_DEVICE));

    uint8_t *param5Host;
    uint8_t *param5Device;
    CHECK_ACL(aclrtMallocHost((void**)(&param5Host), paramSize));
    CHECK_ACL(aclrtMalloc((void**)&param5Device, paramSize, ACL_MEM_MALLOC_HUGE_FIRST));
    ReadFile("./input/param.bin", paramSize, param5Host, paramSize);
    CHECK_ACL(aclrtMemcpy(param5Device, paramSize, param5Host, paramSize, ACL_MEMCPY_HOST_TO_DEVICE));

    topk_custom_do(blockDim, nullptr, stream, param1Device, param2Device, param3Device, param4Device, param5Device);
    CHECK_ACL(aclrtSynchronizeStream(stream));

    CHECK_ACL(aclrtMemcpy(param2Host, ySize, param2Device, ySize, ACL_MEMCPY_DEVICE_TO_HOST));
    WriteFile("./output/npu_output.bin", param2Host, ySize);
    CHECK_ACL(aclrtFree(param2Device));
    CHECK_ACL(aclrtFreeHost(param2Host));

    CHECK_ACL(aclrtDestroyStream(stream));
    CHECK_ACL(aclrtDestroyContext(context));
    CHECK_ACL(aclrtResetDevice(deviceId));
    CHECK_ACL(aclFinalize());
#endif
    return 0;
}
