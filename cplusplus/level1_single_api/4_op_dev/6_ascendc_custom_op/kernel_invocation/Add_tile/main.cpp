/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */
#include "data_utils.h"
#ifndef __CCE_KT_TEST__
#include "acl/acl.h"
extern void add_custom_do(uint32_t coreDim, void* l2ctrl, void* stream, uint8_t* x, uint8_t* y, uint8_t* z,
    uint8_t* workspace, uint8_t* tiling);
#else
#include "tikicpulib.h"
extern "C" __global__ __aicore__ void add_custom(GM_ADDR x, GM_ADDR y, GM_ADDR z, GM_ADDR workspace, GM_ADDR tiling);
#endif

int32_t main(int32_t argc, char* argv[])
{
    size_t tilingSize = 3 * sizeof(uint32_t);
    size_t usrWorkspaceSize = 4096;
    size_t sysWorkspaceSize = 16 * 1024 * 1024;
#ifdef __CCE_KT_TEST__
    uint8_t* usrWorkSpace = (uint8_t*)tik2::GmAlloc(usrWorkspaceSize);
    uint8_t* tiling = (uint8_t*)tik2::GmAlloc(tilingSize);
    ReadFile("./input/tiling.bin", tilingSize, tiling, tilingSize);

    uint32_t blockDim = (*(const uint32_t *)(tiling));
    size_t inputByteSize = blockDim * 2048 * sizeof(uint16_t);  // uint16_t represent half
    size_t outputByteSize = blockDim * 2048 * sizeof(uint16_t);  // uint16_t represent half

    uint8_t* x = (uint8_t*)tik2::GmAlloc(inputByteSize);
    uint8_t* y = (uint8_t*)tik2::GmAlloc(inputByteSize);
    uint8_t* z = (uint8_t*)tik2::GmAlloc(outputByteSize);

    ReadFile("./input/input_x.bin", inputByteSize, x, inputByteSize);
    ReadFile("./input/input_y.bin", inputByteSize, y, inputByteSize);

    ICPU_RUN_KF(add_custom, blockDim, x, y, z, usrWorkSpace, tiling); // use this macro for cpu debug

    WriteFile("./output/output_z.bin", z, outputByteSize);

    tik2::GmFree((void *)x);
    tik2::GmFree((void *)y);
    tik2::GmFree((void *)z);
    tik2::GmFree((void *)usrWorkSpace);
    tik2::GmFree((void *)tiling);
#else
    CHECK_ACL(aclInit(nullptr));
    aclrtContext context;
    int32_t deviceId = 0;
    CHECK_ACL(aclrtSetDevice(deviceId));
    CHECK_ACL(aclrtCreateContext(&context, deviceId));
    aclrtStream stream = nullptr;
    CHECK_ACL(aclrtCreateStream(&stream));

    uint8_t *xHost, *yHost, *zHost, *tilingHost, *workspaceHost;
    uint8_t *xDevice, *yDevice, *zDevice, *tilingDevice, *workspaceDevice;

    CHECK_ACL(aclrtMallocHost((void**)(&tilingHost), tilingSize));
    ReadFile("./input/tiling.bin", tilingSize, tilingHost, tilingSize);

    CHECK_ACL(aclrtMallocHost((void**)(&workspaceHost), tilingSize));

    uint32_t blockDim = (*(const uint32_t *)(tilingHost));
    size_t inputByteSize = blockDim * 2048 * sizeof(uint16_t);  // uint16_t represent half
    size_t outputByteSize = blockDim * 2048 * sizeof(uint16_t);  // uint16_t represent half
    size_t workspaceByteSize = sysWorkspaceSize + usrWorkspaceSize;

    CHECK_ACL(aclrtMallocHost((void**)(&xHost), inputByteSize));
    CHECK_ACL(aclrtMallocHost((void**)(&yHost), inputByteSize));
    CHECK_ACL(aclrtMallocHost((void**)(&zHost), outputByteSize));
    CHECK_ACL(aclrtMallocHost((void**)(&workspaceHost), workspaceByteSize));
    CHECK_ACL(aclrtMalloc((void**)&xDevice, inputByteSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&yDevice, outputByteSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&zDevice, outputByteSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&tilingDevice, tilingSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&workspaceDevice, workspaceByteSize, ACL_MEM_MALLOC_HUGE_FIRST));

    ReadFile("./input/input_x.bin", inputByteSize, xHost, inputByteSize);
    ReadFile("./input/input_y.bin", inputByteSize, yHost, inputByteSize);

    CHECK_ACL(aclrtMemcpy(xDevice, inputByteSize, xHost, inputByteSize, ACL_MEMCPY_HOST_TO_DEVICE));
    CHECK_ACL(aclrtMemcpy(yDevice, inputByteSize, yHost, inputByteSize, ACL_MEMCPY_HOST_TO_DEVICE));
    CHECK_ACL(aclrtMemcpy(tilingDevice, tilingSize, tilingHost, tilingSize, ACL_MEMCPY_HOST_TO_DEVICE));

    add_custom_do(blockDim, nullptr, stream, xDevice, yDevice, zDevice, workspaceDevice, tilingDevice);
    CHECK_ACL(aclrtSynchronizeStream(stream));

    CHECK_ACL(aclrtMemcpy(zHost, outputByteSize, zDevice, outputByteSize, ACL_MEMCPY_DEVICE_TO_HOST));
    WriteFile("./output/output_z.bin", zHost, outputByteSize);

    CHECK_ACL(aclrtFree(xDevice));
    CHECK_ACL(aclrtFree(yDevice));
    CHECK_ACL(aclrtFree(zDevice));
    CHECK_ACL(aclrtFree(workspaceDevice));
    CHECK_ACL(aclrtFree(tilingDevice));
    CHECK_ACL(aclrtFreeHost(xHost));
    CHECK_ACL(aclrtFreeHost(yHost));
    CHECK_ACL(aclrtFreeHost(zHost));
    CHECK_ACL(aclrtFreeHost(workspaceHost));
    CHECK_ACL(aclrtFreeHost(tilingHost));

    CHECK_ACL(aclrtDestroyStream(stream));
    CHECK_ACL(aclrtDestroyContext(context));
    CHECK_ACL(aclrtResetDevice(deviceId));
    CHECK_ACL(aclFinalize());
#endif
    return 0;
}
