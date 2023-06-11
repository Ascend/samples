/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */
#include "data_utils.h"
#ifndef __CCE_KT_TEST__
#include "acl/acl.h"
extern void matmul_custom_do(uint32_t coreDim, void* l2ctrl, void* stream,
    uint8_t *param1, uint8_t *param2, uint8_t *param3);
#else
#include "tikicpulib.h"
extern "C" void matmul_custom(uint8_t *param1, uint8_t *param2, uint8_t *param3);
#endif

int32_t main(int32_t argc, char* argv[])
{
    size_t param1FileSize = 1024 * sizeof(uint16_t);  // uint16_t represent half
    size_t param2FileSize = 1024 * sizeof(uint16_t);  // uint16_t represent half
    size_t param3FileSize = 1024 * sizeof(float);
    uint32_t blockDim = 1;

#ifdef __CCE_KT_TEST__
    uint8_t *param1 = (uint8_t *)tik2::GmAlloc(param1FileSize);
    uint8_t *param2 = (uint8_t *)tik2::GmAlloc(param2FileSize);
    uint8_t *param3 = (uint8_t *)tik2::GmAlloc(param3FileSize);

    ReadFile("./input/x1_gm.bin", param1FileSize, param1, param1FileSize);
    ReadFile("./input/x2_gm.bin", param2FileSize, param2, param2FileSize);

    ICPU_RUN_KF(matmul_custom, blockDim, param1, param2, param3);

    WriteFile("./output/output.bin", param3, param3FileSize);

    tik2::GmFree((void *)param1);
    tik2::GmFree((void *)param2);
    tik2::GmFree((void *)param3);
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
    CHECK_ACL(aclrtMallocHost((void**)(&param1Host), param1FileSize));
    CHECK_ACL(aclrtMalloc((void**)&param1Device, param1FileSize, ACL_MEM_MALLOC_HUGE_FIRST));
    ReadFile("./input/x1_gm.bin", param1FileSize, param1Host, param1FileSize);
    CHECK_ACL(aclrtMemcpy(param1Device, param1FileSize, param1Host, param1FileSize, ACL_MEMCPY_HOST_TO_DEVICE));

    uint8_t *param2Host;
    uint8_t *param2Device;
    CHECK_ACL(aclrtMallocHost((void**)(&param2Host), param2FileSize));
    CHECK_ACL(aclrtMalloc((void**)&param2Device, param2FileSize, ACL_MEM_MALLOC_HUGE_FIRST));
    ReadFile("./input/x2_gm.bin", param2FileSize, param2Host, param2FileSize);
    CHECK_ACL(aclrtMemcpy(param2Device, param2FileSize, param2Host, param2FileSize, ACL_MEMCPY_HOST_TO_DEVICE));

    uint8_t *param3Host;
    uint8_t *param3Device;
    CHECK_ACL(aclrtMallocHost((void**)(&param3Host), param3FileSize));
    CHECK_ACL(aclrtMalloc((void**)&param3Device, param3FileSize, ACL_MEM_MALLOC_HUGE_FIRST));

    matmul_custom_do(blockDim, nullptr, stream, param1Device, param2Device, param3Device);
    CHECK_ACL(aclrtSynchronizeStream(stream));

    CHECK_ACL(aclrtMemcpy(param3Host, param3FileSize, param3Device, param3FileSize, ACL_MEMCPY_DEVICE_TO_HOST));
    WriteFile("./output/output.bin", param3Host, param3FileSize);
    CHECK_ACL(aclrtFree(param3Device));
    CHECK_ACL(aclrtFreeHost(param3Host));

    CHECK_ACL(aclrtDestroyStream(stream));
    CHECK_ACL(aclrtDestroyContext(context));
    CHECK_ACL(aclrtResetDevice(deviceId));
    CHECK_ACL(aclFinalize());
#endif
    return 0;
}