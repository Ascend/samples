/**
* @file sample_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "sample_process.h"
#include <getopt.h>
#include "dvpp_process.h"

using namespace std;

namespace
{
    string g_inFileName = "infile";
    string g_outFileName = "outfile";
    int32_t g_inFormat = 0;
    int32_t g_outFormat = 0;
    int32_t g_inWidth = 0;
    int32_t g_inHeight = 0;
    int32_t g_cropLeft = 0;
    int32_t g_cropRight = 0;
    int32_t g_cropTop = 0;
    int32_t g_cropBottom = 0;
    int32_t g_outWidth = 0;
    int32_t g_outHeight = 0;
    int32_t g_pasteLeft = 0;
    int32_t g_pasteRight = 0;
    int32_t g_pasteTop = 0;
    int32_t g_pasteBottom = 0;
}

SampleProcess::SampleProcess() : deviceId_(0), context_(nullptr), stream_(nullptr)
{
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}

Result SampleProcess::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl init failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("acl init success");

    // set device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl set device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("set device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create context failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create stream failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create stream success");

    // get run mode
    // runMode is ACL_HOST which represents app is running in host
    // runMode is ACL_DEVICE which represents app is running in device
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    bool isDivece = (runMode == ACL_DEVICE);
    RunStatus::SetDeviceStatus(isDivece);
    INFO_LOG("get run mode success");

    return SUCCESS;
}

Result SampleProcess::GetInputOptionSecond(int32_t c)
{
     switch (c) {
        case 'L':
            g_pasteLeft = atoi(optarg);
            break;
        case 'R':
            g_pasteRight = atoi(optarg);
            break;
        case 'T':
            g_pasteTop = atoi(optarg);
            break;
        case 'B':
            g_pasteBottom = atoi(optarg);
            break;
        case 'W':
            g_outWidth = atoi(optarg);
            break;
        case 'H':
            g_outHeight = atoi(optarg);
            break;
        case 'o':
            g_outFormat = atoi(optarg);
            break;
        case 'O':
            g_outFileName = optarg;
            break;
        default:
            ERROR_LOG("invalid parameter");
            return FAILED;
    }
    return SUCCESS;
}

Result SampleProcess::GetInputOption(int argc, char **argv)
{
    int32_t paramNum = 16; // input parameter number is 16
    while (paramNum--) {
        int32_t optionIndex = 0;
        struct option longOptions[] =
        {
            {"inWidth", 1, 0, 'w'},
            {"inHeight", 1, 0, 'h'},
            {"cLeftOffset", 1, 0, 'l'},
            {"cRightOffset", 1, 0, 'r'},
            {"cTopOffset", 1, 0, 't'},
            {"cBottomOffset", 1, 0, 'b'},
            {"inFormat", 1, 0, 'i'},
            {"outFormat", 1, 0, 'o'},
            {"inImgName", 1, 0, 'I'},
            {"outImgName", 1, 0, 'O'},
            {"outWidth", 1, 0, 'W'},
            {"outHeight", 1, 0, 'H'},
            {"pLeftOffset", 1, 0, 'L'},
            {"pRightOffset", 1, 0, 'R'},
            {"pTopOffset", 1, 0, 'T'},
            {"pBottomOffset", 1, 0, 'B'},
        };
        int32_t c = getopt_long(argc, argv, "w:h:l:r:t:b:i:o:I:O:W:H:L:R:T:B",
            longOptions, &optionIndex);
        if (c == -1) {
            return FAILED;
        }
        switch (c) {
            case 'w':
                g_inWidth = atoi(optarg);
                break;
            case 'h':
                g_inHeight = atoi(optarg);
                break;
            case 'l':
                g_cropLeft = atoi(optarg);
                break;
            case 'r':
                g_cropRight = atoi(optarg);
                break;
            case 't':
                g_cropTop = atoi(optarg);
                break;
            case 'b':
                g_cropBottom = atoi(optarg);
                break;
             case 'i':
                g_inFormat = atoi(optarg);
                break;
             case 'I':
                g_inFileName = optarg;
                break;
            default:
                Result ret = GetInputOptionSecond(c);
                if (ret != SUCCESS) {
                    return FAILED;
                }
                break;
        }
    }

    if (paramNum > 0) {
        ERROR_LOG("paramNum must be 16");
        return FAILED;
    }
    return SUCCESS;
}

Result SampleProcess::VpcProcess()
{
    // dvpp init
    DvppProcess dvppProcess(stream_);
    Result ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    // set input and output parameter
    dvppProcess.SetDvppInputPara(g_inWidth, g_inHeight, g_inFormat, g_inFileName);
    dvppProcess.SetDvppCropPara(g_cropLeft, g_cropRight, g_cropTop, g_cropBottom);
    dvppProcess.SetDvppOutputPara(g_outWidth, g_outHeight, g_outFormat, g_outFileName);
    dvppProcess.SetDvppPastePara(g_pasteLeft, g_pasteRight, g_pasteTop, g_pasteBottom);

    ret = dvppProcess.CheckParameter();
    if (ret != SUCCESS) {
        ERROR_LOG("check dvpp process parameter failed");
        return FAILED;
    }

    ret = dvppProcess.Process();
    if (ret != SUCCESS) {
        ERROR_LOG("dvpp process failed");
        return FAILED;
    }

    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy stream failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
    }
    INFO_LOG("end to reset device %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed, errorCode = %d", static_cast<int32_t>(ret));
    }
    INFO_LOG("end to finalize acl");
}
