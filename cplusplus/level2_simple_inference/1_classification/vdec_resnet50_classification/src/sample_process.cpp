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
#include <string>
#include <sstream>
#include "utils.h"

using namespace std;

namespace {
    const std::string filePath= "../data/vdec_h265_1frame_rabbit_1280x720.h265";
    const char* omModelPath = "../model/resnet50_aipp.om";
    const std::string modelOutputBinfileName = "./result/model_output_";
    const std::string dvppOutputfileName = "./result/dvpp_output_";
    bool runFlag = true;
}

SampleProcess::SampleProcess() : deviceId_(0), context_(nullptr), stream_(nullptr), thread_(),
    outFolder_((char*)"outdir/"), picDesc_({}), enType_(H265_MAIN_LEVEL), format_(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
{
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}
void *ThreadFunc(void *arg)
{
    // Notice: create context for this thread
    int deviceId = 0;
    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, deviceId);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtCreateContext failed, errorCode = %d", static_cast<int32_t>(ret));
        return ((void*)(-1));
    }

    INFO_LOG("thread start ");
    while (runFlag) {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
    }

    ret = aclrtDestroyContext(context);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtDestroyContext failed, errorCode = %d", static_cast<int32_t>(ret));
    }

    return (void*)0;
}

Result SampleProcess::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("acl init success");

    // set device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl set device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("set device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
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
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    bool isDivece = (runMode == ACL_DEVICE);
    RunStatus::SetDeviceStatus(isDivece);
    INFO_LOG("get run mode success");

    return SUCCESS;
}

Result SampleProcess::DoVdecProcess()
{
    // create threadId
    thread_ = std::thread(ThreadFunc, nullptr);
    std::ostringstream oss;
    oss << thread_.get_id();
    uint64_t tid = std::stoull(oss.str());
    INFO_LOG("create thread successfully, threadId = %lu", tid);

    Result ret = Utils::CheckAndCreateFolder(outFolder_);
    if (ret != SUCCESS) {
        ERROR_LOG("mkdir out folder error.");
        return FAILED;
    }

    // dvpp init
    VdecProcess vdecProcess;
    ret = vdecProcess.InitResource(tid, enType_, format_);
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        vdecProcess.DestroyResource();
        return FAILED;
    }

    const int inputWidth = 1280;
    const int inputHeight = 720;
    int rest_len = 10;
    picDesc_.width = inputWidth;
    picDesc_.height = inputHeight;

    uint64_t count = 0;
    while (rest_len > 0) {
        void *inBufferDev = nullptr;
        uint32_t inBufferSize = 0;

        // read file to device memory
        if (!Utils::ReadFileToDeviceMem(filePath.c_str(), inBufferDev, inBufferSize)) {
            ERROR_LOG("read file %s to device mem failed.\n", filePath.c_str());
            vdecProcess.DestroyResource();
            return FAILED;
        }
        vdecProcess.SetInput(inBufferDev, inBufferSize, picDesc_.width, picDesc_.height);

        ret = vdecProcess.Process();
        if (ret != SUCCESS) {
            ERROR_LOG("dvpp ProcessVdec failed");
            vdecProcess.DestroyResource();
            return FAILED;
        }
        ++count;
        rest_len = rest_len - 1;
        INFO_LOG("success to execute aclvdecSendFrame, count = %lu", count);
    }
    ret = vdecProcess.SendVdecEos();
    if (ret != SUCCESS) {
        ERROR_LOG("send vdec eos frame failed, errorCode = %d", static_cast<int32_t>(ret));
        vdecProcess.DestroyResource();
        return FAILED;
    }
    INFO_LOG("success to send vdec eos frame");

    vdecProcess.DestroyResource();

    return SUCCESS;
}

Result SampleProcess::DoModelProcess()
{
    // model init
    ModelProcess modelProcess;

    Result ret = Utils::CheckAndCreateFolder("result");
    if (ret != SUCCESS) {
        ERROR_LOG("mkdir out folder error.");
        return FAILED;
    }

    // dvpp init
    DvppProcess dvppProcess(stream_);
    ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    // model resource init
    ret = modelProcess.LoadModel(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModel failed");
        return FAILED;
    }
    ret = modelProcess.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }
    ret = modelProcess.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    int modelInputWidth;
    int modelInputHeight;
    ret = modelProcess.GetModelInputWH(modelInputWidth, modelInputHeight);
    if (ret != SUCCESS) {
        ERROR_LOG("execute GetModelInputWH failed");
        return FAILED;
    }

    std::vector<std::string> fileList = Utils::ReadDir(outFolder_);
    for (size_t frameId = 0; frameId < fileList.size(); frameId++) {
        void *dvppOutputBuffer = nullptr;
        uint32_t dvppOutputSize;

        // read image file to device memory
        std::string fileNameSave = outFolder_ + fileList[frameId];
        if (!Utils::ReadFileToDeviceMem(fileNameSave.c_str(), dvppOutputBuffer, dvppOutputSize)) {
            ERROR_LOG("read file %s to device mem failed.\n", fileNameSave.c_str());
            return FAILED;
        }
        dvppProcess.SetInput(picDesc_.width, picDesc_.height, format_);
        ret = dvppProcess.InitOutputPara(modelInputWidth, modelInputHeight);
        if (ret != SUCCESS) {
            ERROR_LOG("init dvpp output para failed");
            (void)acldvppFree(dvppOutputBuffer);
            return FAILED;
        }

        // dvpp process
        dvppProcess.Process(dvppOutputBuffer, dvppOutputSize);
        dvppProcess.GetOutput(&dvppOutputBuffer, dvppOutputSize);

        // model proces
        ret = modelProcess.CreateInput(dvppOutputBuffer, dvppOutputSize);
        if (ret != SUCCESS) {
            ERROR_LOG("execute CreateInput failed");
            dvppProcess.DestroyOutputPara();
            (void)acldvppFree(dvppOutputBuffer);
            return FAILED;
        }
        ret = modelProcess.Execute();
        if (ret != SUCCESS) {
            ERROR_LOG("execute inference failed");
            dvppProcess.DestroyOutputPara();
            modelProcess.DestroyInput();
            (void)acldvppFree(dvppOutputBuffer);
            return FAILED;
        }
        (void)acldvppFree(dvppOutputBuffer);
        remove(fileNameSave.c_str());

        aclmdlDataset *modelOutput = modelProcess.GetModelOutputData();
        if (modelOutput == nullptr) {
            ERROR_LOG("get model output data failed");
            dvppProcess.DestroyOutputPara();
            modelProcess.DestroyInput();
            return FAILED;
        }
        std::string modelOutputBinfileNameCur = modelOutputBinfileName + std::to_string(frameId);
        ret = Utils::PullModelOutputData(modelOutput, modelOutputBinfileNameCur.c_str());
        if (ret != SUCCESS) {
            ERROR_LOG("pull model output data failed");
            dvppProcess.DestroyOutputPara();
            modelProcess.DestroyInput();
            return FAILED;
        }

        std::string modelOutputTxtfileNameCur = modelOutputBinfileNameCur + ".txt";
        ret = Utils::SaveModelOutputData(modelOutputBinfileNameCur.c_str(), modelOutputTxtfileNameCur.c_str());
        if (ret != SUCCESS) {
            ERROR_LOG("save model output data failed");
            dvppProcess.DestroyOutputPara();
            modelProcess.DestroyInput();
            return FAILED;
        }
        dvppProcess.DestroyOutputPara();
        modelProcess.DestroyInput();
    }

    dvppProcess.DestroyResource();
    Utils::RemoveDir(outFolder_);
    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    runFlag = false;
    if (thread_.joinable()) {
        thread_.join();
    }

    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
    }
    INFO_LOG("end to reset device %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed, errorCode = %d", static_cast<int32_t>(ret));
    }
    INFO_LOG("end to finalize acl");
}
