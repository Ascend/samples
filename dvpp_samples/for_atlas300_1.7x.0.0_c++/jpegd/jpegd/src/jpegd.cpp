#include <iostream>
#include "jpegd.h"
#include "acl/acl.h"
#include <fstream>
#include <cstring>
#include <sys/stat.h>
#include "acl/ops/acl_dvpp.h"

using namespace std;

void* GetDeviceBufferOfPicture(const PicDesc &picDesc, uint32_t &devPicBufferSize)
{
    if (picDesc.picName.empty()) {
        ERROR_LOG("picture file name is empty");
        return nullptr;
    }

    FILE *fp = fopen(picDesc.picName.c_str(), "rb");
    if (fp == nullptr) {
        ERROR_LOG("open file %s failed", picDesc.picName.c_str());
        return nullptr;
    }

    fseek(fp, 0, SEEK_END);
    uint32_t fileLen = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    uint32_t inputHostBuffSize = fileLen;
    void *inputHostBuff = nullptr;

    aclError aclRet = aclrtMallocHost(&inputHostBuff, inputHostBuffSize);
    if (aclRet !=  ACL_ERROR_NONE) {
        ERROR_LOG("malloc host data buffer failed, aclRet is %d", aclRet);
        return nullptr;
    }
    size_t readSize = fread(inputHostBuff, sizeof(char), inputHostBuffSize, fp);
    if (readSize < inputHostBuffSize) {
        ERROR_LOG("need read file %s %u bytes, but only %zu readed",
            picDesc.picName.c_str(), inputHostBuffSize, readSize);
        (void)acldvppFree(inputHostBuff);
        return nullptr;
    }

    void *inBufferDev = nullptr;
    aclRet = acldvppMalloc(&inBufferDev, inputHostBuffSize);
    if (aclRet !=  ACL_ERROR_NONE) {
        (void)aclrtFreeHost(inputHostBuff);
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return nullptr;
    }

    aclError ret = aclrtMemcpy(inBufferDev, inputHostBuffSize, inputHostBuff, inputHostBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("memcpy failed. Input host buffer size is %u",
             inputHostBuffSize);
        acldvppFree(inBufferDev);
        (void)aclrtFreeHost(inputHostBuff);
        return nullptr;
    }

    (void)aclrtFreeHost(inputHostBuff);
    devPicBufferSize = inputHostBuffSize;
    return inBufferDev;
}

void SetInput(void *inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight)
{
    inDevBuffer_ = inDevBuffer;
    inDevBufferSize_ = inDevBufferSize;
    inputWidth_ = inputWidth;
    inputHeight_ = inputHeight;
}

void GetOutput(void **outputBuffer, int &outputSize)
{
    *outputBuffer = decodeOutDevBuffer_;
    outputSize = decodeDataSize_;
    decodeOutDevBuffer_= nullptr;
    decodeDataSize_ = 0;
}

Result SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize)
{
    void* hostPtr = nullptr;
    aclError aclRet = aclrtMallocHost(&hostPtr, dataSize);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc host data buffer failed, aclRet is %d", aclRet);
        return FAILED;
    }

    aclRet = aclrtMemcpy(hostPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("dvpp output memcpy to host failed, aclRet is %d", aclRet);
        (void)aclrtFreeHost(hostPtr);
        return FAILED;
    }

    FILE *outFileFp = fopen(fileName, "wb+");
    if (nullptr == outFileFp) {
        ERROR_LOG("fopen out file %s failed.", fileName);
        (void)aclrtFreeHost(hostPtr);
        return FAILED;
    }

    size_t writeSize = fwrite(hostPtr, sizeof(char), dataSize, outFileFp);
    if (writeSize != dataSize) {
        ERROR_LOG("need write %u bytes to %s, but only write %zu bytes.",
            dataSize, fileName, writeSize);
        (void)aclrtFreeHost(hostPtr);
        return FAILED;
    }

    (void)aclrtFreeHost(hostPtr);
    fflush(outFileFp);
    fclose(outFileFp);
    return SUCCESS;
}

void DestroyDecodeResource()
{
    if (decodeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(decodeOutputDesc_);
        decodeOutputDesc_ = nullptr;
    }
}

void DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}

int main()
{
    //1.ACL初始化
    const char *aclConfigPath = "../src/acl.json";
    aclInit(aclConfigPath);
    INFO_LOG("acl init success");
    //2.运行管理资源申请,包括Device、Context、Stream，stream_是aclrtStream类型
    aclrtSetDevice(deviceId_);
    INFO_LOG("open device %d success", deviceId_);
    // create context (set current)
    aclrtCreateContext(&context_, deviceId_);
    INFO_LOG("create context success");
    // create stream
    aclrtCreateStream(&stream_);
    INFO_LOG("create stream success");
    std::string dvppOutputfileName = "./dvpp_output";
    // loop begin
    PicDesc testPic = {"../data/dog1_1024_683.jpg", 1024, 683};

    INFO_LOG("start to process picture:%s", testPic.picName.c_str());
    // dvpp process
    //3.将图片读入内存，inDevBuffer_表示存放输入图片的内存, inDevBufferSize表示内存大小，输入内存要提前申请
    uint32_t devPicBufferSize;
    void *picDevBuffer = GetDeviceBufferOfPicture(testPic, devPicBufferSize);
    if (picDevBuffer == nullptr) {
        ERROR_LOG("get pic device buffer failed,index is 0");
        return FAILED;
    }
    //4.创建图片数据处理的通道
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    acldvppCreateChannel(dvppChannelDesc_);
    INFO_LOG("dvpp init resource success");


    SetInput(picDevBuffer, devPicBufferSize, testPic.width, testPic.height);

    // InitDecodeOutputDesc
    uint32_t decodeOutWidthStride = (inputWidth_ + 127) / 128 * 128; // 128-byte alignment
    uint32_t decodeOutHeightStride = (inputHeight_ + 15) / 16 * 16; // 16-byte alignment
    uint32_t decodeOutBufferSize = decodeOutWidthStride * decodeOutHeightStride * 3 / 2; // yuv format size
    aclError ret = acldvppMalloc(&decodeOutDevBuffer_, decodeOutBufferSize);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc jpegOutBufferDev failed, ret = %d", ret);
        return FAILED;
    }

    decodeOutputDesc_ = acldvppCreatePicDesc();
    if (decodeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc decodeOutputDesc failed");
        return FAILED;
    }

    acldvppSetPicDescData(decodeOutputDesc_, decodeOutDevBuffer_);
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(decodeOutputDesc_, inputWidth_);
    acldvppSetPicDescHeight(decodeOutputDesc_, inputHeight_);
    acldvppSetPicDescWidthStride(decodeOutputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(decodeOutputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(decodeOutputDesc_, decodeOutBufferSize);

    ret = acldvppJpegDecodeAsync(dvppChannelDesc_, inDevBuffer_, inDevBufferSize_,
        decodeOutputDesc_, stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppJpegDecodeAsync failed, ret = %d", ret);
        return FAILED;
    }

    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtSynchronizeStream failed");
        return FAILED;
    }

    decodeDataSize_ = acldvppGetPicDescSize(decodeOutputDesc_);

    (void)acldvppFree(picDevBuffer);
    picDevBuffer = nullptr;

    void *dvppOutputBuffer = nullptr;
    int dvppOutputSize;
    GetOutput(&dvppOutputBuffer, dvppOutputSize);
        std::string dvppOutputfileNameCur = dvppOutputfileName + ".yuv";
    ret = SaveDvppOutputData(dvppOutputfileNameCur.c_str(), dvppOutputBuffer, dvppOutputSize);
    if (ret != SUCCESS) {
        ERROR_LOG("save dvpp output data failed");
    // allow not return
    }
    DestroyDecodeResource();
    DestroyResource();

    INFO_LOG("execute sample success");
    return SUCCESS;
}
