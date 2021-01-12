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
* Description: dvpp sample main func
*/
#include <cstdint>
#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include "acl/acl.h"
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/opencv.hpp"
#include "acl/ops/acl_dvpp.h"
#include "main.h"

using namespace std;

int32_t deviceId;
aclrtContext context;
aclrtStream stream;
aclrtRunMode runMode;
pthread_t threadId;

aclvencChannelDesc *vencChannelDesc = nullptr;
acldvppPicDesc *vpcInputDesc = nullptr;
aclvencFrameConfig *vencFrameConfig = nullptr;
acldvppStreamDesc *outputStreamDesc = nullptr;
void *codeInputBufferDev = nullptr;
acldvppPixelFormat format;
int32_t enType = 2;
uint32_t inputBufferSize;
FILE *outFileFp;
bool runFlag= true;

void *ThreadFunc(void *arg)
{
    // Notice: create context for this thread
    int deviceId = 0;
    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, deviceId);
    while (runFlag) {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
        //pthread_testcancel();
    }
    ret = aclrtDestroyContext(context);
    return (void*)0;
}
bool WriteToFile(FILE *outFileFp_, const void *dataDev, uint32_t dataSize)
{
    bool ret = true;
    size_t writeRet = fwrite(dataDev, 1, dataSize, outFileFp_);
    if (writeRet != dataSize) {
        ret = false;
    }
    fflush(outFileFp_);

    return ret;
}

void callback(acldvppPicDesc *input, acldvppStreamDesc *outputStreamDesc, void *userdata)
{
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
    void *outputDev = acldvppGetStreamDescData(outputStreamDesc);
    uint32_t streamDescSize = acldvppGetStreamDescSize(outputStreamDesc);
    bool ret;

    if (runMode == ACL_HOST) {
        void * hostPtr = nullptr;
        aclrtMallocHost(&hostPtr, streamDescSize);
        aclrtMemcpy(hostPtr, streamDescSize, outputDev, streamDescSize, ACL_MEMCPY_DEVICE_TO_HOST);
        ret = WriteToFile(outFileFp, hostPtr, streamDescSize);
        (void)aclrtFreeHost(hostPtr);
    }
    else{
        ret = WriteToFile(outFileFp, outputDev, streamDescSize);
    }

    if (!ret) {
        ERROR_LOG("write file failed.");
    }
    INFO_LOG("success to callback, stream size:%u", streamDescSize);
}

Result InitResource(){
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl open device %d failed", deviceId);
        return FAILED;
    }
    INFO_LOG("Open device %d success", deviceId);

    ret = aclrtCreateContext(&context, deviceId);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
    INFO_LOG("create context success");

    //Gets whether the current application is running on host or Device
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }
}

Result Init(int imgWidth, int imgHeight){

    InitResource();

    pthread_create(&threadId, nullptr, ThreadFunc, nullptr);
    int width = imgWidth;
    int height = imgHeight;
    uint32_t alignWidth = ALIGN_UP128(width);
    uint32_t alignHeight = ALIGN_UP16(height);
    if (alignWidth == 0 || alignHeight == 0) {
        ERROR_LOG("InitCodeInputDesc AlignmentHelper failed. image w %d, h %d, align w%u, h%u",
        width, height, alignWidth, alignHeight);
        return FAILED;
    }
    //Allocate a large enough memory
    inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    aclError ret = acldvppMalloc(&codeInputBufferDev, inputBufferSize);

    format = static_cast<acldvppPixelFormat>(PIXEL_FORMAT_YVU_SEMIPLANAR_420);
    vencFrameConfig = aclvencCreateFrameConfig();
    aclvencSetFrameConfigForceIFrame(vencFrameConfig, 0);
    if (vencFrameConfig == nullptr) {
        ERROR_LOG("Dvpp init failed for create config failed");
        return FAILED;
    }

    vencChannelDesc = aclvencCreateChannelDesc();
    if (vencChannelDesc == nullptr) {
        ERROR_LOG("aclvencCreateChannelDesc failed");
        return FAILED;
    }
    ret = aclvencSetChannelDescThreadId(vencChannelDesc, threadId);
    ret = aclvencSetChannelDescCallback(vencChannelDesc, callback);
    ret = aclvencSetChannelDescEnType(vencChannelDesc, static_cast<acldvppStreamFormat>(enType));
    ret = aclvencSetChannelDescPicFormat(vencChannelDesc, format);
    ret = aclvencSetChannelDescKeyFrameInterval(vencChannelDesc, 1);
    ret = aclvencSetChannelDescPicWidth(vencChannelDesc, width);
    ret = aclvencSetChannelDescPicHeight(vencChannelDesc, height);
    ret = aclvencCreateChannel(vencChannelDesc);

    vpcInputDesc = acldvppCreatePicDesc();
    if (vpcInputDesc == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcInputDesc failed");
        return FAILED;
    }
    ret = acldvppSetPicDescFormat(vpcInputDesc, format);
    ret = acldvppSetPicDescWidth(vpcInputDesc, width);
    ret = acldvppSetPicDescHeight(vpcInputDesc, height);
    ret = acldvppSetPicDescWidthStride(vpcInputDesc, alignWidth);
    ret = acldvppSetPicDescHeightStride(vpcInputDesc, alignHeight);
    INFO_LOG("dvpp init resource ok");
    return SUCCESS;
}

Result Process(cv::Mat& srcImage) {
    aclError ret;
    if(runMode == ACL_HOST) {
        ret = aclrtMemcpy(codeInputBufferDev, inputBufferSize, srcImage.data, inputBufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
    }
    else {
        ret = aclrtMemcpy(codeInputBufferDev, inputBufferSize, srcImage.data, inputBufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }

    ret = acldvppSetPicDescData(vpcInputDesc, codeInputBufferDev);
    ret = acldvppSetPicDescSize(vpcInputDesc, inputBufferSize);

    ret = aclvencSendFrame(vencChannelDesc, vpcInputDesc,
    static_cast<void *>(outputStreamDesc), vencFrameConfig, nullptr);
    return SUCCESS;
}

void DestroyAclResource(){
    aclError ret;
    if (context != nullptr) {
        ret = aclrtDestroyContext(context);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}

void DestroyResource(){

    aclvencSetFrameConfigEos(vencFrameConfig, 1);
    aclvencSetFrameConfigForceIFrame(vencFrameConfig, 0);
    aclvencSendFrame(vencChannelDesc, nullptr, nullptr, vencFrameConfig, nullptr);

    if (vencFrameConfig != nullptr) {
        (void)aclvencDestroyFrameConfig(vencFrameConfig);
        vencFrameConfig = nullptr;
    }

    if (vpcInputDesc != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc);
        vpcInputDesc = nullptr;
    }

    if (codeInputBufferDev != nullptr) {
        (void)acldvppFree(codeInputBufferDev);
        codeInputBufferDev = nullptr;
    }

    if (outputStreamDesc != nullptr) {
        (void)acldvppDestroyStreamDesc(outputStreamDesc);
        outputStreamDesc = nullptr;
    }

    aclError aclRet;
    if (vencChannelDesc != nullptr) {
        aclRet = aclvencDestroyChannel(vencChannelDesc);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("aclvencDestroyChannel failed, aclRet = %d", aclRet);
        }
        (void)aclvencDestroyChannelDesc(vencChannelDesc);
        vencChannelDesc = nullptr;
    }

    runFlag = false;
    void *res = nullptr;
    pthread_cancel(threadId);
    pthread_join(threadId, &res);
    fclose(outFileFp);
    DestroyAclResource();
    INFO_LOG("end to destroy Resource");
}

int main(int argc, char *argv[]) {
    //Check the input when the application executes, which takes the path to the input video file
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }

    //Use Opencv to open the video file
    string videoFile = string(argv[1]);
    printf("open %s\n", videoFile.c_str());
    cv::VideoCapture capture(videoFile);
    if (!capture.isOpened()) {
        cout << "Movie open Error" << endl;
        return FAILED;
    }
    cout << "Movie open success" << endl;

    int imgHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    int imgWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));

    //open the output file
    string outputPath = "./output/out_video.h264";
    outFileFp = fopen(outputPath.c_str(), "ab");
    if(outFileFp == nullptr)    {
        ERROR_LOG("Failed to open  file %s.", outputPath.c_str());
        return FAILED;
    }

    Init(imgWidth, imgHeight);
    cv::Mat frame;
    while(1) {
        //Read a frame of an image
        if (!capture.read(frame)) {
            INFO_LOG("Video capture return false");
            break;
        }
        int cols = frame.cols;
        int rows = frame.rows;
        int Yindex = 0;
        int UVindex = rows * cols;
        cv::Mat NV21(rows+rows/2, cols, CV_8UC1);
        int UVRow{ 0 };
        for (int i=0;i<rows;i++)
        {
            for (int j=0;j<cols;j++)
            {
                uchar* YPointer = NV21.ptr<uchar>(i);
                int B = frame.at<cv::Vec3b>(i, j)[0];
                int G = frame.at<cv::Vec3b>(i, j)[1];
                int R = frame.at<cv::Vec3b>(i, j)[2];
                int Y = (77 * R + 150 * G + 29 * B) >> 8;
                YPointer[j] = Y;
                uchar* UVPointer = NV21.ptr<uchar>(rows+i/2);
                if (i%2==0&&(j)%2==0)
                {
                    int U = ((-44 * R - 87 * G + 131 * B) >> 8) + 128;
                    int V = ((131 * R - 110 * G - 21 * B) >> 8) + 128;
                    UVPointer[j] = V;
                    UVPointer[j+1] = U;
                }
            }
        }
        Process(NV21);
    }
    DestroyResource();
    INFO_LOG("Execute video object detection success");
    return SUCCESS;
}
