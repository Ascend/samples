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

* File sample_process.cpp
* Description: handle acl resource
*/
#include "classify_process.h"
#include <iostream>

#include "acl/acl.h"
#include "model_process.h"
#include "image_net_classes.h"
#include "utils.h"

using namespace std;

namespace {
    uint32_t kTopNConfidenceLevels = 5;
}

ClassifyProcess::ClassifyProcess(const char* modelPath,
uint32_t modelWidth, uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), inputBuf_(nullptr),
modelWidth_(modelWidth), modelHeight_(modelHeight), isInited_(false){
    modelPath_ = modelPath;
    inputDataSize_ = RGBU8_IMAGE_SIZE(modelWidth_, modelHeight_);
}

ClassifyProcess::~ClassifyProcess() {
    DestroyResource();
}

Result ClassifyProcess::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("Open device %d success", deviceId_);

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ClassifyProcess::InitModel(const char* omModelPath) {
    Result ret = model_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = model_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = model_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ERROR_LOG("Acl malloc image buffer failed.");
        return FAILED;
    }

    ret = model_.CreateInput(inputBuf_, inputDataSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    return SUCCESS;
}

//add by cz,20200925
double ClassifyProcess::getPSNR(const cv::Mat &I1, const cv::Mat&I2){
    cv::Mat s1;
    cv::absdiff(I1, I2, s1);
    s1.convertTo(s1, CV_32F);
    s1 = s1.mul(s1);
    cv::Scalar s = sum(s1);
    double sse = s.val[0] + s.val[1] + s.val[2];
    if(sse <= 1e-10){
        return 0;
    }
    else{
        double mse = sse / (double)(I1.channels() * I1.total());
        double psnr = 10.0 * log10(255 * 255 / mse);
        return psnr;
    }
}

double ClassifyProcess::getSSIM(const cv::Mat& i1, const cv::Mat& i2){
    const double C1 = 6.5025, C2 = 58.5225;
    int d = CV_32F;

    cv::Mat I1, I2;
    i1.convertTo(I1, d);
    i2.convertTo(I2, d);
    cv::Mat I2_2 = I2.mul(I2);     // I2^2
    cv::Mat I1_2 = I1.mul(I1);      //I1^2
    cv::Mat I1_I2 = I1.mul(I2);     // I1*I2

    cv::Mat mu1, mu2;
    GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    GaussianBlur(I2, mu2 ,cv::Size(11, 11), 1.5);

    cv::Mat mu1_2 = mu1.mul(mu1);
    cv::Mat mu2_2 = mu2.mul(mu2);
    cv::Mat mu1_mu2 = mu1.mul(mu2);

    cv::Mat sigma1_2, sigma2_2, sigma12;

    cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;

    cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11,11), 1.5);
    sigma2_2 -= mu2_2;

    cv::GaussianBlur(I1_I2, sigma12, cv::Size(11,11), 1.5);
    sigma12 -= mu1_mu2;

    cv::Mat t1, t2, t3;
    t1 = 2*mu1_mu2 + C1;
    t2 = 2*sigma12 + C2;
    t3 = t1.mul(t2);

    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);

    cv::Mat ssim_map;
    cv::divide(t3, t1, ssim_map);
    cv::Scalar mssim = mean(ssim_map);
    double ssim = (mssim.val[0] + mssim.val[1] + mssim.val[2]) / I1.channels();
    return ssim;
}

Result ClassifyProcess::Init() {
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }

    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }

    ret = InitModel(modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result ClassifyProcess::Preprocess(const string& imageFile) {
    // read image using OPENCV
    INFO_LOG("Read image %s", imageFile.c_str());
    cv::Mat origMat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);   //BGR
    if (origMat.empty()) {
        ERROR_LOG("Read image failed");
        return FAILED;
    }

    INFO_LOG("Resize image %s", imageFile.c_str());
    //resize
    cv::Mat resizeMat;
    cv::resize(origMat, resizeMat, cv::Size(modelWidth_, modelHeight_));
    if (resizeMat.empty()) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }

    //add
    cv::cvtColor(resizeMat, resizeMat, cv::COLOR_BGR2RGB);

    if (runMode_ == ACL_HOST) {
        //AI1上运行时,需要将图片数据拷贝到device侧
        aclError ret = aclrtMemcpy(inputBuf_, inputDataSize_,
        resizeMat.ptr<uint8_t>(), inputDataSize_,
        ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("Copy resized image data to device failed.");
            return FAILED;
        }
    } else {
        //Atals200DK上运行时,数据拷贝到本地即可.
        //reiszeMat是局部变量,数据无法传出函数,需要拷贝一份
        memcpy(inputBuf_, resizeMat.ptr<void>(), inputDataSize_);
    }

    return SUCCESS;
}

Result ClassifyProcess::Inference(aclmdlDataset*& inferenceOutput) {
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }
    //Release model input buffer
    //model_.DestroyInput();

    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result ClassifyProcess::Postprocess(const string& origImageFile,
aclmdlDataset* modelOutput){
    //INFO_LOG("Postprocess image");

    uint32_t dataSize = 0;
    void* data = GetInferenceOutputItem(dataSize, modelOutput);
    if (data == nullptr) {
        return FAILED;
    }
    INFO_LOG("InferenceOutput dataSize: %d BYTE", dataSize); //output size 262144 BYTE(1*256*256*1*4Byte)

    float* outData = NULL;
    outData = reinterpret_cast<float*>(data);

    //add by cz
    //read origImageFile,replace Y channel with InferenceOutput 'outData'
    //=============================================================================
    int pos = origImageFile.find_last_of("/");
    string filename(origImageFile.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_"  << filename;
    string outputPath = sstream.str();

    cv::Mat inputImage = cv::imread(origImageFile, CV_LOAD_IMAGE_COLOR);
    cv::Mat yuvImgIn,yuvImgOut;
    //cv::cvtColor(inputImage,yuvImgIn,cv::COLOR_BGR2YUV_YV12 );
    //cv::cvtColor(inputImage,yuvImgIn,cv::COLOR_BGR2YUV_I420);
    cv::cvtColor(inputImage,yuvImgIn,cv::COLOR_BGR2YCrCb);

    yuvImgOut = yuvImgIn.clone();

    INFO_LOG("YCrCb channel: %d", yuvImgIn.channels());

//    cv::Mat tempYCrCb;
//    cv::cvtColor(yuvImgIn,tempYCrCb,cv::COLOR_YCrCb2BGR);
//    cv::imwrite(outputPath, tempYCrCb);

    for (uint32_t i = 0; i < 256; i++){
        for(uint32_t j = 0; j < 256; j++){
            //把outData中的数据拷贝至yuvImg.data[k];
            if(i*256+j<100)
            {
                //cout<<"result"<<k<<"="<<(float)(outData[k])<<endl;  //调试时打印输出
            }
            outData[i*256+j] *= 0.92;
            if(outData[i*256+j]<0.0)
            {
                //yuvImgOut.data[i*256+j] = 0;
                yuvImgOut.at<cv::Vec3b>(i,j)[0] = 0;
            }
            else if(outData[i*256+j]>1.0)
            {
                //yuvImgOut.data[i*256+j] = 255;
                yuvImgOut.at<cv::Vec3b>(i,j)[0] = 255;
            }
            else
            {
                //yuvImgOut.data[i*256+j] = (unsigned char)(outData[i*256+j]*255*1.0); //强制类型转换
                yuvImgOut.at<cv::Vec3b>(i,j)[0] = (unsigned char)(outData[i*256+j]*255);
            }
        }
    }
    cv::Mat outputImage;
    //cv::cvtColor(yuvImgOut,outputImage,cv::COLOR_YUV2BGR_YV12 );
    //cv::cvtColor(yuvImgOut,outputImage, cv::COLOR_YUV2BGR_I420 );
    cv::cvtColor(yuvImgOut,outputImage, cv::COLOR_YCrCb2BGR);
    cv::imwrite(outputPath, outputImage);

#ifdef CalcuPSNR
    int posofSlash = origImageFile.find_last_of("/");
    int posofUnderline = origImageFile.find_last_of("_");
    string testPicNum(origImageFile.substr(posofSlash + 1,posofUnderline - posofSlash - 1));

    stringstream sstream1;
    sstream1.str("");
    sstream1 << "../data/" << testPicNum << "_GT.png";
    string GTPicPath = sstream1.str();

    cv::Mat GTImage = cv::imread(GTPicPath, CV_LOAD_IMAGE_COLOR);

    //RGB channel
    double dPSNR = getPSNR(GTImage,outputImage);
    double dSSIM = getSSIM(GTImage,outputImage);

    //R/G/B channel
//    std::vector<cv::Mat> GTImageChannels,outputImageChannels;
//    cv::split(GTImage, GTImageChannels);
//    cv::split(outputImage, outputImageChannels);
//    double dPSNR = getPSNR(GTImageChannels[0],outputImageChannels[0]);
//    double dSSIM = getSSIM(GTImageChannels[0],outputImageChannels[0]);

    //YUV channel
//    double dPSNR = getPSNR(yuvImgIn,yuvImgOut);
//    double dSSIM = getSSIM(yuvImgIn,yuvImgOut);

    //Y channel
//    cv::Mat grayImgIn,grayImgOut;
//    cv::cvtColor(yuvImgIn,grayImgIn,cv::COLOR_YUV2GRAY_YV12 );
//    cv::cvtColor(yuvImgOut,grayImgOut,cv::COLOR_YUV2GRAY_YV12 );
//    double dPSNR = getPSNR(grayImgIn,grayImgOut);
//    double dSSIM = getSSIM(grayImgIn,grayImgOut);

    //INFO_LOG("PSNR: %.2lf , SSIM: %.2lf. ",dPSNR, dSSIM);

    dPSNRs.push_back(dPSNR);
    dSSIMs.push_back(dSSIM);
    //=============================================================================
#endif
    return SUCCESS;
}

void ClassifyProcess::PrintMeanPSNR(){
    double MeanPSNR = accumulate(begin(dPSNRs),end(dPSNRs),0.0) / dPSNRs.size();
    double MeanSSIM = accumulate(begin(dSSIMs),end(dSSIMs),0.0) / dSSIMs.size();
    INFO_LOG("MeanPSNR: %.2lf , MeanSSIM: %.2lf. ",MeanPSNR, MeanSSIM);
}

void* ClassifyProcess::GetInferenceOutputItem(uint32_t& itemDataSize,
aclmdlDataset* inferenceOutput) {
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(inferenceOutput, 0);
    if (dataBuffer == nullptr) {
        ERROR_LOG("Get the dataset buffer from model "
        "inference output failed");
        return nullptr;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ERROR_LOG("Get the dataset buffer address "
        "from model inference output failed");
        return nullptr;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The dataset buffer size of "
        "model inference output is 0");
        return nullptr;
    }

    void* data = nullptr;
    if (runMode_ == ACL_HOST) {
        data = Utils::CopyDataDeviceToLocal(dataBufferDev, bufferSize);
        if (data == nullptr) {
            ERROR_LOG("Copy inference output to host failed");
            return nullptr;
        }
    }
    else {
        data = dataBufferDev;
    }

    itemDataSize = bufferSize;
    return data;
}

void ClassifyProcess::LabelClassToImage(int classIdx, const string& origImagePath) {
    cv::Mat resultImage = cv::imread(origImagePath, CV_LOAD_IMAGE_COLOR);

    // generate colorized image
    int pos = origImagePath.find_last_of("/");
    string filename(origImagePath.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_"  << filename;

    string outputPath = sstream.str();
    string text;

    if (classIdx < 0 || classIdx >= IMAGE_NET_CLASSES_NUM) {
        text = "none";
    } else {
        text = g_k_strlmag_net_classes[classIdx];
    }

    int fontFace = 0;
    double fontScale = 1;
    int thickness = 2;
  //  int baseline = 0;
    cv::Point origin;
    origin.x = 10;
    origin.y = 50;
    cv::putText(resultImage, text, origin, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 4, 0);
    cv::imwrite(outputPath, resultImage);
}

void ClassifyProcess::DestroyResource()
{   aclrtFree(inputBuf_);
    inputBuf_ = nullptr;
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
