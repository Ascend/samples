#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <memory>
#include <algorithm>
#include <dirent.h>
#include "AclLiteUtils.h"
#include "AclLiteError.h"
#include "AclLiteResource.h"
#include "AclLiteImageProc.h"
#include "AclLiteModel.h"
#include "acl/acl.h"
using namespace std;

namespace {
    const int32_t g_batch = 10;
    const int g_invalidSize = -1;
    const uint32_t g_lineSolid = 2;
    const uint32_t g_labelOffset = 11;
    const uint32_t g_eachResultTensorNum = 9;
    const double g_fountScale = 0.5;
    const cv::Scalar g_fontColor(0, 0, 255);
    const vector<cv::Scalar> g_colors {
    cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
    cv::Scalar(139, 85, 26) };
    const string g_carColorClass[9] = { "black", "blue", "brown", "green", "pink", "red",
                                        "silver", "white", "yellow"};
}

struct Rectangle {
    cv::Point lt;  // left top
    cv::Point rb;  // right bottom
};

struct CarInfo {
    ImageData cropedImgs;  // cropped image from original image
    ImageData resizedImgs;  // resized image for inference
    Rectangle rectangle;
    std::string detectClass;   // detect class
    std::string carColorResult;   // color
};

typedef struct BoundBox {
    float x;
    float y;
    float width;
    float height;
    float score;
    size_t classIndex;
    size_t index;
} BoundBox;

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

int CopyOneBatchImages(uint8_t* buffer, uint32_t dataSize, 
                       uint32_t bufferSize, vector<CarInfo> &carInfo, int batchIdx, aclrtRunMode runMode)
{
    uint32_t j = 0;
    int totalSize = 0;
    AclLiteError ret = ACLLITE_OK;
    for (uint32_t i = batchIdx * g_batch;
         i < carInfo.size() && j < g_batch && bufferSize > totalSize;
         i++, j++) {
        ret = CopyDataToDeviceEx(buffer + totalSize, bufferSize - totalSize, 
                                 carInfo[i].resizedImgs.data.get(), dataSize, runMode);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Copy data to device failed");
            return g_invalidSize;
        }
        totalSize += dataSize;
    }
    
    if (j < g_batch) {
        for (uint32_t k = 0; k < g_batch - j && bufferSize > totalSize; k++) {
            ret = CopyDataToDeviceEx(buffer + totalSize, bufferSize - totalSize, 
                                     carInfo[carInfo.size() - 1].resizedImgs.data.get(), dataSize, runMode);
            if (ret != ACL_SUCCESS) {
                ACLLITE_LOG_ERROR("Copy data to device failed");
                return g_invalidSize;
            }
            totalSize += dataSize;
        }
    }
    return j;
}

bool sortScore(BoundBox box1, BoundBox box2)
{
    return box1.score > box2.score;
}

class SampleCarColor {
    public:
    SampleCarColor(const char *detectModelPath, const int32_t detectModelWidth, 
                   const int32_t detectModelHeight, const char *colorModelPath, 
                   const int32_t colorModelWidth, const int32_t colorModelHeight);
    ~SampleCarColor();
    Result InitResource();
    Result detectProcessInput(ImageData& resizedImage, ImageData& srcImage, ImageData& yuvImage, string imageFile);
    Result colorProcessInput(ImageData& srcImage, std::vector<CarInfo> &carImgs, int& flag);
    Result detectInference(std::vector<InferenceOutput>& inferenceOutput, ImageData& resizedImage);
    Result colorInference(vector<CarInfo> &carInfo, std::vector<InferenceOutput>& inferenceOutput, int flag);
    Result detectGetResult(std::vector<InferenceOutput>& inferOutputs, string imagePath, std::vector<CarInfo>& carData);
    Result colorGetResult(std::vector<InferenceOutput>& inferenceOutput,
    std::vector<CarInfo>& carInfo, const string& origImagePath, int flag);
    private:
    void ReleaseResource();
    AclLiteResource aclResource_;
    aclrtRunMode runMode_;
    AclLiteImageProc imageProcess_;

    // detect 
    const int32_t detectModelWidth_;
    const int32_t detectModelHeight_;
    AclLiteModel detectModel_;
   
    // color
    const int32_t colorModelWidth_;
    const int32_t colorModelHeight_;
    AclLiteModel colorModel_;
    uint32_t colorInputSize_;
    uint8_t* colorInputBuf_;
};

SampleCarColor::SampleCarColor(const char *detectModelPath, const int32_t detectModelWidth, 
                               const int32_t detectModelHeight, const char *colorModelPath, 
                               const int32_t colorModelWidth, const int32_t colorModelHeight) :
detectModel_(detectModelPath), detectModelWidth_(detectModelWidth), detectModelHeight_(detectModelHeight), 
colorModel_(colorModelPath), colorModelWidth_(colorModelWidth), colorModelHeight_(colorModelHeight) 
{
}

SampleCarColor::~SampleCarColor()
{
    ReleaseResource();
}

Result SampleCarColor::InitResource()
{
    // init acl resource
    AclLiteError aclRet = aclResource_.Init();
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("resource init failed, errorCode is %d", aclRet);
        return FAILED;
    }

    aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("aclrtGetRunMode failed, errorCode is %d", aclRet);
        return FAILED;
    }

    aclRet = imageProcess_.Init();
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Dvpp init failed, errorCode is %d", aclRet);
        return FAILED;
    }

    aclRet = detectModel_.Init();
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("detectModel init failed, errorCode is %d", aclRet);
        return FAILED;
    }

    aclRet = colorModel_.Init();
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("colorModel init failed, errorCode is %d", aclRet);
        return FAILED;
    }

    colorInputSize_ = YUV420SP_SIZE(colorModelWidth_, colorModelHeight_) * g_batch;
    void* buf = nullptr;
    aclRet = aclrtMalloc(&buf, colorInputSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if ((buf == nullptr) || (aclRet != ACL_ERROR_NONE)) {
        ACLLITE_LOG_ERROR("Malloc inference input buffer failed, "
                          "errorCode is %d", aclRet);
        return FAILED;
    }
    colorInputBuf_ = (uint8_t *)buf;
    return SUCCESS;
}

Result SampleCarColor::detectProcessInput(ImageData& resizedImage, ImageData& srcImage, 
                                          ImageData& yuvImage, string imageFile)
{
    ReadJpeg(srcImage, imageFile);
    if (srcImage.data == nullptr) {
        ACLLITE_LOG_ERROR("Read image %s failed", imageFile.c_str());
        return FAILED;
    }

    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, srcImage, runMode_, MEMORY_DVPP);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Copy image to device failed, errorCode is %d", ret);
        return FAILED;
    }

    ret = imageProcess_.JpegD(yuvImage, imageDevice);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed, errorCode is %d", ret);
        return FAILED;
    }

    ret = imageProcess_.CropPaste(resizedImage, yuvImage, detectModelWidth_, detectModelHeight_,
                                  0, 0, yuvImage.width, yuvImage.height);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("CropResolution image failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleCarColor::colorProcessInput(ImageData& srcImage, std::vector<CarInfo> &carInfo, int& flag)
{
     // No car detected
    if (carInfo.size() == 0) {
        flag = 1;
        ACLLITE_LOG_INFO("No car detected");
        return SUCCESS;
    }

    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, srcImage, runMode_, MEMORY_DVPP);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Copy image to device failed, errorCode is %d", ret);
        return FAILED;
    }

    for (int i = 0; i < carInfo.size(); i++) {
        ret = imageProcess_.Crop(carInfo[i].cropedImgs, 
                                 imageDevice, 
                                 carInfo[i].rectangle.lt.x, 
                                 carInfo[i].rectangle.lt.y,
                                 carInfo[i].rectangle.rb.x, 
                                 carInfo[i].rectangle.rb.y);                                                         
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Crop image failed, error: %d, image width %d, "
                              "height %d, size %d, crop area (%d, %d) (%d, %d)",
                              ret, carInfo[i].cropedImgs.width, carInfo[i].cropedImgs.height,
                              carInfo[i].cropedImgs.size, carInfo[i].rectangle.lt.x,
                              carInfo[i].rectangle.lt.y, carInfo[i].rectangle.rb.x,
                              carInfo[i].rectangle.rb.y);
            return FAILED;
        }
    }

    for (size_t i = 0; i < carInfo.size(); i++) {
        AclLiteError ret = imageProcess_.Resize(carInfo[i].resizedImgs, carInfo[i].cropedImgs,
                                                colorModelWidth_, colorModelHeight_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("ColorClassify Resize image failed");
            return FAILED;
        }
    }

    return SUCCESS;
}

Result SampleCarColor::detectInference(std::vector<InferenceOutput>& inferenceOutput,
                                       ImageData& resizedImage)
{
    AclLiteError ret = detectModel_.CreateInput(resizedImage.data.get(), resizedImage.size);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed, errorCode is %d", ret);
        return FAILED;
    }

    ret = detectModel_.Execute(inferenceOutput);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Execute model inference failed, errorCode is %d", ret);
        return FAILED;
    }

    return SUCCESS;
}

Result SampleCarColor::colorInference(vector<CarInfo> &carInfo,
                                      std::vector<InferenceOutput>& inferenceOutput,
                                      int flag)
{
    if (flag == 1) {
        return SUCCESS;
    }
    int carInfoSize = carInfo.size();
    int batchNum = max(carInfoSize, g_batch) / g_batch;
    uint32_t dataSize = YUV420SP_SIZE(colorModelWidth_, colorModelHeight_);

    for (int i = 0; i < batchNum; i++) {
        // Copy one batch preprocessed image data to device
        int carNum = CopyOneBatchImages(colorInputBuf_, dataSize, colorInputSize_, carInfo, i, runMode_);
        if (carNum < 0) {
            ACLLITE_LOG_ERROR("Copy the %dth batch images failed", i);
            break;
        }

        // Inference one batch data
        AclLiteError ret = colorModel_.Execute(inferenceOutput, colorInputBuf_, colorInputSize_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Execute model inference failed\n");
            break;
        }
    }

    return SUCCESS;
}

Result SampleCarColor::detectGetResult(std::vector<InferenceOutput>& inferOutputs,
                                       string imagePath, std::vector<CarInfo>& carData)
{
    uint32_t outputDataBufId = 0;
    float *classBuff = static_cast<float *>(inferOutputs[outputDataBufId].data.get());
    // confidence threshold
    float confidenceThreshold = 0.25;

    // class number
    size_t classNum = 80;

    // number of (x, y, width, hight, confidence)
    size_t offset = 5;

    // total number = class number + (x, y, width, hight, confidence)
    size_t totalNumber = classNum + offset;

    // total number of boxs
    size_t modelOutputBoxNum = 25200;

    // top 5 indexes correspond (x, y, width, hight, confidence),
    // and 5~85 indexes correspond object's confidence
    size_t startIndex = 5;

    // read source image from file
    cv::Mat srcImage = cv::imread(imagePath);
    int srcWidth = srcImage.cols;
    int srcHeight = srcImage.rows;

    // filter boxes by confidence threshold
    vector <BoundBox> boxes;
    size_t yIndex = 1;
    size_t widthIndex = 2;
    size_t heightIndex = 3;
    size_t classConfidenceIndex = 4;
    for (size_t i = 0; i < modelOutputBoxNum; ++i) {
        float maxValue = 0;
        float maxIndex = 0;
        for (size_t j = startIndex; j < totalNumber; ++j) {
            float value = classBuff[i * totalNumber + j] * classBuff[i * totalNumber + classConfidenceIndex];
                if (value > maxValue) {
                // index of class
                maxIndex = j - startIndex;
                maxValue = value;
            }
        }
        float classConfidence = classBuff[i * totalNumber + classConfidenceIndex];
        if (classConfidence >= confidenceThreshold) {
            // index of object's confidence
            size_t index = i * totalNumber + maxIndex + startIndex;

            // finalConfidence = class confidence * object's confidence
            float finalConfidence =  classConfidence * classBuff[index];
            BoundBox box;
            box.x = classBuff[i * totalNumber] * srcWidth / detectModelWidth_;
            box.y = classBuff[i * totalNumber + yIndex] * srcHeight / detectModelHeight_;
            box.width = classBuff[i * totalNumber + widthIndex] * srcWidth / detectModelWidth_;
            box.height = classBuff[i * totalNumber + heightIndex] * srcHeight / detectModelHeight_;
            box.score = finalConfidence;
            box.classIndex = maxIndex;
            box.index = i;
            if (maxIndex < classNum) {
                boxes.push_back(box);
            }
        }
    }

    // filter boxes by NMS
    vector <BoundBox> result;
    result.clear();
    float NMSThreshold = 0.45;
    int32_t maxLength = detectModelWidth_ > detectModelHeight_ ? detectModelWidth_ : detectModelHeight_;
    std::sort(boxes.begin(), boxes.end(), sortScore);
    BoundBox boxMax;
    BoundBox boxCompare;
    while (boxes.size() != 0) {
        size_t index = 1;
        result.push_back(boxes[0]);
        while (boxes.size() > index) {
            boxMax.score = boxes[0].score;
            boxMax.classIndex = boxes[0].classIndex;
            boxMax.index = boxes[0].index;

            // translate point by maxLength * boxes[0].classIndex to
            // avoid bumping into two boxes of different classes
            boxMax.x = boxes[0].x + maxLength * boxes[0].classIndex;
            boxMax.y = boxes[0].y + maxLength * boxes[0].classIndex;
            boxMax.width = boxes[0].width;
            boxMax.height = boxes[0].height;

            boxCompare.score = boxes[index].score;
            boxCompare.classIndex = boxes[index].classIndex;
            boxCompare.index = boxes[index].index;

            // translate point by maxLength * boxes[0].classIndex to
            // avoid bumping into two boxes of different classes
            boxCompare.x = boxes[index].x + boxes[index].classIndex * maxLength;
            boxCompare.y = boxes[index].y + boxes[index].classIndex * maxLength;
            boxCompare.width = boxes[index].width;
            boxCompare.height = boxes[index].height;

            // the overlapping part of the two boxes
            float xLeft = max(boxMax.x, boxCompare.x);
            float yTop = max(boxMax.y, boxCompare.y);
            float xRight = min(boxMax.x + boxMax.width, boxCompare.x + boxCompare.width);
            float yBottom = min(boxMax.y + boxMax.height, boxCompare.y + boxCompare.height);
            float width = max(0.0f, xRight - xLeft);
            float hight = max(0.0f, yBottom - yTop);
            float area = width * hight;
            float iou =  area / (boxMax.width * boxMax.height + boxCompare.width * boxCompare.height - area);

            // filter boxes by NMS threshold
            if (iou > NMSThreshold) {
                boxes.erase(boxes.begin() + index);
                continue;
            }
            ++index;
        }
        boxes.erase(boxes.begin());
    }

    int half = 2;
    for (size_t i = 0; i < result.size(); ++i) {
        CarInfo carInfo;
        carInfo.rectangle.lt.x = result[i].x - result[i].width / half;
        carInfo.rectangle.lt.y = result[i].y - result[i].height / half;
        carInfo.rectangle.rb.x = result[i].x + result[i].width / half;
        carInfo.rectangle.rb.y = result[i].y + result[i].height / half;
        carInfo.detectClass = "car" + std::to_string(result[i].score) + "\%";
        carData.emplace_back(carInfo);
    }
    classBuff = nullptr;

    return SUCCESS;
}

Result SampleCarColor::colorGetResult(std::vector<InferenceOutput>& inferenceOutput, 
                                      std::vector<CarInfo>& carInfo, const string& origImagePath,
                                      int flag)
{
    int pos = origImagePath.find_last_of("/");
    string fileName(origImagePath.substr(pos + 1));
    stringstream sstream;
    cv::Mat origImage = cv::imread(origImagePath, CV_LOAD_IMAGE_UNCHANGED);
    if (flag == 1) {
        sstream.str("");
        sstream << "./out_" << fileName;
        cv::imwrite(sstream.str(), origImage);
        return SUCCESS;
    }
    void* data = (void *)inferenceOutput[0].data.get();
    if (data == nullptr) {
        return FAILED;
    }
    float* outData = nullptr;
    outData = static_cast<float*>(data);

    for (int i = 0; i < carInfo.size(); i++) {
        int maxConfidentIndex = i * g_eachResultTensorNum;
        for (int j = 0; j < g_eachResultTensorNum; j++) {
            int index = i * g_eachResultTensorNum + j;
            if (outData[index] > outData[maxConfidentIndex]) {
                maxConfidentIndex = index;
            }
        }
        int colorIndex = maxConfidentIndex - i * g_eachResultTensorNum;
        carInfo[i].carColorResult = g_carColorClass[colorIndex];
    }

    // draw the result
    for (int i = 0; i < carInfo.size(); ++i) {
        cv::rectangle(origImage, carInfo[i].rectangle.lt, carInfo[i].rectangle.rb,
                      g_colors[i % g_colors.size()], g_lineSolid);
        cv::putText(origImage, carInfo[i].detectClass,
                    cv::Point(carInfo[i].rectangle.lt.x - g_labelOffset,
                              carInfo[i].rectangle.lt.y - g_labelOffset),
                    cv::FONT_HERSHEY_COMPLEX, g_fountScale, g_fontColor);
        cv::putText(origImage, carInfo[i].carColorResult,
                    cv::Point(carInfo[i].rectangle.lt.x,
                              carInfo[i].rectangle.lt.y + g_labelOffset),
                    cv::FONT_HERSHEY_COMPLEX, g_fountScale, g_fontColor);
        ACLLITE_LOG_INFO("%d %d %d %d  %s", carInfo[i].rectangle.lt.x, carInfo[i].rectangle.lt.y,
                         carInfo[i].rectangle.rb.x, carInfo[i].rectangle.rb.y,
                         carInfo[i].detectClass.c_str());
    }
    sstream.str("");
    sstream << "./out_" << fileName;
    cv::imwrite(sstream.str(), origImage);
    data = nullptr;
    outData = nullptr;
    return SUCCESS;
}

void SampleCarColor::ReleaseResource()
{
    detectModel_.DestroyResource();
    colorModel_.DestroyResource();
    imageProcess_.DestroyResource();
    aclResource_.Release();
}

int main(int argc, char *argv[])
{
    const char* detectModelPath = "../model/yolov7x.om";
    uint32_t detectModelWidth = 640;
    uint32_t detectModelHeight = 640;
    const char* colorModelPath = "../model/color_dvpp_10batch.om";
    uint32_t colorModelWidth = 224;
    uint32_t colorModelHeight = 224;
    
    SampleCarColor sample(detectModelPath, detectModelWidth, detectModelHeight, 
                          colorModelPath, colorModelWidth, colorModelHeight) ;
    AclLiteError ret = sample.InitResource();
    if (ret == FAILED) {
        ACLLITE_LOG_ERROR("Init detect failed, errorCode is %d", ret);
        return FAILED;
    }

    string inputImageDir = "../data";
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s", inputImageDir.c_str());
        return FAILED;
    }

    ImageData image;
    for (string imageFile : fileVec) {
        ImageData resizedImage, yuvImage;
        ret = sample.detectProcessInput(resizedImage, image, yuvImage, imageFile);
        if (ret == FAILED) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",
                              imageFile.c_str());
            continue;
        }

        std::vector<InferenceOutput> detectInferOutput;
        ret = sample.detectInference(detectInferOutput, resizedImage);
        if (ret == FAILED) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed, errorCode is %d", ret);
            return FAILED;
        }

        vector<CarInfo> carInfo;
        ret = sample.detectGetResult(detectInferOutput, imageFile, carInfo);
        if (ret == FAILED ) {
            ACLLITE_LOG_ERROR("Process model inference output data failed, errorCode is %d", ret);
            return FAILED;
        }

        int flag = 0;
        ret = sample.colorProcessInput(yuvImage, carInfo, flag);
        if (ret == FAILED) {
            ACLLITE_LOG_ERROR("Process model inference input data failed, errorCode is %d", ret);
            return FAILED;
        }
        if (flag ==1) {
            ACLLITE_LOG_INFO("Nothing found");
        }

        std::vector<InferenceOutput> classifyInferOutput;
        ret = sample.colorInference(carInfo, classifyInferOutput, flag);
        if (ret == FAILED) {
            ACLLITE_LOG_ERROR("Process model inference input data failed, errorCode is %d", ret);
            return FAILED;
        }
        
        ret = sample.colorGetResult(classifyInferOutput, carInfo, imageFile, flag);
        if (ret == FAILED) {
            ACLLITE_LOG_ERROR("Process model inference output data failed, errorCode is %d", ret);
            return FAILED;
        }
    }
    ACLLITE_LOG_INFO("Execute sample success");
    return SUCCESS;
}