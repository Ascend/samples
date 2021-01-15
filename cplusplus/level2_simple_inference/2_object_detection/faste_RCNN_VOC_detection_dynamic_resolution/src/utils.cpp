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

* File utils.cpp
* Description: handle file operations
*/
#include "utils.h"
#include <map>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <dirent.h>
#include <vector>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "acl/acl.h"

using namespace std;

aclrtRunMode Utils::runMode_ = ACL_DEVICE;

const static std::vector<string> label = {
    "background","aeroplane","bicycle",
    "bird","boat",
    "bottle","bus",
    "car","cat","chair",
    "cow","diningtable","dog",
    "horse","motorbike","person",
    "pottedplant","sheep","sofa",
    "train","tvmonitor"};

enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };

Result Utils::CreateImageInfoBuffer(void* &imageInfoBuf, uint32_t &imageInfoSize)
{
    static auto it = resizeWidthHeight.begin();
    if (it == resizeWidthHeight.end()){
        it = resizeWidthHeight.begin();
    }


    const float imageInfo[3] = {(float)(it->second), (float)(it->first),  (float)3};

    aclError ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    if (runMode_ == ACL_HOST){
        aclError aclRet = aclrtMemcpy(imageInfoBuf, imageInfoSize, (void *)(imageInfo), imageInfoSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
            (void)aclrtFree(imageInfoBuf);
            return FAILED;
        }
    }
    else{
        aclError aclRet = aclrtMemcpy(imageInfoBuf, imageInfoSize, (void *)(imageInfo), imageInfoSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
            (void)aclrtFree(imageInfoBuf);
            return FAILED;
        }
    }
    it++;
    return SUCCESS;
}

Result Utils::PostProcess(const string &path, aclmdlDataset *modelOutput)
{
    size_t outDatasetNum = aclmdlGetDatasetNumBuffers(modelOutput);
	if (outDatasetNum != 2) {
		ERROR_LOG("outDatasetNum=%zu must be 2",outDatasetNum);
		return FAILED;
	}

    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelOutput, 1);
    if (dataBuffer == nullptr) {
        ERROR_LOG("get model output aclmdlGetDatasetBuffer failed");
        return FAILED;
    }

    void* data = aclGetDataBufferAddr(dataBuffer);
    if (data == nullptr) {
        ERROR_LOG("aclGetDataBufferAddr from dataBuffer failed.");
        return FAILED;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The dataset buffer size of "
        "model inference output is 0");
        return FAILED;
    }

    float * boxInfo = new float[bufferSize/sizeof(float)];
    if (runMode_ == ACL_HOST) {
        aclError ret = aclrtMemcpy(boxInfo, bufferSize, data, bufferSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("box count aclrtMemcpy failed!");
            return FAILED;
        }
    }
    else {
        aclError ret = aclrtMemcpy(boxInfo, bufferSize, data, bufferSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("box count aclrtMemcpy failed!");
            return FAILED;
        }
    }

    cv::Mat resultImage = cv::imread(path, CV_LOAD_IMAGE_COLOR);

    float image_width = resultImage.cols;
    float image_height = resultImage.rows;

    static auto it = resizeWidthHeight.begin();
    if (it == resizeWidthHeight.end()){
        it = resizeWidthHeight.begin();
    }


    float widthScale = (float)(image_width) / it->first;
    float heightScale = (float)(image_height) / it->second;
    ofstream inferRes;
    inferRes.open("inference_result.txt",ios::app);

    stringstream inferResultStream;
    inferResultStream.str("");
    inferResultStream << "image = " << path << endl;
    for (int i=1; i<classNum; i++) {
        for (int j=0; j<singleClassMaxBox; j++) {
            uint32_t score = uint32_t(boxInfo[SCORE + j*boxNum + i*singleClassMaxBox*boxNum] * 100);
            if (score < 95 || score > 100) continue;
            uint32_t objIndex = (uint32_t)boxInfo[LABEL + j*boxNum + i*singleClassMaxBox*boxNum];
            if (objIndex <= 0 || objIndex > 21 ) continue;
            
            float ltX_Scale = boxInfo[TOPLEFTX + j*boxNum + i*singleClassMaxBox*boxNum] / it->first;
            float ltY_Scale = boxInfo[TOPLEFTY + j*boxNum + i*singleClassMaxBox*boxNum] / it->second;
            float rbX_Scale = boxInfo[BOTTOMRIGHTX + j*boxNum + i*singleClassMaxBox*boxNum] / it->first;
            float rbY_Scale = boxInfo[BOTTOMRIGHTY + j*boxNum + i*singleClassMaxBox*boxNum] / it->second;

            inferResultStream << "label = " << label[objIndex] << ";" << "score = " << score << ";" << "ltX_Scale = " << ltX_Scale << ";" << "ltY_Scale = " << ltY_Scale << ";" << "rbX_Scale = " << rbX_Scale \
            << ";" << "rbY_Scale = " << rbY_Scale << endl;

            string text = label[objIndex] + std::to_string(score) + "\%";
            cv::Point p1, p2;
            p1.x = ltX_Scale * image_width;
            p1.y = ltY_Scale * image_height;
            p2.x = rbX_Scale * image_width;
            p2.y = rbY_Scale * image_height;
            cv::rectangle(resultImage, p1, p2, cv::Scalar(255, 0, 255),1, 8,0);
            cv::putText(resultImage, text, cv::Point(p1.x, p1.y), 0, 1, cv::Scalar(0, 255, 255), 2, 4, 0);
        }
    }
    inferResultStream << endl;
    inferRes << inferResultStream.str();
    it++;

	// generate result image
    string cmd("mkdir -p ./output");
    system(cmd.c_str());

    int pos = path.find_last_of(kFileSperator);
    string file_name(path.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output" << kFileSperator << kOutputFilePrefix << file_name;

    string outputPath = sstream.str();
	cv::imwrite(outputPath, resultImage);
    
    inferRes.close();
    delete [] boxInfo;

    return SUCCESS;
}


bool Utils::IsDirectory(const string &path) {
  // get path stat
  struct stat buf;
  if (stat(path.c_str(), &buf) != kStatSuccess) {
    return false;
  }

  // check
  if (S_ISDIR(buf.st_mode)) {
    return true;
  } else {
    return false;
  }
}

bool Utils::IsPathExist(const string &path) {
  ifstream file(path);
  if (!file) {
    return false;
  }
  return true;
}

void Utils::SplitPath(const string &path, vector<string> &path_vec) {
    char *char_path = const_cast<char*>(path.c_str());
    const char *char_split = kImagePathSeparator.c_str();
    char *tmp_path = strtok(char_path, char_split);
    while (tmp_path) {
        path_vec.emplace_back(tmp_path);
        tmp_path = strtok(nullptr, char_split);
    }
}

void Utils::GetAllFiles(const string &path, vector<string> &file_vec) {
    // split file path
    vector<string> path_vector;
    SplitPath(path, path_vector);

    for (string every_path : path_vector) {
        // check path exist or not
        if (!IsPathExist(path)) {
        ERROR_LOG("Failed to deal path=%s. Reason: not exist or can not access.",
                every_path.c_str());
        continue;
        }
        // get files in path and sub-path
        GetPathFiles(every_path, file_vec);
    }
}

void Utils::GetPathFiles(const string &path, vector<string> &file_vec) {
    struct dirent *dirent_ptr = nullptr;
    DIR *dir = nullptr;
    if (IsDirectory(path)) {
        dir = opendir(path.c_str());
        while ((dirent_ptr = readdir(dir)) != nullptr) {
            // skip . and ..
            if (dirent_ptr->d_name[0] == '.') {
            continue;
            }

            // file path
            string full_path = path + kPathSeparator + dirent_ptr->d_name;
            // directory need recursion
            if (IsDirectory(full_path)) {
                GetPathFiles(full_path, file_vec);
            } else {
                // put file
                file_vec.emplace_back(full_path);
            }
        }
    } 
    else {
        file_vec.emplace_back(path);
    }
}

void* Utils::CopyDataToDevice(void* data, uint32_t dataSize, aclrtMemcpyKind policy) {
    void* buffer = nullptr;
    aclError aclRet = aclrtMalloc(&buffer, dataSize, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return nullptr;
    }

    aclRet = aclrtMemcpy(buffer, dataSize, data, dataSize, policy);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
        (void)aclrtFree(buffer);
        return nullptr;
    }

    return buffer;
}

void* Utils::CopyDataDeviceToDevice(void* deviceData, uint32_t dataSize) {
    return CopyDataToDevice(deviceData, dataSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
}

void* Utils::CopyDataHostToDevice(void* deviceData, uint32_t dataSize) {
    return CopyDataToDevice(deviceData, dataSize, ACL_MEMCPY_HOST_TO_DEVICE);
}


void Utils::ImageNchw(shared_ptr<ImageDesc>& imageData, std::vector<cv::Mat>& nhwcImageChs, uint32_t size) {
    uint8_t* nchwBuf = new uint8_t[size];
    int channelSize = IMAGE_CHAN_SIZE_F32(nhwcImageChs[0].rows, nhwcImageChs[0].cols);
    int pos = 0;
    for (int i = 0; i < nhwcImageChs.size(); i++) {
        memcpy(static_cast<uint8_t *>(nchwBuf) + pos,  nhwcImageChs[i].ptr<float>(0), channelSize);
        pos += channelSize;
    }

    imageData->size = size;
    imageData->data.reset((uint8_t *)nchwBuf, [](uint8_t* p) { delete[](p);} );

}

bool Utils::PreProcess(shared_ptr<ImageDesc>& imageData, const string& imageFile) {

    //TODO:
    static auto it = resizeWidthHeight.begin();
    if (it == resizeWidthHeight.end()){
        it = resizeWidthHeight.begin();
    }


    //Read image using opencv
    cv::Mat image = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    if (image.empty()) {
        ERROR_LOG("Read image %s failed", imageFile.c_str());
        return false;
    }
    //resize image to model size
    cv::Mat reiszedImage, rsImageF32;
    cv::resize(image, reiszedImage, cv::Size(it->first, it->second));
    reiszedImage.convertTo(rsImageF32, CV_32FC3);


    //R G B channel means
    std::vector<cv::Mat> channels;
    cv::split(rsImageF32, channels);
    channels[0] -= kImageBChannelMean;
    channels[1] -= kImageGChannelMean;
    channels[2] -= kImageRChannelMean;


    //Transform NHWC to NCHW
    uint32_t size = RGB_IMAGE_SIZE_F32(it->first, it->second);
    ImageNchw(imageData,channels,size);

    aclError ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
    }

    void* imageDev;
    //copy image data to device
    if (runMode_ == ACL_HOST) {
        imageDev = CopyDataHostToDevice(imageData->data.get(), size);
        if (imageDev == nullptr) {
            ERROR_LOG("Copy image info to device failed");
            return FAILED;
        }
    }
    else {
        imageDev = CopyDataDeviceToDevice(imageData->data.get(), size);
        if (imageDev == nullptr) {
            ERROR_LOG("Copy image info to device failed");
            return FAILED;
        }
    }
    it++;
    imageData->size = size;
    imageData->data.reset((uint8_t *)imageDev,
    [](uint8_t* p) { aclrtFree(p);} );

    return true;
}


