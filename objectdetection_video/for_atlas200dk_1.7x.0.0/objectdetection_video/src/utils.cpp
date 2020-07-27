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
#include "acl/ops/acl_dvpp.h"
#include <ctime>

using namespace std;
using namespace ascend::presenter;

const static std::vector<std::string> yolov3Label = {"person", "bicycle", "car", "motorbike",
"aeroplane","bus", "train", "truck", "boat", 
"traffic light", "fire hydrant", "stop sign", "parking meter", 
"bench", "bird", "cat", "dog", "horse", 
"sheep", "cow", "elephant", "bear", "zebra", 
"giraffe", "backpack", "umbrella", "handbag","tie", 
"suitcase", "frisbee", "skis", "snowboard", "sports ball",
"kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
"tennis racket", "bottle", "wine glass", "cup", 
"fork", "knife", "spoon", "bowl", "banana", 
"apple", "sandwich", "orange", "broccoli", "carrot", 
"hot dog", "pizza", "donut", "cake", "chair", 
"sofa", "potted plant", "bed", "dining table", "toilet", 
"TV monitor", "laptop", "mouse", "remote", "keyboard", 
"cell phone", "microwave", "oven", "toaster", "sink", 
"refrigerator", "book", "clock", "vase","scissors", 
"teddy bear", "hair drier", "toothbrush"};

enum BBoxIndex {TOPLEFTX=0,TOPLEFTY,BOTTOMRIGHTX,BOTTOMRIGHTY,SCORE,LABEL,BBOX_MAX};

Result Utils::PresentOutputFrame(Channel *channel, aclmdlDataset *modelOutput, cv::Mat mat)
{
    size_t outDatasetNum = aclmdlGetDatasetNumBuffers(modelOutput);
	if (outDatasetNum != 2) {
	    ERROR_LOG("outDatasetNum=%zu failed,must be 2",outDatasetNum);
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
	uint32_t count;
	aclError ret = aclrtMemcpy(&count, sizeof(count), data, sizeof(count), ACL_MEMCPY_DEVICE_TO_DEVICE);
	if (ret != ACL_ERROR_NONE) {
	    ERROR_LOG("box count aclrtMemcpy failed!");
	    return FAILED;
	}
	dataBuffer = aclmdlGetDatasetBuffer(modelOutput, 0);
	if (dataBuffer == nullptr) {
	    ERROR_LOG("get model output aclmdlGetDatasetBuffer failed");
	    return FAILED;
	}
	data = aclGetDataBufferAddr(dataBuffer);
	if (data == nullptr) {
	    ERROR_LOG("aclGetDataBufferAddr from dataBuffer failed.");
	    return FAILED;
	}
	float outInfo[count*BBOX_MAX+1];
	if(count > 0){
	    ret = aclrtMemcpy(&outInfo, sizeof(outInfo), data, sizeof(outInfo), ACL_MEMCPY_DEVICE_TO_DEVICE);
	    if (ret != ACL_ERROR_NONE) {
	        ERROR_LOG("box info aclrtMemcpy failed!");
		return FAILED;
	    }
	}
	cv::Mat resultImage = mat;
	std::vector<DetectionResult> drs;
	for(uint32_t b=0;b<count;b++){
	    DetectionResult dr;
            uint32_t score=uint32_t(outInfo[count*SCORE+b]*100);
	    if(score<90)continue;
	    dr.lt.x=outInfo[count*TOPLEFTX+b]*mat.cols/MODEL_INPUT_WIDTH;
	    dr.lt.y=outInfo[count*TOPLEFTY+b]*mat.rows/MODEL_INPUT_HEIGHT;
	    dr.rb.x=outInfo[count*BOTTOMRIGHTX+b]*mat.cols/MODEL_INPUT_WIDTH;
	    dr.rb.y=outInfo[count*BOTTOMRIGHTY+b]*mat.rows/MODEL_INPUT_HEIGHT;
	    uint32_t objIndex = (uint32_t)outInfo[count*LABEL+b];
	    dr.result_text = yolov3Label[objIndex]+std::to_string(score)+"\%";
	    drs.push_back(dr);
	}
	
	vector<unsigned char> img_encode;
	vector<int> param= vector<int>(2); 
	param[0]=CV_IMWRITE_JPEG_QUALITY;
	param[1]=95;//default(95) 0-100
	
	cv::imencode(".jpg", mat, img_encode, param);
	
	ImageFrame image_frame_para;
	image_frame_para.format = ImageFormat::kJpeg;
	image_frame_para.width = mat.cols;
	image_frame_para.height = mat.rows;
	image_frame_para.size = img_encode.size();
	image_frame_para.data = reinterpret_cast<unsigned char *>(img_encode.data());
	image_frame_para.detection_results = drs;

	PresenterErrorCode errorCode = PresentImage(channel,image_frame_para);
	if (errorCode != PresenterErrorCode::kNone) {
		ERROR_LOG("PresentImage failed %d",static_cast<int>(errorCode));
	}
    return SUCCESS;
}

Result Utils::SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize)
{
    void* hostPtr = nullptr;
    aclError aclRet = aclrtMalloc(&hostPtr, dataSize, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc host data buffer failed, aclRet is %d", aclRet);
        return FAILED;
    }

    aclRet = aclrtMemcpy(hostPtr, dataSize, devPtr, dataSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("dvpp output memcpy to host failed, aclRet is %d", aclRet);
        (void)aclrtFree(hostPtr);
        return FAILED;
    }

    FILE *outFileFp = fopen(fileName, "wb+");
    if (nullptr == outFileFp) {
        ERROR_LOG("fopen out file %s failed.", fileName);
        (void)aclrtFree(hostPtr);
        return FAILED;
    }

    size_t writeSize = fwrite(hostPtr, sizeof(char), dataSize, outFileFp);
    if (writeSize != dataSize) {
        ERROR_LOG("need write %u bytes to %s, but only write %zu bytes.",
            dataSize, fileName, writeSize);
        (void)aclrtFree(hostPtr);
        return FAILED;
    }

    (void)aclrtFree(hostPtr);
    fflush(outFileFp);
    fclose(outFileFp);
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

