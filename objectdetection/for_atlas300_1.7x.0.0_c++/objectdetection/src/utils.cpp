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

using namespace std;

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

enum BBoxIndex {TOPLEFTX=0,TOPLEFTY,BOTTOMRIGHTX,BOTTOMRIGHTY,SCORE,LABEL};

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
	uint32_t count;
	aclError ret = aclrtMemcpy(&count, sizeof(count), data, sizeof(count), ACL_MEMCPY_DEVICE_TO_DEVICE);
	if (ret != ACL_ERROR_NONE) {
	  ERROR_LOG("box count aclrtMemcpy failed!");
	  return FAILED;
	}
	INFO_LOG("box count=%d",count);
	dataBuffer = aclmdlGetDatasetBuffer(modelOutput, 0);
	if (dataBuffer == nullptr) {
		ERROR_LOG("get model output aclmdlGetDatasetBuffer failed");
		return FAILED;
	}
	data = aclGetDataBufferAddr(dataBuffer);
	if (data == nullptr) {
		ERROR_LOG("aclGetDataBufferAddr from dataBuffer failed.");
	}
	float outInfo[count*6];
	ret = aclrtMemcpy(&outInfo, sizeof(outInfo), data, sizeof(outInfo), ACL_MEMCPY_DEVICE_TO_DEVICE);
	if (ret != ACL_ERROR_NONE) {
	  ERROR_LOG("box outInfo aclrtMemcpy failed!");
	  return FAILED;
	}
	cv::Rect rect;
	int font_face = 0; 
	double font_scale = 1;
	int thickness = 2;
	int baseline;
	cv::Mat resultImage = cv::imread(path, CV_LOAD_IMAGE_COLOR);
	for(uint32_t b=0;b<count;b++){
		uint32_t score=uint32_t(outInfo[count*SCORE+b]*100);
		if(score<90)continue;
		rect.x=outInfo[count*TOPLEFTX+b]*resultImage.cols/MODEL_INPUT_WIDTH;
		rect.y=outInfo[count*TOPLEFTY+b]*resultImage.rows/MODEL_INPUT_HEIGHT;
		rect.width=outInfo[count*BOTTOMRIGHTX+b]*resultImage.cols/MODEL_INPUT_WIDTH-rect.x;
		rect.height=outInfo[count*BOTTOMRIGHTY+b]*resultImage.rows/MODEL_INPUT_HEIGHT-rect.y;
		uint32_t objIndex = (uint32_t)outInfo[count*LABEL+b];
		string text = yolov3Label[objIndex]+std::to_string(score)+"\%";
      	cv::Point origin; 
      	origin.x = rect.x;
      	origin.y = rect.y;
      	cv::putText(resultImage, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 4, 0);
		cv::rectangle(resultImage, rect, cv::Scalar(0, 255, 255),1, 8,0);
	}
	// generate result image
    int pos = path.find_last_of(kFileSperator);
    string file_name(path.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output" << kFileSperator
      << kOutputFilePrefix << file_name;

    string outputPath = sstream.str();
    cv::imwrite(outputPath, resultImage);

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

bool Utils::PreProcess(shared_ptr<ImageDesc> &image_path,
                                    const string &path) {
    // read image using OPENCV
    cv::Mat mat = cv::imread(path, CV_LOAD_IMAGE_COLOR);
    //resize
    cv::Mat mat_rs;
    cv::resize(mat, mat_rs, cv::Size(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT));
 
    if (mat.empty()) {
        ERROR_LOG("read image failed.");
        return false;
    }

    // set property
    image_path->input_path = path;
    image_path->img_width = mat_rs.cols;
    image_path->img_height = mat_rs.rows;

    // set image data
    uint32_t size = mat_rs.rows * mat_rs.cols*3;
    void *image_buf_ptr = nullptr;
    aclError mem_ret = aclrtMalloc(&image_buf_ptr, (size_t)(size), ACL_MEM_MALLOC_HUGE_FIRST);
  
    if (image_buf_ptr == nullptr) {
        ERROR_LOG("new image buffer failed.");
        return false;
    }
    mem_ret = aclrtMemcpy(image_buf_ptr, size, mat_rs.ptr<u_int8_t>(), size, ACL_MEMCPY_DEVICE_TO_DEVICE);

    if (mem_ret != ACL_ERROR_NONE) {
        aclrtFree(image_buf_ptr);
        delete[] (u_int8_t *)image_buf_ptr;
        ERROR_LOG("memcpy_s failed.");
        image_buf_ptr = nullptr;
        return false;
    }
    image_path->size = size;
    image_path->data.reset((u_int8_t *)image_buf_ptr,[](u_int8_t* p){aclrtFree(p);});
    return true;
}
