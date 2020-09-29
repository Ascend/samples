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

* File utils.h
* Description: handle file operations
*/
#pragma once
#include <iostream>
#include <vector>
#include "acl/acl.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/imgproc/types_c.h"

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR]  " fmt "\n", ##args)

#define MODEL_INPUT_WIDTH	300
#define MODEL_INPUT_HEIGHT	300
#define RGB_IMAGE_SIZE_F32(width, height) ((width) * (height) * 3 * 4)
#define IMAGE_CHAN_SIZE_F32(width, height) ((width) * (height) * 4)
#define IMAGE_CHAN_SIZE_U8(width, height) ((width) * (height) * 3)

using namespace std;

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef struct PicDesc {
    std::string picName;
    int width;
    int height;
}PicDesc;

struct ImageDesc {
  uint32_t img_width = 0;
  uint32_t img_height = 0;
  int32_t size = 0;
  std::string input_path = "";
  std::shared_ptr<u_int8_t> data;
};

const std::string kImagePathSeparator = ",";
const int kStatSuccess = 0;
const std::string kFileSperator = "/";
const std::string kPathSeparator = "/";
// output image prefix
const std::string kOutputFilePrefix = "out_";

/**
 * Utils
 */
class Utils {
public:

    static aclrtRunMode runMode_;
    /**
    * @brief create device buffer of pic
    * @param [in] picDesc: pic desc
    * @param [in] PicBufferSize: aligned pic size
    * @return device buffer of pic
    */

    static Result PostProcess(const std::string &path, aclmdlDataset *modelOutput);

    static bool IsDirectory(const std::string &path);

    static bool IsPathExist(const std::string &path);

    static void SplitPath(const std::string &path, std::vector<std::string> &path_vec);

    static void GetAllFiles(const std::string &path, std::vector<std::string> &file_vec);

    static void GetPathFiles(const std::string &path, std::vector<std::string> &file_vec);

    static bool PreProcess(shared_ptr<ImageDesc>& imageData, const string& imageFile);

    static void* CopyDataToDevice(void* data, uint32_t dataSize, aclrtMemcpyKind policy);

    static void* CopyDataHostToDevice(void* deviceData, uint32_t dataSize);

    static void* CopyDataDeviceToDevice(void* deviceData, uint32_t dataSize);

    static void ImageNchw(shared_ptr<ImageDesc>& imageData, std::vector<cv::Mat>& nhwcImageChs, uint32_t size);
};

