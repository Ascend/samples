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

#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <unistd.h>

#include "gesture_detect.h"
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteError.h"
#include "acllite/AclLiteResource.h"
using namespace std;
int gSuccessNum = -4;
namespace {
const uint32_t kOpenPoseModelWidth = 128;
const uint32_t kOpenPoseModelHeight = 128;
const char* kOpenPoseModelPath = "../model/pose_deploy.om";
const char* kGestureModelPath = "../model/stgcn_fps30_sta_ho_ki4.om";
}

std::shared_ptr<EngineTransNewT> gMotionDataNew = std::make_shared<EngineTransNewT>();
int gImageNum = 0;

// Standardized bone key point sequence
AclLiteError ProcessOpenPoseData(){
    float tempLeft = 128;
    float tempRight = 0;
    float tempTop = 128;
    float tempBottom = 0;
    float totalLeft = 0;
    float totalRight = 0;
    float totalTop = 0;
    float totalBottom = 0;
    // The pixel length of human torso in the first five frames is calculated and normalized as the standard
    for (int picNum = FRAME_LENGTH-1; picNum > FRAME_LENGTH-6; picNum--){
        totalBottom +=  float(gMotionDataNew->data[0][1][picNum][8] + 
                        gMotionDataNew->data[0][1][picNum][11]) / 2 
                        - float(gMotionDataNew->data[0][1][picNum][1]);
    }

    totalBottom /= 5.0;
    for (int picNum = 0; picNum < FRAME_LENGTH; picNum++){
        for(int key_num = 0; key_num < 14; key_num++){
            gMotionDataNew->data[0][0][picNum][key_num] /= totalBottom;
            gMotionDataNew->data[0][1][picNum][key_num] /= totalBottom;
        }
    }
    return ACLLITE_OK;
}

// Save the data of input action recognition models
AclLiteError SaveData(){
    string fileName = "./data/" + to_string(gImageNum) + "_raw_data.txt";
    ofstream file;
    file.open(fileName.c_str(), ios::trunc);
    for (int jj = 0; jj < 2; jj++){
        for (int aa = 0; aa < FRAME_LENGTH; aa++){
            for (int qq = 0; qq < 14; qq++){
                file << gMotionDataNew->data[0][jj][aa][qq] << "\n";
            }
        }
    }
    file.close();
    return ACLLITE_OK;
}

int main(int argc, char *argv[]) {

    //init acl resource
    AclLiteResource aclDev;
    AclLiteError ret = aclDev.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }  
    
    GestureDetect detect(kOpenPoseModelPath, kGestureModelPath, 
                         kOpenPoseModelWidth, kOpenPoseModelHeight);
    ret = detect.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Classification Init resource failed");
        return 1;
    }

    //inference picture by picture
    ImageData image;
    for (gImageNum = 0; gImageNum < 50; gImageNum++) {
        string imageFile = "../data/" + to_string(gImageNum) + ".jpg";
        const char* tmp = imageFile.data();
        if((access(tmp, 0)) == -1){
            gImageNum--;
            usleep(40000);
            continue;
        }

        ret = ReadJpeg(image, imageFile);
        if (ret != ACLLITE_OK){
            continue;
        }
        if (image.data == nullptr) {
            ACLLITE_LOG_ERROR("Read image %s failed", imageFile.c_str());
            return 1;
        }
        //Preprocessing image: read the image and zoom it to the size required by the model input
        ImageData resizedImage;
        ret = detect.Preprocess(resizedImage, image);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next", imageFile.c_str());
            continue;
        }
        //The preprocessed images are sent to the openpose model for inference, and the openpose inference results are obtained
        std::vector<InferenceOutput> inferenceOutput;
        ret = detect.OpenPoseInference(inferenceOutput, resizedImage);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }

        // Analysis of openpose inference output
        ret = detect.Postprocess(image, inferenceOutput, gMotionDataNew, gSuccessNum);
        if (ret != ACLLITE_OK) {
            continue;
        }
        // Every five frames are updated for action recognition
        if (gSuccessNum % 5 == 0) {
            ProcessOpenPoseData();
            //SaveData();
            //The skeleton sequence of human body is sent into the structure model for inference, and the possibility of five actions is obtained
            std::vector<InferenceOutput> gestureOutput;

            ret = detect.GestureInference(gestureOutput, gMotionDataNew);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Inference model inference output data failed");
                return 1;
            }
            // post processing
            ret = detect.PostGestureProcess(gestureOutput);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Process model inference output data failed");
                break;
            }
        }
    }
    return 0;
}
