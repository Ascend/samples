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

#include <iostream>
#include <stdlib.h>
#include <dirent.h>

#include "classify_process.h"
#include "utils.h"
#include "ctime"
using namespace std;

namespace {
uint32_t kModelWidth = 256;
uint32_t kModelHeight = 256;
//const char* kModelPath = "../model/frozen_graph_noDWT_V2_20200916.om";
const char* kModelPath = "../model/DeRain.om";
//const char* kModelPath = "../model/model3_20201006.om";
}

int main(int argc, char *argv[]) {
    //检查应用程序执行时的输入,程序执行要求输入图片目录参数
//    if((argc < 2) || (argv[1] == nullptr)){
//        ERROR_LOG("Please input: ./main <image_dir>");
//        return FAILED;
//    }
    //实例化Derain推理对象,参数为Derain模型路径,模型输入要求的宽和高
    ClassifyProcess classify(kModelPath, kModelWidth, kModelHeight);
    //初始化分类推理的acl资源, 模型和内存
    Result ret = classify.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }
    //获取图片目录下所有的图片文件名
    //string inputImageDir = string(argv[1]);
    string inputImageDir = string("../data");
    vector<string> fileVec;
    Utils::GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ERROR_LOG("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return FAILED;
    }
    //逐张图片推理
    for (string imageFile : fileVec) {
        //add by cz,20200928
        int posofPoint = imageFile.find_last_of(".");
        int posofUnderline = imageFile.find_last_of("_");
        if(posofPoint-posofUnderline-1>0){
            string PicType(imageFile.substr(posofUnderline + 1,posofPoint-posofUnderline-1));
            //cout << "PicType:" << PicType << endl;
            if(PicType == "GT"){
                continue;
            }
        }

        clock_t Begin,End;
        Begin = clock();
        //预处理图片:读取图片,讲图片缩放到模型输入要求的尺寸
        Result ret = classify.Preprocess(imageFile);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }

        //将预处理的图片送入模型推理,并获取推理结果
        aclmdlDataset* inferenceOutput = nullptr;
        ret = classify.Inference(inferenceOutput);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }

        //解析推理输出,并将推理得到的物体类别标记到图片上
        ret = classify.Postprocess(imageFile, inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
        End = clock();
        double duration = (End-Begin) * 1000.0 / CLOCKS_PER_SEC;
        cout << "tick:" << End-Begin << endl;
        cout << CLOCKS_PER_SEC << endl;
        cout << "duration:" << duration << "(ms)" << endl;
    }
    classify.PrintMeanPSNR();
    INFO_LOG("Execute sample success");
    return SUCCESS;
}
