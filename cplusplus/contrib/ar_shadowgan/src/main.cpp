#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include <string>
#include <fstream>
#include <cstring>
#include "acl/acl.h"
#include <sys/stat.h>

#include "process.h"
#include "utils.h"
using namespace std;

namespace {
uint32_t kModelWidth = 256;
uint32_t kModelHeight = 256;
const char* kModelPath = "../model/model.om";
}

int main(int argc, char *argv[]) {
//    检查应用程序执行时的输入,程序执行要求输入图片目录参数
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
//    获取图片目录下所有的图片文件名
    string inputImageDir = string(argv[1]);
//    string inputImageDir = string("../data");
    vector<string> fileVec;
    string inputimage = inputImageDir + "/test_A.png.bin";
    string maskimage = inputImageDir + "/test_B.png.bin";

    //实例化分类推理对象,参数为分类模型路径,模型输入要求的宽和高
    Process process(kModelPath, kModelWidth, kModelHeight);
    //初始化分类推理的acl资源, 模型和内存
    Result ret = process.Init(inputimage, maskimage);
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }

    //将预处理的图片送入模型推理,并获取推理结果
    aclmdlDataset* inferenceOutput = nullptr;
    ret = process.Inference(inferenceOutput);
    if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
        ERROR_LOG("Inference model inference output data failed");
        return FAILED;
    }

    //解析推理输出,并将推理结果保存
    ret = process.Postprocess(inferenceOutput);
    

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
