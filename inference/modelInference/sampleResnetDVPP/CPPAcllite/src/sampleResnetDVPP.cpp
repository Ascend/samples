#include <cmath>
#include <dirent.h>
#include <string.h>
#include "AclLiteUtils.h"
#include "AclLiteImageProc.h"
#include "AclLiteResource.h"
#include "AclLiteError.h"
#include "AclLiteModel.h"
#include "label.h"

using namespace std;
typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

class SampleResnetDVPP {
    public:
    SampleResnetDVPP(const char *modelPath, const int32_t modelWidth, const int32_t modelHeight);
    ~SampleResnetDVPP();
    Result InitResource();
    Result ProcessInput(const string testImgPath);
    Result Inference(std::vector<InferenceOutput>& inferOutputs);
    Result GetResult(std::vector<InferenceOutput>& inferOutputs);
    private:
    void ReleaseResource();
    AclLiteResource aclResource_;
    AclLiteImageProc imageProcess_;
    AclLiteModel model_;
    aclrtRunMode runMode_;
    ImageData resizedImage_;
    const char *modelPath_;
    int32_t modelWidth_;
    int32_t modelHeight_;
};

SampleResnetDVPP::SampleResnetDVPP(const char *modelPath, 
                                   const int32_t modelWidth,
                                   const int32_t modelHeight) :
modelPath_(modelPath), modelWidth_(modelWidth), modelHeight_(modelHeight)
{
}

SampleResnetDVPP::~SampleResnetDVPP()
{
    ReleaseResource();
}

Result SampleResnetDVPP::InitResource()
{
    // init acl resource
    AclLiteError ret = aclResource_.Init();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("resource init failed, errorCode is %d", ret);
        return FAILED;
    }

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("get runMode failed, errorCode is %d", ret);
        return FAILED;
    }

    // init dvpp resource
    ret = imageProcess_.Init();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("imageProcess init failed, errorCode is %d", ret);
        return FAILED;
    }

    // load model from file
    ret = model_.Init(modelPath_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("model init failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleResnetDVPP::ProcessInput(const string testImgPath)
{
    // read image from file
    ImageData image;
    AclLiteError ret = ReadJpeg(image, testImgPath);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("ReadJpeg failed, errorCode is %d", ret);
        return FAILED;
    }

    // copy image from host to dvpp 
    ImageData imageDevice;
    ret = CopyImageToDevice(imageDevice, image, runMode_, MEMORY_DVPP);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("CopyImageToDevice failed, errorCode is %d", ret);
        return FAILED;
    }

    // image decoded from JPEG format to YUV
    ImageData yuvImage;
    ret = imageProcess_.JpegD(yuvImage, imageDevice);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed, errorCode is %d", ret);
        return FAILED;
    }

    // zoom image to modelWidth_ * modelHeight_
    ret = imageProcess_.Resize(resizedImage_, yuvImage, modelWidth_, modelHeight_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Resize image failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleResnetDVPP::Inference(std::vector<InferenceOutput>& inferOutputs)
{
    // create input data set of model
    AclLiteError ret = model_.CreateInput(static_cast<void *>(resizedImage_.data.get()), resizedImage_.size);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("CreateInput failed, errorCode is %d", ret);
        return FAILED;
    }

    // inference
    ret = model_.Execute(inferOutputs);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("execute model failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleResnetDVPP::GetResult(std::vector<InferenceOutput>& inferOutputs) {
    // get result from output data set
    uint32_t outputDataBufId = 0;
    uint32_t dataSize = inferOutputs[outputDataBufId].size;
    float* outData = static_cast<float*>(inferOutputs[outputDataBufId].data.get());
    if (outData == nullptr) {
        return FAILED;
    }
    map<float, unsigned int, greater<float> > resultMap;
    for (uint32_t j = 0; j < dataSize / sizeof(float); ++j) {
        resultMap[*outData] = j;
        outData++;
    }

    // do data processing with softmax and print top 5 classes
    uint32_t topConfidenceLevels = 5;
    double totalValue = 0.0;
    for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
        totalValue += exp(it->first);
    }

    int cnt = 0;
    for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
        // print top 5
        if (++cnt > topConfidenceLevels) {
            break;
        }
        ACLLITE_LOG_INFO("top %d: index[%d] value[%lf] class[%s]", cnt, it->second,
                         exp(it->first) / totalValue, label[it->second].c_str());
    }
    outData = nullptr;
    return SUCCESS;
}

void SampleResnetDVPP::ReleaseResource()
{
    model_.DestroyResource();
    imageProcess_.DestroyResource();
    aclResource_.Release();
}

int main()
{
    const char* modelPath = "../model/resnet50.om";
    const string imagePath = "../data";
    const int32_t modelWidth = 224;
    const int32_t modelHeight = 224;
    DIR *dir = opendir(imagePath.c_str());
    if (dir == nullptr)
    {
        ACLLITE_LOG_ERROR("file folder does no exist, please create folder %s", imagePath.c_str());
        return FAILED;
    }
    vector<string> allPath;
    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0
        || strcmp(entry->d_name, ".keep") == 0)
        {
            continue;
        }else{
            string name = entry->d_name;
            string imgDir = imagePath +"/"+ name;
            allPath.push_back(imgDir);
        }
    }
    closedir(dir);

    if (allPath.size() == 0){
        ACLLITE_LOG_ERROR("the directory is empty, please download image to %s", imagePath.c_str());
        return FAILED;
    }

    string fileName;
    SampleResnetDVPP sampleResnet(modelPath, modelWidth, modelHeight);
    Result ret = sampleResnet.InitResource();
    if (ret != SUCCESS) {
        ACLLITE_LOG_ERROR("InitResource failed");
        return FAILED;
    }

    for (size_t i = 0; i < allPath.size(); i++)
    {
        fileName =  allPath.at(i).c_str();
        std::vector<InferenceOutput> inferOutputs;
        ret = sampleResnet.ProcessInput(fileName);
        if (ret != SUCCESS) {
            ACLLITE_LOG_ERROR("ProcessInput image failed");
            return FAILED;
        }

        ret = sampleResnet.Inference(inferOutputs);
        if (ret != SUCCESS) {
            ACLLITE_LOG_ERROR("Inference failed");
            return FAILED;
        }

        ret = sampleResnet.GetResult(inferOutputs);
        if (ret != SUCCESS) {
            ACLLITE_LOG_ERROR("GetResult failed");
            return FAILED;
        }
    }
    return SUCCESS;
}