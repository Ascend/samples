#include <cmath>
#include <dirent.h>
#include <string.h>
#include "AclLiteUtils.h"
#include "AclLiteImageProc.h"
#include "AclLiteVideoProc.h"
#include "AclLiteResource.h"
#include "AclLiteError.h"
#include "AclLiteModel.h"
#include "label.h"

using namespace std;
typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

class SampleResnetRtsp {
    public:
    SampleResnetRtsp(const string modelPath, const uint32_t modelWidth, const uint32_t modelHeight);
    ~SampleResnetRtsp();
    Result InitResource(const string inputDataPath);
    Result ProcessInput(ImageData &image);
    Result Inference(std::vector<InferenceOutput> &inferOutputs);
    Result GetResult(std::vector<InferenceOutput> &inferOutputs);
                    
    private:
    void ReleaseResource();
    const uint32_t modelWidth_;
    const int32_t modelHeight_;
    AclLiteModel model_;
    AclLiteImageProc imageProcess_;
    AclLiteResource aclResource_;
    ImageData resizedImage_;
};

SampleResnetRtsp::SampleResnetRtsp(const string modelPath, 
                                   const uint32_t modelWidth, 
                                   const uint32_t modelHeight):
model_(modelPath), modelWidth_(modelWidth), modelHeight_(modelHeight)
{
}

SampleResnetRtsp::~SampleResnetRtsp() {
    ReleaseResource();
}

Result SampleResnetRtsp::InitResource(const string inputDataPath) 
{
     // init acl resource
    AclLiteError atlRet = aclResource_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", atlRet);
        return FAILED;
    }

    // init dvpp resource
    atlRet = imageProcess_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", atlRet);
        return FAILED;
    }

    // load model 
    atlRet = model_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", atlRet);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleResnetRtsp::ProcessInput(ImageData &image) 
{
    // zoom image to modelWidth_ * modelHeight_
    AclLiteError ret = imageProcess_.Resize(resizedImage_, image, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return SUCCESS;
    }
}

Result SampleResnetRtsp::Inference(std::vector<InferenceOutput> &inferOutputs) 
{
    AclLiteError ret = model_.CreateInput(resizedImage_.data.get(), resizedImage_.size);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed\n");
        return FAILED;
    }

    // inference
    ret = model_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed\n");
    }
    return SUCCESS;
}

Result SampleResnetRtsp::GetResult(std::vector<InferenceOutput> &inferOutputs) 
{
    // get result from output data set
    uint32_t outputDataBufId = 0;
    uint32_t dataSize = inferOutputs[outputDataBufId].size;
    float* outData = static_cast<float*>(inferOutputs[outputDataBufId].data.get());
    if (outData == nullptr) {
        printf("outData == nullptr");
    }

    // do data processing with softmax and print top 1 class;
    map<float, unsigned int, greater<float> > resultMap;
    for (uint32_t j = 0; j < dataSize / sizeof(float); ++j) {
        resultMap[*outData] = j; 
        outData++;
    }

    double totalValue = 0.0;
    for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
        totalValue += exp(it->first);
    }

    int cnt = 1;
    ACLLITE_LOG_INFO("top %d: index[%d] value[%lf] class[%s]", cnt, resultMap.begin()->second,
                     exp(resultMap.begin()->first) / totalValue, label[resultMap.begin()->second].c_str());
    outData = nullptr;
    return SUCCESS;
}

void SampleResnetRtsp::ReleaseResource() {
    model_.DestroyResource(); 
    imageProcess_.DestroyResource();
    aclResource_.Release();
}

int main(int argc, char *argv[]) 
{
    int argNum = 2;
    if ((argc < argNum)) {
        ACLLITE_LOG_ERROR("Please input: ./main rtsp address");
        return FAILED;
    }

    const string modelPath = "../model/resnet50.om";
    const uint32_t modelWidth = 224;
    const uint32_t modelHeight = 224;
    std::string inputDataPath =  string(argv[1]);

    SampleResnetRtsp sampleResnetRtsp(modelPath, modelWidth, modelHeight);
    Result  ret = sampleResnetRtsp.InitResource(inputDataPath);
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return FAILED;
    }

    // open camera
    int device = 0;
    AclLiteVideoProc cap = AclLiteVideoProc(inputDataPath, device);
    if (!cap.IsOpened()) {
        ACLLITE_LOG_ERROR("Open camera failed");
        return FAILED;
    }
   
    // read frame from camera and inference
    while (true) {
        std::vector<InferenceOutput> inferOutputs;
        ImageData image;
        AclLiteError ret = cap.Read(image);
        if (ret) {
            break;
        }

        ret = sampleResnetRtsp.ProcessInput(image);
        if (ret) {
            ACLLITE_LOG_ERROR("Inference image failed, error %d",  ret);
            return FAILED;
        }

        ret = sampleResnetRtsp.Inference(inferOutputs);
        if (ret) {
            ACLLITE_LOG_ERROR("Inference image failed");
            return FAILED;        
        }

        sampleResnetRtsp.GetResult(inferOutputs);
    }

    ACLLITE_LOG_INFO("Execute sample success");
    return SUCCESS;
}
