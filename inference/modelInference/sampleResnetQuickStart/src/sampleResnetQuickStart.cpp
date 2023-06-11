#include <opencv2/opencv.hpp>
#include <dirent.h>
#include "acl/acl.h"
#include "label.h"
#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);fflush(stdout)
#define ERROR_LOG(fmt, ...)fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)

using namespace cv;
using namespace std;
typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

namespace {
    const float min_chn_0 = 123.675;
    const float min_chn_1 = 116.28;
    const float min_chn_2 = 103.53;
    const float var_reci_chn_0 = 0.0171247538316637;
    const float var_reci_chn_1 = 0.0175070028011204;
    const float var_reci_chn_2 = 0.0174291938997821;
}

class SampleResnetQuickStart {
    public:
        SampleResnetQuickStart (int32_t device, const char* ModelPath,
        int32_t modelWidth, int32_t modelHeight);
        ~SampleResnetQuickStart();
        Result InitResource();
        Result ProcessInput(const string testImgPath);
        Result Inference();
        Result GetResult();
    private:
        void ReleaseResource();
        int32_t deviceId_;
        aclrtContext context_;
        aclrtStream stream_;

        uint32_t modelId_;
        const char* modelPath_;
        int32_t modelWidth_;
        int32_t modelHeight_;
        aclmdlDesc *modelDesc_;
        aclmdlDataset *inputDataset_;
        aclmdlDataset *outputDataset_;
        void *picDevBuffer_;
        size_t devBufferSize_;
        float* nchwImage_;
};

SampleResnetQuickStart::SampleResnetQuickStart(int32_t device, const char* modelPath,
                                               int32_t modelWidth, int32_t modelHeight) :
deviceId_(device), context_(nullptr), stream_(nullptr), modelId_(0),
modelPath_(modelPath), modelWidth_(modelWidth), modelHeight_(modelHeight),
modelDesc_(nullptr), inputDataset_(nullptr), outputDataset_(nullptr),
picDevBuffer_(nullptr), nchwImage_(nullptr)
{
}

SampleResnetQuickStart::~SampleResnetQuickStart()
{
    ReleaseResource();
}

Result SampleResnetQuickStart::InitResource()
{
    // init acl resource
    const char *aclConfigPath = "";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclInit failed, errorCode is %d", ret);
        return FAILED;
    }

    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSetDevice failed, errorCode is %d", ret);
        return FAILED;
    }

    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtCreateContext failed, errorCode is %d", ret);
        return FAILED;
    }

    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtCreateStream failed, errorCode is %d", ret);
        return FAILED;
    }

    // load model from file
    ret = aclmdlLoadFromFile(modelPath_, &modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclmdlLoadFromFile failed, errorCode is %d", ret);
        return FAILED;
    }

    // create description of model
    modelDesc_ = aclmdlCreateDesc();
    ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclmdlGetDesc failed, errorCode is %d", ret);
        return FAILED;
    }

    // create data set of input
    int inputIndex = 0;
    devBufferSize_ = aclmdlGetInputSizeByIndex(modelDesc_, inputIndex);
    aclrtMalloc(&picDevBuffer_, devBufferSize_, ACL_MEM_MALLOC_NORMAL_ONLY);
    inputDataset_ = aclmdlCreateDataset();
    aclDataBuffer *inputData = aclCreateDataBuffer(picDevBuffer_, devBufferSize_);
    ret = aclmdlAddDatasetBuffer(inputDataset_, inputData);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclmdlAddDatasetBuffer failed, errorCode is %d", ret);
        return FAILED;
    }

    // create data set of output
    outputDataset_ = aclmdlCreateDataset();
    size_t outputSize = aclmdlGetNumOutputs(modelDesc_);
    for (size_t i = 0; i < outputSize; ++i) {
        size_t modelOutputSize = aclmdlGetOutputSizeByIndex(modelDesc_, i);
        void *outputBuffer = nullptr;
        aclrtMalloc(&outputBuffer, modelOutputSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        aclDataBuffer *outputData = aclCreateDataBuffer(outputBuffer, modelOutputSize);
        ret = aclmdlAddDatasetBuffer(outputDataset_, outputData);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclmdlAddDatasetBuffer failed, errorCode is %d", ret);
            return FAILED;
        }
    }
    return SUCCESS;
}

Result SampleResnetQuickStart::ProcessInput(const string testImgPath)
{
    // read image from file
    Mat srcImage = imread(testImgPath);
    Mat resizedImage;

    // zoom image to modelWidth_ * modelHeight_
    resize(srcImage, resizedImage, Size(modelWidth_, modelHeight_));

    // get properties of image
    int32_t channel = resizedImage.channels();
    int32_t resizeHeight = resizedImage.rows;
    int32_t resizeWeight = resizedImage.cols;

    // create malloc of image, which is shape with NCHW
    nchwImage_ = (float*)malloc(channel * resizeHeight * resizeWeight * sizeof(float));
    memset(nchwImage_, 0, channel * resizeHeight * resizeWeight * sizeof(float));
    float meanRgb[3] = {min_chn_2, min_chn_1, min_chn_0};
    float stdRgb[3]  = {var_reci_chn_2, var_reci_chn_1, var_reci_chn_0};
    uint8_t bgrToRgb=2;

    // image with shape NHWC change to NCHW, and switch channel BGR to RGB
    for (int c = 0; c < channel; ++c)
    {
        for (int h = 0; h < resizeHeight; ++h)
        {
            for (int w = 0; w < resizeWeight; ++w)
            {
                int dstIdx = (bgrToRgb - c) * resizeHeight * resizeWeight + h * resizeWeight + w;
                nchwImage_[dstIdx] =  static_cast<float>((resizedImage.at<cv::Vec3b>(h, w)[c] -
                                                         1.0f*meanRgb[c]) * 1.0f*stdRgb[c] );
            }
        }
    }
    return SUCCESS;
}

Result SampleResnetQuickStart::Inference()
{
    // copy image from imageBuffer_ to input data buffer
    aclError ret = aclrtMemcpy(picDevBuffer_, devBufferSize_, nchwImage_, devBufferSize_, ACL_MEMCPY_HOST_TO_DEVICE);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("memcpy  failed, errorCode is %d", ret);
        return FAILED;
    }

    // inference
    ret = aclmdlExecute(modelId_, inputDataset_, outputDataset_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleResnetQuickStart::GetResult() {
    // get result from output data set
    void *outHostData = nullptr;
    float *outData = nullptr;
    size_t outputIndex = 0;
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(outputDataset_, outputIndex);
    void* data = aclGetDataBufferAddr(dataBuffer);
    uint32_t len = aclGetDataBufferSizeV2(dataBuffer);

    aclrtMallocHost(&outHostData, len);
    aclError ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("memcpy  failed, errorCode is %d", ret);
        return FAILED;
    }
    outData = reinterpret_cast<float*>(outHostData);

    map<float, unsigned int, greater<float> > resultMap;
    for (unsigned int j = 0; j < len / sizeof(float); ++j) {
        resultMap[*outData] = j;
        outData++;
    }

    // do data processing with softmax and print top 5 classes
    double totalValue=0.0;
    for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
        totalValue += exp(it->first);
    }

    int cnt = 0;
    for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
        // print top 5
        if (++cnt > 5) {
            break;
        }
        INFO_LOG("top %d: index[%d] value[%lf] class[%s]", cnt, it->second,
                 exp(it->first) / totalValue, label[it->second].c_str());
    }
    ret = aclrtFreeHost(outHostData);
    outHostData = nullptr;
    outData = nullptr;
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtFreeHost failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

void SampleResnetQuickStart::ReleaseResource()
{
    aclError ret;
    free(nchwImage_);
    nchwImage_ = nullptr;
    picDevBuffer_ = nullptr;
    // release resource includes acl resource, data set and unload model
    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(inputDataset_); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(inputDataset_, i);
        (void)aclDestroyDataBuffer(dataBuffer);
    }
    (void)aclmdlDestroyDataset(inputDataset_);
    inputDataset_ = nullptr;

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(outputDataset_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(outputDataset_, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
    }
    (void)aclmdlDestroyDataset(outputDataset_);
    outputDataset_ = nullptr;

    ret = aclmdlDestroyDesc(modelDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("destroy description failed, errorCode is %d", ret);
    }

    ret = aclmdlUnload(modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("unload model failed, errorCode is %d", ret);
    }

    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtDestroyStream failed, errorCode is %d", ret);
        }
        stream_ = nullptr;
    }

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtDestroyContext failed, errorCode is %d", ret);
        }
        context_ = nullptr;
    }

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtResetDevice failed, errorCode is %d", ret);
    }

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclFinalize failed, errorCode is %d", ret);
    }
}

int main()
{
    const char* modelPath = "../model/resnet50.om";
    const string imagePath = "../data";
    int32_t device = 0;
    int32_t modelWidth = 224;
    int32_t modelHeight = 224;

    // all images in dir
    DIR *dir = opendir(imagePath.c_str());
    if (dir == nullptr)
    {
        ERROR_LOG("file folder does no exist, please create folder %s", imagePath.c_str());
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
            string imgDir = imagePath + "/" + name;
            allPath.push_back(imgDir);
        }
    }
    closedir(dir);

    if (allPath.size() == 0){
        ERROR_LOG("the directory is empty, please download image to %s", imagePath.c_str());
        return FAILED;
    }

    string fileName;
    SampleResnetQuickStart sampleResnet(device, modelPath, modelWidth, modelHeight);
    Result ret = sampleResnet.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("InitResource  failed");
        return FAILED;
    }

    for (size_t i = 0; i < allPath.size(); i++)
    {
        fileName = allPath.at(i).c_str();
        ret = sampleResnet.ProcessInput(fileName);
        if (ret != SUCCESS) {
            ERROR_LOG("ProcessInput  failed");
            return FAILED;
        }

        ret = sampleResnet.Inference();
        if (ret != SUCCESS) {
            ERROR_LOG("Inference  failed");
            return FAILED;
        }

        ret = sampleResnet.GetResult();
        if (ret != SUCCESS) {
            ERROR_LOG("GetResult  failed");
            return FAILED;
        }
    }
    return SUCCESS;
}