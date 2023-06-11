#include <fstream>
#include <dirent.h>
#include <vector>
#include <string.h>
#include <map>
#include <cmath>
#include <memory>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "label.h"
#define SHARED_PTR_DVPP_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { acldvppFree(p); }))
#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);fflush(stdout)
#define ERROR_LOG(fmt, ...)fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
#define YUV420SP_SIZE(width, height) ((width) * (height) * 3 / 2)
#define ALIGN_UP(num, align) (((num) + (align) - 1) & ~((align) - 1))
#define ALIGN_UP2(num) ALIGN_UP(num, 2)
#define ALIGN_UP16(num) ALIGN_UP(num, 16)
#define ALIGN_UP64(num) ALIGN_UP(num, 64)
#define ALIGN_UP128(num) ALIGN_UP(num, 128)

using namespace std;
typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

struct ImageData {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t alignWidth = 0;
    uint32_t alignHeight = 0;
    uint32_t size = 0;
    std::shared_ptr<uint8_t> data;
};

class SampleResnetDVPP {
    public:
    SampleResnetDVPP(int32_t device, const char* modelPath,
    const int32_t modelWidth, const int32_t modelHeight);
    ~SampleResnetDVPP();
    Result InitResource();
    Result ProcessInput(const string testImgPath);
    Result Inference();
    Result GetResult();
    private:
    void ReleaseResource();
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;

    acldvppChannelDesc *dvppChannelDesc_; // dvpp channel
    acldvppPicDesc *decodeOutputDesc_; // jpg decode output desc
    void* decodeOutBufferDev_; // jpg decode output buffer
    acldvppPicDesc *vpcInputDesc_; // resize input desc
    acldvppPicDesc *vpcOutputDesc_; // resize output desc
    acldvppResizeConfig *resizeConfig_;

    uint32_t modelId_;
    const char* modelPath_;
    aclmdlDesc *modelDesc_;
    int32_t modelWidth_;
    int32_t modelHeight_;

    aclmdlDataset *inputDataset_;
    aclmdlDataset *outputDataset_;
    size_t devBufferSize_;
    void *picDevBuffer_;
};

SampleResnetDVPP::SampleResnetDVPP(int32_t device, const char* modelPath,
                                   const int32_t modelWidth, const int32_t modelHeight) :
                                   deviceId_(device), dvppChannelDesc_(nullptr), decodeOutputDesc_(nullptr),
                                   decodeOutBufferDev_(nullptr), vpcInputDesc_(nullptr), vpcOutputDesc_(nullptr),
                                   resizeConfig_(nullptr), context_(nullptr),
                                   stream_(nullptr), modelId_(0), modelDesc_(nullptr),
                                   inputDataset_(nullptr), outputDataset_(nullptr),
                                   picDevBuffer_(nullptr), modelPath_(modelPath),
                                   modelWidth_(modelWidth), modelHeight_(modelHeight)
{
}

SampleResnetDVPP::~SampleResnetDVPP()
{
    ReleaseResource();
}

Result SampleResnetDVPP::InitResource() {
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

    // create dvpp description
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    ret = acldvppCreateChannel(dvppChannelDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppCreateChannel failed, errorCode is %d", ret);
        return FAILED;
    }

    resizeConfig_ = acldvppCreateResizeConfig();
    vpcOutputDesc_ = acldvppCreatePicDesc();
    vpcInputDesc_ = acldvppCreatePicDesc();
    decodeOutputDesc_ = acldvppCreatePicDesc();

    // load model from file
    ret = aclmdlLoadFromFile(modelPath_, &modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclmdlLoadFromFile failed, errorCode is %d", ret);
        return FAILED;
    }

    // create model description
    modelDesc_ = aclmdlCreateDesc();
    ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclmdlGetDesc failed, errorCode is %d", ret);
        return FAILED;
    }

    // create input data set
    int32_t index = 0;
    devBufferSize_ = aclmdlGetInputSizeByIndex(modelDesc_, index);
    aclrtMalloc(&picDevBuffer_, devBufferSize_, ACL_MEM_MALLOC_NORMAL_ONLY);
    inputDataset_ = aclmdlCreateDataset();
    aclDataBuffer *inputData = aclCreateDataBuffer(picDevBuffer_, devBufferSize_);
    aclmdlAddDatasetBuffer(inputDataset_, inputData);

    // create output data set
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

Result SampleResnetDVPP::ProcessInput(const string imageFile)
{
    // read image from file
    ImageData image;
    std::ifstream binFile(imageFile, std::ifstream::binary);
    uint32_t offset = 0;
    binFile.seekg(offset, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    binFile.seekg(offset, binFile.beg);
    uint8_t* binFileBufferData = new(std::nothrow) uint8_t[binFileBufferLen];
    binFile.read((char *)binFileBufferData, binFileBufferLen);
    binFile.close();
    image.data.reset(binFileBufferData, [](uint8_t* p) { delete[](p); });
    image.size = binFileBufferLen;

    // copy data to dvpp
    void *buffer;
    acldvppMalloc(&buffer, image.size);
    aclError ret = aclrtMemcpy(buffer, image.size, image.data.get(),
                               image.size, ACL_MEMCPY_HOST_TO_DEVICE);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("copy image dvpp failed, errorCode is %d", ret);
        return FAILED;
    };
    ImageData imageDvpp;
    imageDvpp.size = image.size;
    imageDvpp.data = SHARED_PTR_DVPP_BUF(buffer);

    // get image info
    int32_t components = 0;
    acldvppJpegGetImageInfo(binFileBufferData, binFileBufferLen,
                            &(image.width), &(image.height),
                            &components);

    // alignment with different chip version
    auto socVersion = aclrtGetSocName();
    uint32_t decodeOutWidthStride;
    uint32_t decodeOutHeightStride;
    if (strncmp(socVersion, "Ascend310P3", sizeof("Ascend310P3") - 1) == 0) {
        imageDvpp.width = ALIGN_UP2(image.width);
        imageDvpp.height = ALIGN_UP2(image.height);
        decodeOutWidthStride = ALIGN_UP64(image.width); // 64-byte alignment
        decodeOutHeightStride = ALIGN_UP16(image.height); // 16-byte alignment
    } else {
        imageDvpp.width = image.width;
        imageDvpp.height = image.height;
        decodeOutWidthStride = ALIGN_UP128(image.width); // 128-byte alignment
        decodeOutHeightStride = ALIGN_UP16(image.height); // 16-byte alignment
    }

    // output size is calculated by  (widthStride * heightStride) * 3 / 2
    uint32_t decodeOutBufferSize = YUV420SP_SIZE(decodeOutWidthStride, decodeOutHeightStride);

    // create output description and set the properties
    acldvppMalloc(&decodeOutBufferDev_, decodeOutBufferSize);
    acldvppSetPicDescData(decodeOutputDesc_, decodeOutBufferDev_);
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(decodeOutputDesc_, imageDvpp.width);
    acldvppSetPicDescHeight(decodeOutputDesc_, imageDvpp.height);
    acldvppSetPicDescWidthStride(decodeOutputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(decodeOutputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(decodeOutputDesc_, decodeOutBufferSize);

    // decode image from JPEG format to YUV
    acldvppJpegDecodeAsync(dvppChannelDesc_, reinterpret_cast<void *>(imageDvpp.data.get()),
                           imageDvpp.size, decodeOutputDesc_, stream_);
    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppJpegDecodeAsync failed, errorCode is %d", ret);
        return FAILED;
    }

    // input data of resize
    ImageData yuvImage;
    yuvImage.width = ALIGN_UP2(imageDvpp.width);
    yuvImage.height = ALIGN_UP2(imageDvpp.height);
    yuvImage.alignWidth = decodeOutWidthStride;
    yuvImage.alignHeight = decodeOutHeightStride;
    yuvImage.size = decodeOutBufferSize;
    yuvImage.data = SHARED_PTR_DVPP_BUF(decodeOutBufferDev_);

    // create input description and set the attributes
    acldvppSetPicDescData(vpcInputDesc_, yuvImage.data.get());  // set input desc
    acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcInputDesc_, yuvImage.width);
    acldvppSetPicDescHeight(vpcInputDesc_, yuvImage.height);
    acldvppSetPicDescWidthStride(vpcInputDesc_, yuvImage.alignWidth);
    acldvppSetPicDescHeightStride(vpcInputDesc_, yuvImage.alignHeight);
    acldvppSetPicDescSize(vpcInputDesc_, yuvImage.size);

    // output widthStride should be 16 aligned, heightStride should be 2 aligned
    int resizeOutWidth = ALIGN_UP2(modelWidth_);
    int resizeOutHeight = ALIGN_UP2(modelHeight_);
    int resizeOutWidthStride = ALIGN_UP16(resizeOutWidth);
    int resizeOutHeightStride = ALIGN_UP2(resizeOutHeight);
    uint32_t vpcOutBufferSize;
    void *vpcOutBufferDev; // vpc output buffer
    vpcOutBufferSize = YUV420SP_SIZE(resizeOutWidthStride, resizeOutHeightStride);
    acldvppMalloc(&vpcOutBufferDev, vpcOutBufferSize);

    // create output description and set the properties
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev);   // set output desc
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, resizeOutWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, resizeOutHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize);

    // execute resize
    acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
                          vpcOutputDesc_, resizeConfig_, stream_);
    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppVpcResizeAsync failed, errorCode is %d", ret);
        return FAILED;
    }

    std::shared_ptr<uint8_t> data = SHARED_PTR_DVPP_BUF(vpcOutBufferDev);
    ret = aclrtMemcpy(picDevBuffer_, devBufferSize_,
                      data.get(), devBufferSize_,
    ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("memcpy from device to device failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleResnetDVPP::Inference()
{
    // inference
    aclError ret = aclmdlExecute(modelId_, inputDataset_, outputDataset_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, errorCode is %d", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result SampleResnetDVPP::GetResult() {
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
        ERROR_LOG("aclrtMemcpy failed, errorCode is %d", ret);
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

void SampleResnetDVPP::ReleaseResource()
{
    aclError ret;
    ret = acldvppDestroyResizeConfig(resizeConfig_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("destroy resize config failed, errorCode is %d", ret);
    }

    ret = acldvppDestroyPicDesc(decodeOutputDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("destroy picture description failed, errorCode is %d", ret);
    }

    ret = acldvppDestroyPicDesc(vpcInputDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("destroy picture description failed, errorCode is %d", ret);
    }

    ret = acldvppDestroyPicDesc(vpcOutputDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("destroy picture description failed, errorCode is %d", ret);
    }

    ret = acldvppDestroyChannel(dvppChannelDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("destroy channel failed, errorCode is %d", ret);
    }

    ret = acldvppDestroyChannelDesc(dvppChannelDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("destroy channel description failed, errorCode is %d", ret);
    }

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

    aclmdlDestroyDesc(modelDesc_);
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
            ERROR_LOG("aclrtDestroyContext faild, errorCode is %d", ret);
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
    const int32_t modelWidth = 224;
    const int32_t modelHeight = 224;
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
            string imgDir = imagePath +"/"+ name;
            allPath.push_back(imgDir);
        }
    }
    closedir(dir);

    if (allPath.size() == 0){
        ERROR_LOG("the directory is empty, please download image to %s", imagePath.c_str());
        return FAILED;
    }

    string fileName;
    SampleResnetDVPP sampleResnet(device, modelPath, modelWidth, modelHeight);
    Result ret = sampleResnet.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("InitResource failed");
        return FAILED;
    }

    for (size_t i = 0; i < allPath.size(); i++)
    {
        fileName = allPath.at(i).c_str();
        ret = sampleResnet.ProcessInput(fileName);
        if (ret != SUCCESS) {
            ERROR_LOG("ProcessInput failed");
            return FAILED;
        }

        ret = sampleResnet.Inference();
        if (ret != SUCCESS) {
            ERROR_LOG("Inference failed");
            return FAILED;
        }

        ret = sampleResnet.GetResult();
        if (ret != SUCCESS) {
            ERROR_LOG("GetResult failed");
            return FAILED;
        }
    }
    return SUCCESS;
}