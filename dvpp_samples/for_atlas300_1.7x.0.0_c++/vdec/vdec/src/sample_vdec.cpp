#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "sample_vdec.h"

std::string filePath= "../vdec_h265_1frame_rabbit_1280x720.h265";
const int inputWidth = 1280;
const int inputHeight = 720;

int32_t deviceId_;
aclrtContext context_;
aclrtStream stream_;
pthread_t threadId_;
char *outFolder;
PicDesc picDesc_;

int32_t format_ = 1; // 1：YUV420 semi-planner（nv12）; 2：YVU420 semi-planner（nv21）

    /* 0：H265 main level
     * 1：H264 baseline level
     * 2：H264 main level
     * 3：H264 high level
     */
int32_t enType_ =0 ;

aclvdecChannelDesc *vdecChannelDesc_;
acldvppStreamDesc *streamInputDesc_;
acldvppPicDesc *picOutputDesc_;
void *picOutBufferDev_;
void *inBufferDev_;
uint32_t inBufferSize_;
static bool runFlag = true;

bool ReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize)
{
    // read data from file.
    FILE *fp = fopen(fileName, "rb+");

    fseek(fp, 0, SEEK_END);
    long fileLenLong = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    auto fileLen = static_cast<uint32_t>(fileLenLong);
    void *dataHost = malloc(fileLen);

    size_t readSize = fread(dataHost, 1, fileLen, fp);
    if (readSize < fileLen) {
        free(dataHost);
        return false;
    }

    dataSize = fileLen;
    // Malloc input device memory
    auto aclRet = acldvppMalloc(&dataDev, dataSize);
    // copy input to device memory
    aclRet = aclrtMemcpy(dataDev, dataSize, dataHost, fileLen, ACL_MEMCPY_HOST_TO_DEVICE);
    free(dataHost);
    return true;
}

void *ThreadFunc(void *arg)
{
    // Notice: create context for this thread
    int deviceId = 0;
    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, deviceId);
    while (runFlag) {
        // Notice: timeout 1000ms
        aclError aclRet = aclrtProcessReport(1000);
    }

    ret = aclrtDestroyContext(context);
    return (void*)0;
}
bool WriteToFile(const char *fileName, const void *dataDev, uint32_t dataSize)
{
    void *dataHost = malloc(dataSize);
    if (dataHost == nullptr) {
        ERROR_LOG("malloc host data buffer failed. dataSize=%u\n", dataSize);
        return false;
    }

    // copy output to host memory
    auto aclRet = aclrtMemcpy(dataHost, dataSize, dataDev, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);

    FILE *outFileFp = fopen(fileName, "wb+");

    bool ret = true;
    size_t writeRet = fwrite(dataHost, 1, dataSize, outFileFp);
    if (writeRet != dataSize) {
        ret = false;
    }
    free(dataHost);
    fflush(outFileFp);
    fclose(outFileFp);
    return ret;
}

//3.创建回调函数
void callback(acldvppStreamDesc *input, acldvppPicDesc *output, void *userdata)
{
    //获取VDEC解码的输出内存，调用自定义函数WriteToFile将输出内存中的数据写入文件后，再调用acldvppFree接口释放输出内存
    void *vdecOutBufferDev = acldvppGetPicDescData(output);
    uint32_t size = acldvppGetPicDescSize(output);
    static int count = 1;
    std::string fileNameSave = "image" + std::to_string(count);
    if (!WriteToFile(fileNameSave.c_str(), vdecOutBufferDev, size)) {
        ERROR_LOG("write file failed.");
    }
    aclError ret = acldvppFree(reinterpret_cast<void *>(vdecOutBufferDev));
    // 释放acldvppPicDesc类型的数据，表示解码后输出图片描述数据
    ret = acldvppDestroyPicDesc(output);
	
    //......
    count++;
}

int main()
{	
	/* 1. ACL初始化 */	
    const char *aclConfigPath = "../acl.json";
    aclError ret = aclInit(aclConfigPath);
	
	/* 2. 运行管理资源申请,包括Device、Context、Stream */	
    ret = aclrtSetDevice(deviceId_);
    ret = aclrtCreateContext(&context_, deviceId_);
    ret = aclrtCreateStream(&stream_);
		
	/* 3. Vdec 资源初始化 */	
	// create threadId
    int createThreadErr = pthread_create(&threadId_, nullptr, ThreadFunc, nullptr);
    (void)aclrtSubscribeReport(static_cast<uint64_t>(threadId_), stream_);
	
	//4.创建视频码流处理通道时的通道描述信息，设置视频处理通道描述信息的属性，其中callback回调函数需要用户提前创建。
	//vdecChannelDesc_是aclvdecChannelDesc类型
	vdecChannelDesc_ = aclvdecCreateChannelDesc();
	
	// channelId: 0-15
	ret = aclvdecSetChannelDescChannelId(vdecChannelDesc_, 10);
	ret = aclvdecSetChannelDescThreadId(vdecChannelDesc_, threadId_);
	/* 设置回调函数 callback*/
	ret = aclvdecSetChannelDescCallback(vdecChannelDesc_, callback);
	
	//示例中使用的是H265_MAIN_LEVEL视频编码协议
	ret = aclvdecSetChannelDescEnType(vdecChannelDesc_, static_cast<acldvppStreamFormat>(enType_));
	//示例中使用的是PIXEL_FORMAT_YVU_SEMIPLANAR_420
	ret = aclvdecSetChannelDescOutPicFormat(vdecChannelDesc_, static_cast<acldvppPixelFormat>(format_));

	/* 5.创建视频码流处理的通道 */
	ret = aclvdecCreateChannel(vdecChannelDesc_);
	
	/* 视频解码处理 */
    int rest_len = 10;
    void *inBufferDev = nullptr;
    uint32_t inBufferSize = 0;
    size_t DataSize = (inputWidth * inputHeight * 3) / 2;

    // read file to device memory
    ReadFileToDeviceMem(filePath.c_str(), inBufferDev, inBufferSize);
	
    int32_t count = 0;
	
 	// 创建输入视频码流描述信息，设置码流信息的属性
	streamInputDesc_ = acldvppCreateStreamDesc(); 
    while (rest_len > 0) {
		
		//inBufferDev_表示Device存放输入视频数据的内存，inBufferSize_表示内存大小  
		ret = acldvppSetStreamDescData(streamInputDesc_, inBufferDev);
		ret = acldvppSetStreamDescSize(streamInputDesc_, inBufferSize);

		//申请Device内存picOutBufferDev_，用于存放VDEC解码后的输出数据
		ret = acldvppMalloc(&picOutBufferDev_, DataSize);

		//创建输出图片描述信息，设置图片描述信息的属性
		//picOutputDesc_是acldvppPicDesc类型
		picOutputDesc_ = acldvppCreatePicDesc();
		ret = acldvppSetPicDescData(picOutputDesc_, picOutBufferDev_);
		ret = acldvppSetPicDescSize(picOutputDesc_, DataSize);
		ret = acldvppSetPicDescFormat(picOutputDesc_, static_cast<acldvppPixelFormat>(format_));

		// 执行视频码流解码，解码每帧数据后，系统自动调用callback回调函数将解码后的数据写入文件，再及时释放相关资源
		ret = aclvdecSendFrame(vdecChannelDesc_, streamInputDesc_, picOutputDesc_, nullptr, nullptr);
		//......
		++count;
		rest_len = rest_len - 1;		
    }
	acldvppFree(inBufferDev);
	
    ret = aclvdecDestroyChannel(vdecChannelDesc_);
    aclvdecDestroyChannelDesc(vdecChannelDesc_);
    vdecChannelDesc_ = nullptr;
	
    (void)aclrtUnSubscribeReport(static_cast<uint64_t>(threadId_), stream_);
	
	ret = acldvppDestroyStreamDesc(streamInputDesc_);
	
    // destory thread
    runFlag = false;
    void *res = nullptr;
    int joinThreadErr = pthread_join(threadId_, &res);
    
    ret = aclrtDestroyStream(stream_);
    stream_ = nullptr;
    ret = aclrtDestroyContext(context_);
    context_ = nullptr;
    ret = aclrtResetDevice(deviceId_);
    ret = aclFinalize();
    return SUCCESS;
}
