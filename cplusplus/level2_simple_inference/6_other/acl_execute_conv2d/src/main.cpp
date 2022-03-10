// int main()
#include "acl/acl.h"
#include "utils.h"
#include "acl/ops/acl_cblas.h"
#include "acl/acl_op_compiler.h"

using namespace std;

void PrintResult(void * out_buffers,uint32_t out_tensor_size, std::string out_file){
    void* hostBuffer = nullptr;
    void* outData = nullptr;
    aclError ret = aclrtMallocHost(&hostBuffer, out_tensor_size);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to print result, malloc host failed");
	    
    }
    ret = aclrtMemcpy(hostBuffer, out_tensor_size, out_buffers,out_tensor_size, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to print result, memcpy device to host failed, errorCode is %d", static_cast<int32_t>(ret));
        aclrtFreeHost(hostBuffer);
	    
    }
    outData = reinterpret_cast<aclFloat16*>(hostBuffer);
    ofstream outstr(out_file, ios::out | ios::binary);
    outstr.write((char*)outData, out_tensor_size);
    outstr.close();

}

int main(int argc, char* argv[])
{ 

    for (int i = 0; i < argc; i++) {
        cout << "No." << i << " parameter is:" << argv[i] << endl;
    }

    std::string input_x_file = argv[1];
    std::string input_filter_file = argv[2];
    std::string out_file = argv[3];

    std::vector<int64_t> inputShapeCast{2, 1024, 1024, 3};
    std::vector<int64_t> inputFilterShapeCast{6, 3, 3, 3};
    std::vector<int64_t> outputShapeCast{2, 1024, 1024, 6};

    // single op call
    const char* opType_ = "Conv2D";
    int numInput = 2;
    int numOutput = 1;

    aclDataType inputDataTypeCast = ACL_FLOAT16;
    aclDataType outputDataTypeCast = ACL_FLOAT16;

    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("acl init success");

    int32_t deviceId_ = 0;
    aclrtContext context_;
    aclrtStream stream_;

    // set device
    ret = aclrtSetDevice(deviceId_);
    ret = aclrtCreateContext(&context_, deviceId_);
    ret = aclrtCreateStream(&stream_);  

    // set model dir 
    ret = aclopSetModelDir("../model");

    aclTensorDesc *inputDescCast[numInput];
    aclTensorDesc *OutputDescCast[numOutput];

    // Create aclTensorDesc, to describe the shape/format/datatype, etc.
    inputDescCast[0] = aclCreateTensorDesc(inputDataTypeCast, 
                                           inputShapeCast.size(), 
                                           inputShapeCast.data(), 
                                           ACL_FORMAT_NHWC);
    inputDescCast[1] = aclCreateTensorDesc(inputDataTypeCast, 
                                           inputFilterShapeCast.size(), 
                                           inputFilterShapeCast.data(), 
                                           ACL_FORMAT_NCHW);
    OutputDescCast[0] = aclCreateTensorDesc(outputDataTypeCast, 
                                            outputShapeCast.size(), 
                                            outputShapeCast.data(), 
                                            ACL_FORMAT_NHWC);
 
    // set Conv2D attr
    aclopAttr *opAttr = aclopCreateAttr();
    if (opAttr == nullptr) {
        ERROR_LOG("singleOp create attr failed");
        return FAILED;
    }
    int64_t intList[4]{1, 1, 1, 1};

    ret = aclopSetAttrListInt(opAttr, "strides", 4, intList);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("singleOp set strides attr failed");
        aclopDestroyAttr(opAttr);
        return FAILED;
    }
    
    ret = aclopSetAttrListInt(opAttr, "pads", 4, intList);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("singleOp set pads attr failed");
        aclopDestroyAttr(opAttr);
        return FAILED;
    }

    ret = aclopSetAttrListInt(opAttr, "dilations", 4, intList);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("singleOp set dilations attr failed");
        aclopDestroyAttr(opAttr);
        return FAILED;
    }   

    void* x_tensor_ptr = nullptr;
    void* filter_tensor_ptr = nullptr;
    void* out_tensor_ptr = nullptr;
    uint32_t x_tensor_size;
    uint32_t filter_tensor_size;
    uint32_t out_tensor_size = 25165824;
    std::vector<aclDataBuffer*> in_buffers;
    std::vector<aclDataBuffer*> out_buffers;

    x_tensor_ptr = Utils::GetDeviceBufferOfFile(input_x_file, x_tensor_size);
    filter_tensor_ptr = Utils::GetDeviceBufferOfFile(input_filter_file, filter_tensor_size);

    ret = aclrtMalloc(&out_tensor_ptr, out_tensor_size, ACL_MEM_MALLOC_HUGE_FIRST);

    aclDataBuffer* x_tensor_data = aclCreateDataBuffer(x_tensor_ptr, x_tensor_size);
  
    if (x_tensor_data == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        return FAILED;
    }

    aclDataBuffer* filter_tensor_data = aclCreateDataBuffer(filter_tensor_ptr, filter_tensor_size);
    if (filter_tensor_data == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        return FAILED;
    }

    aclDataBuffer* bias_tensor_data = aclCreateDataBuffer(nullptr, 0);

    aclDataBuffer* out_tensor_data = aclCreateDataBuffer(out_tensor_ptr, out_tensor_size);
    if (out_tensor_data == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        return FAILED;
    }

    in_buffers.push_back(x_tensor_data);
    in_buffers.push_back(filter_tensor_data);
    in_buffers.push_back(bias_tensor_data);
    out_buffers.push_back(out_tensor_data);
                            
    for(int i = 0; i < 1; i++)
    {
        ret = aclopExecuteV2(opType_, numInput, inputDescCast, 
        in_buffers.data(), numOutput, OutputDescCast, out_buffers.data(),
        opAttr, stream_);
        ret = aclrtSynchronizeStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("execute singleOp conv2d failed, errorCode is %d", static_cast<int32_t>(ret));
            aclDestroyTensorDesc(inputDescCast[0]);
            aclDestroyTensorDesc(OutputDescCast[0]);
            return FAILED;
        }

        INFO_LOG("execute conv2d %d", i);
        PrintResult(out_tensor_ptr, out_tensor_size, out_file);

    }    

    INFO_LOG("execute op success");

    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
    }
    INFO_LOG("end to reset device ");

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed, errorCode = %d", static_cast<int32_t>(ret));
    }
    INFO_LOG("end to finalize acl");

    return SUCCESS;
}
