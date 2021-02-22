/**
 * Copyright (C)  2020. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use this
 file except in compliance with the License.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * @file acl_engine.cpp
 *
 * @brief
 *
 * @version 1.0
 *
 */

#include "acl_engine.h"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <regex>

using namespace std;

#define LOG_DEBUG(fmt, ...)                                                    \
    do {                                                                       \
        if (params_.is_debug) {                                                \
            printf(fmt, ##__VA_ARGS__);                                        \
        }                                                                      \
    } while (0)

AclEngine::~AclEngine() {
    delete[] model_data_ptr_;
    aclrtFree(device_ptr_);
    aclrtFree(weight_ptr_);

    for (auto item : input_buffers_) {
        if (item != nullptr) {
            aclrtFreeHost(item);
        }
    }

    ReleaseAllAclModelResource(model_id_, model_desc_ptr_, contexts_);
    aclFinalize();
}

bool AclEngine::ParseParams(int argc, char **argv) {
    const struct option long_options[] = {
        {"inputFiles", 1, nullptr, 'i'},   {"outputFile", 1, nullptr, 'o'},
        {"model", 1, nullptr, 'm'},        {"batchSize", 1, nullptr, 'b'},
        {"loopNum", 1, nullptr, 'l'},      {"help", 0, nullptr, 'h'},
        {"onlyOneBatch", 0, nullptr, 'x'}, {"debug", 0, nullptr, 'd'},
        {nullptr, 0, nullptr, 0},
    };

    bool parse_result = true;

    while (true) {
        int option_index = 0;
        int c =
            getopt_long(argc, argv, "i:o:m:b:l:h", long_options, &option_index);
        if (c == -1) {
            break;
        }

        switch (c) {
        case 'i':
            split(optarg, &(params_.input_files), ",");
            printf("[INFO]inputFile NUM = %lu\n", params_.input_files.size());
            break;
        case 'o':
            params_.output_path = std::string(optarg);
            break;
        case 'm':
            params_.model_file = std::string(optarg);
            break;
        case 'b':
            params_.batch_size = strtol(optarg, nullptr, 10);
            break;
        case 'l':
            params_.loops = strtol(optarg, nullptr, 10);
            break;
        case 'h':
            params_.is_help = true;
            break;
        case 'x':
            params_.only_run_one_batch = true;
            break;
        case 'd':
            params_.is_debug = true;
            break;
        default:
            parse_result = false;
            break;
        }
    }

    return parse_result;
}

bool AclEngine::Inference() {
    aclError ret = 0;
    int retVal = 0;

    const std::string &modelPath = params_.model_file;
    const uint32_t batchSize = params_.batch_size;

    if (LoadModel() != 0) {
        return false;
    }
    puts("[step 1] load model success\n");

    // input_count_ -= 1;
    if (input_count_ != params_.input_files.size()) {
        printf("[ERROR] input file num not match, [%lu / %d]\n",
               params_.input_files.size(), input_count_);
        return false;
    }

    vector<vector<string>> all_inputs;
    for (const auto &item : params_.input_files) {
        all_inputs.emplace_back(GetFiles(item));
    }
    auto all_files_count =
        all_inputs[0].size(); //一共多少个文件，一次输入当作一个文件
    printf("all_files_count=%u\n", all_files_count);
    size_t batch_index = 1;
    std::vector<int64_t> cost_times;
    cost_times.reserve(all_files_count * params_.loops);
    vector<vector<string>> inferFileVec(input_count_);
    for (size_t index = 0; index < all_files_count; index++) {
        if (index < params_.batch_size * batch_index) {
            for (size_t input_index = 0; input_index < input_count_;
                 input_index++) {
                inferFileVec[input_index].push_back(
                    all_inputs[input_index][index]);
            }
        }
        if (index + 1 == all_files_count ||
            inferFileVec[0].size() == params_.batch_size) {
            batch_index++;

            if (ReadInputFiles(inferFileVec) != 0) {
                puts("read input file failed.");
                return 1;
            }

            retVal = AclInferenceProcess(model_id_, batch_dsts_, inferFileVec,
                                         &cost_times);
            for (auto &item : inferFileVec) {
                item.clear();
            }

            if (0 != retVal) {
                printf("kModelId %u aclInferenceProcess, ret[%u]\n", model_id_,
                       ret);
                ReleaseAllAclModelResource(model_id_, model_desc_ptr_,
                                           contexts_);
                return 1;
            }

            if (params_.only_run_one_batch) {
                break;
            }
        }
    }

    puts("Success to execute acl demo!\n");

    double average = 0.0;
    int batch_cnt = cost_times.size();

    for (auto cost_time : cost_times) {
        average += cost_time;
    }

    average = average / (batch_cnt * batchSize * 1000);
    printf("batch:%lu\n", batchSize);
    printf("input count:%u\noutput count:%u\n", input_count_, output_count_);
    printf("NN inference cost average time: %4.3f ms %4.3f fps/s\n", average,
           (1000 / average));

    return 0;
}

char *AclEngine::ReadBinFile(const std::string &file_name,
                             uint32_t *p_file_size) {
    std::ifstream binFile(file_name, std::ifstream::binary);
    if (!binFile.is_open()) {
        printf("open file[%s] failed\n", file_name.c_str());
        return nullptr;
    }

    binFile.seekg(0, std::ifstream::end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        printf("binfile is empty, filename: %s", file_name.c_str());
        binFile.close();
        return nullptr;
    }

    binFile.seekg(0, std::ifstream::beg);

    char *binFileBufferData = new (std::nothrow) char[binFileBufferLen];
    printf("binFileBufferData:%p\n", binFileBufferData);
    if (binFileBufferData == nullptr) {
        printf("malloc binFileBufferData failed\n");
        binFile.close();
        return nullptr;
    }
    binFile.read(binFileBufferData, binFileBufferLen);
    binFile.close();
    *p_file_size = binFileBufferLen;
    return binFileBufferData;
}

aclError AclEngine::InitAclDeviceContext() {

    aclError ret = ACL_ERROR_NONE;
    uint32_t device_count = 1;
    ret = aclrtGetDeviceCount(&device_count);
    if (ret != ACL_ERROR_NONE) {
        printf("[ERROR]aclrtGetDeviceCount failed, ret %d\n", ret);
        return ret;
    }

    printf("device count:%u\n", device_count);

    if (kDeviceNum > device_count) {
        printf("[ERROR] kDeviceNum failed, ret %d\n", ret);
        return 1;
    }

    uint32_t device_start_id = 0;
    const char *env_device_id = getenv("DEVICE_ID");
    if (env_device_id != nullptr) {
        device_start_id = strtoul(env_device_id, nullptr, 10);
    }
    printf("DEVICE_ID=%u\n", device_start_id);

    for (uint32_t devIndex = device_start_id;
         devIndex < kDeviceNum + device_start_id; devIndex++) {
        ret = aclrtSetDevice(devIndex);
        if (ret != ACL_ERROR_NONE) {
            printf("[ERROR]aclrtSetDevice failed, ret %d\n", ret);
            return ret;
        }
        used_devices_.push_back(devIndex);
        aclrtContext context;
        ret = aclrtCreateContext(&context, devIndex);
        if (ret != ACL_ERROR_NONE) {
            printf("[ERROR]aclrtCreateContext failed, ret %d\n", ret);
            return ret;
        }
        contexts_.push_back(context);
    }
    return ret;
}

void AclEngine::DestroyAclModelDesc(uint32_t modelId, aclmdlDesc *modelDesc) {

    aclError ret;

    ret = aclmdlUnload(modelId);
    if (ret != ACL_ERROR_NONE) {
        printf("aclmdlUnload  failed, ret[%d]\n", ret);
    }
    printf("[step 10] unload model success\n");

    ret = aclmdlDestroyDesc(modelDesc);
    if (ret != ACL_ERROR_NONE) {
        printf("aclmdlDestroyDesc  failed, ret[%d]\n", ret);
    }
}
void AclEngine::DestroyAclDeviceContext(
    uint32_t devNum, const std::vector<aclrtContext> &context_vec) {
    for (uint32_t devIndex : used_devices_) {
        aclrtResetDevice(devIndex);
    }

    for (auto context : context_vec) {
        aclrtDestroyContext(context);
    }
}
void AclEngine::ReleaseAllAclModelResource(
    uint32_t modelId, aclmdlDesc *modelDesc,
    const std::vector<aclrtContext> &contex_vec) {
    if (modelDesc && modelId) {
        DestroyAclModelDesc(modelId, modelDesc);
    }

    DestroyAclDeviceContext(kDeviceNum, contex_vec);
}

void AclEngine::ReleaseAclModelInput(aclmdlDataset *input) {

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(input); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(input, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        aclrtFree(data);
        aclDestroyDataBuffer(dataBuffer);
    }
    aclmdlDestroyDataset(input);
}

void AclEngine::ReleaseAclModelOutput(aclmdlDataset *output) {

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(output); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(output, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        aclrtFree(data);
        aclDestroyDataBuffer(dataBuffer);
    }
    aclmdlDestroyDataset(output);
}

bool AclEngine::Init() {
    std::string configPath = "./acl.json";
    if (!is_file(configPath)) {
        printf("configPath %s not exist, will not dump data.\n",
               configPath.c_str());
        configPath = "";
    }

    int ret = aclInit(configPath.c_str());
    if (ret != ACL_ERROR_NONE) {
        printf("aclInit failed, ret[%d]\n", ret);
        return false;
    }

    ret = InitAclDeviceContext();
    if (ret != ACL_ERROR_NONE) {
        printf("aclDeviceContexInit failed, ret[%d]\n", ret);
        return true;
    }
    return true;
}

int AclEngine::LoadModel() {
    const std::string &model_path = params_.model_file;
    uint32_t model_size = 0;
    model_data_ptr_ = ReadBinFile(model_path, &model_size);
    if (model_data_ptr_ == nullptr) {
        printf("model ReadBinFile failed\n");
        DestroyAclDeviceContext(kDeviceNum, contexts_);
        return 1;
    }

    int ret = aclmdlQuerySizeFromMem(model_data_ptr_, model_size, &mem_size_,
                                     &weight_size_);
    if (ret != 0) {
        printf("model aclmdlQuerySizeFromMem failed, ret[%d]\n", ret);
        DestroyAclDeviceContext(kDeviceNum, contexts_);
    } else {
        printf(
            "aclmdlQuerySizeFromMem success, kMemSize[%u], kWeightSize[%u]\n",
            mem_size_, weight_size_);
    }

    ret = aclrtMalloc(&device_ptr_, mem_size_, ACL_MEM_MALLOC_HUGE_FIRST);
    printf("Dev_ptr:%p\n", device_ptr_);
    if (ret != 0) {
        printf("aclrtMalloc kDevPtr failed, ret[%d]\n", ret);
        DestroyAclDeviceContext(kDeviceNum, contexts_);
        return 1;
    }

    ret = aclrtMalloc(&weight_ptr_, weight_size_, ACL_MEM_MALLOC_HUGE_FIRST);
    printf("kWeightPtr:%p\n", weight_ptr_);
    if (ret != 0) {
        printf("aclrtMalloc kWeightPtr failed, ret[%d]\n", ret);
        DestroyAclDeviceContext(kDeviceNum, contexts_);
        return 1;
    }

    model_desc_ptr_ =
        aclModelLoadAndIOGet(model_data_ptr_, model_size, &model_id_,
                             device_ptr_, mem_size_, weight_ptr_, weight_size_);
    if (nullptr == model_desc_ptr_) {
        printf("model aclModelIOGet");
        DestroyAclDeviceContext(kDeviceNum, contexts_);
        return 1;
    }

    input_count_ = aclmdlGetNumInputs(model_desc_ptr_);
    output_count_ = aclmdlGetNumOutputs(model_desc_ptr_);
    printf("From model input count:%lu\n", input_count_);
    printf("From model output count:%lu\n", output_count_);

    for (size_t index = 0; index < input_count_; index++) {
        auto buff_size = aclmdlGetInputSizeByIndex(model_desc_ptr_, index);
        printf("From model input%lu buffer size:%lu\n", index, buff_size);
        input_buffer_sizes_.push_back(buff_size);
        void *p_imgBuf = nullptr;
        auto ret = aclrtMallocHost(&p_imgBuf, buff_size);
        if (ret != ACL_ERROR_NONE) {
            printf("p_imgBuf aclrtMallocHost failed[%d]\n", ret);
            ReleaseAllAclModelResource(model_id_, model_desc_ptr_, contexts_);
            return 1;
        }

        input_buffers_.push_back(p_imgBuf);
    }

    for (size_t index = 0; index < output_count_; index++) {
        auto buff_size = aclmdlGetOutputSizeByIndex(model_desc_ptr_, index);
        printf("From model output%lu buffer size:%lu\n", index, buff_size);
        output_buffer_sizes_.push_back(buff_size);
    }
    return 0;
}

aclmdlDesc *AclEngine::aclModelLoadAndIOGet(const void *model,
                                            size_t model_size,
                                            uint32_t *model_id, void *dev_ptr,
                                            size_t mem_size, void *weight_ptr,
                                            size_t weight_size) {
    aclError ret =
        aclmdlLoadFromMemWithMem(model, model_size, model_id, dev_ptr, mem_size,
                                 weight_ptr, weight_size);
    if (ret != ACL_ERROR_NONE) {
        printf("aclmdlLoadFromMemWithMem failed, ret[%d]\n", ret);
        return nullptr;
    }

    aclmdlDesc *p_aclmdl_desc = aclmdlCreateDesc();
    if (p_aclmdl_desc == nullptr) {
        printf("aclmdlCreateDesc failed\n");
        aclmdlUnload(*model_id);
        return nullptr;
    }

    ret = aclmdlGetDesc(p_aclmdl_desc, *model_id);
    if (ret != 0) {
        printf("aclmdlGetDesc ret fail, ret:%d\n", ret);
        aclmdlDestroyDesc(p_aclmdl_desc);
        aclmdlUnload(*model_id);
        return nullptr;
    }

    return p_aclmdl_desc;
}

int AclEngine::ReadInputFiles(const vector<vector<string>> &inputs) {
    if (input_count_ > inputs.size()) {
        printf("[ERROR] input file num not match, [%lu / %lu]\n", inputs.size(),
               input_count_);
        return 1;
    }

    batch_dsts_.clear();
    printf("input_count_ = %lu\n", input_count_);
    for (size_t inIdx = 0; inIdx < input_count_; ++inIdx) {
        const auto &fileNames = inputs[inIdx];
        void *p_imgBuf = input_buffers_[inIdx];
        size_t fileSize = input_buffer_sizes_[inIdx] / params_.batch_size;
        printf("params_.batch_size = %lu \n", params_.batch_size);
        printf("fileSize = %lu \n", fileSize);
        size_t pos = 0;
        bool isInputBin = false; // TO READ BIN WITHOUT C++ PREPROCESS OR READ
                                 // TXT WITH C++ PREPROCESS

        for (const auto &fileName : fileNames) {
            if (isInputBin == true) {
                FILE *pFile = fopen(fileName.c_str(), "r");
                if (nullptr == pFile) {
                    printf("open file %s failed\n", fileName.c_str());
                    ReleaseAllAclModelResource(model_id_, model_desc_ptr_,
                                               contexts_);
                    return 1;
                }

                LOG_DEBUG("read file:%s\n", fileName.c_str());
                if (fread(((char *)p_imgBuf) + pos, sizeof(char), fileSize,
                          pFile) != fileSize) {
                    printf(
                        "fread %s failed. file size not match input size %lu.\n",
                        fileName.c_str(), fileSize);
                    fclose(pFile);
                    return 1;
                }
                pos += fileSize;
                fclose(pFile);
            } else {
                cout << "start to do PREPROCESS ........................"
                     << endl;
                PreProcess(fileName, p_imgBuf, pos,
                           fileSize); // data preprocess
                cout << "............................end  PREPROCESS " << endl;
            }
        }

        fileSize *= params_.batch_size;
        void *pbatchDst;
        auto ret =
            aclrtMalloc(&pbatchDst, fileSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            printf("aclrtMalloc failed, ret[%d]\n", ret);
            ReleaseAllAclModelResource(model_id_, model_desc_ptr_, contexts_);
            return 1;
        }

        ret = aclrtMemcpy(pbatchDst, fileSize, p_imgBuf, fileSize,
                          ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            printf("aclrtMemcpy failed, ret[%d] line : %d\n", ret, __LINE__);
            aclrtFree(pbatchDst);
            ReleaseAllAclModelResource(model_id_, model_desc_ptr_, contexts_);
            return 1;
        }
        batch_dsts_.push_back(pbatchDst);
    }

    return 0;
}

int AclEngine::AclInferenceProcess(uint32_t model_id,
                                   std::vector<void *> inputBuf,
                                   const vector<vector<string>> &inputs,
                                   std::vector<int64_t> *cost_times) {
    const auto &output_path = params_.output_path;
    const auto &loop_num = params_.loops;

    aclError ret;
    printf("start to process inference");
    char cur_dir[256] = {0};
    if (getcwd(cur_dir, sizeof(cur_dir) - 1) == nullptr) {
        printf("getcwd failed.\n");
        return 1;
    }

    shared_ptr<aclmdlDataset> input(aclmdlCreateDataset(),
                                    ReleaseAclModelInput);
    if (!input) {
        printf("ACL_ModelInputCreate failed\n");
        return 1;
    }

    size_t inputSize = input_count_;
    for (size_t inIdx = 0; inIdx < inputSize; ++inIdx) {
        uint64_t buffer_size = input_buffer_sizes_[inIdx];

        aclDataBuffer *inputData =
            aclCreateDataBuffer(inputBuf[inIdx], buffer_size);
        if (inputData == nullptr) {
            printf("aclCreateDataBuffer failed\n");
            return 1;
        }

        ret = aclmdlAddDatasetBuffer(input.get(), inputData);
        if (ret != ACL_ERROR_NONE) {
            printf("ACL_ModelInputDataAdd failed, ret[%d]\n", ret);
            return 1;
        }
    }
    /*
    size_t index;
    ret = aclmdlGetInputIndexByName(model_desc_ptr_, ACL_DYNAMIC_TENSOR_NAME,
    &index); if (ret != ACL_ERROR_NONE) { printf("aclmdlGetInputIndexByName
    failed, maybe static model\n");
    }
    size_t buffer_size = aclmdlGetInputSizeByIndex(model_desc_ptr_, index);
    void* inputBuffer = NULL;
    ret = aclrtMalloc(&inputBuffer, buffer_size , ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != ACL_ERROR_NONE) {
        printf("aclrtMalloc failed, ret[%d]", ret);
    }
    aclDataBuffer* inputData = aclCreateDataBuffer(inputBuffer, buffer_size);
    if(inputData == NULL) {
        printf("aclCreateDataBuffer failed");
    }
    ret = aclmdlAddDatasetBuffer(input.get(), inputData);
    if(ret != ACL_ERROR_NONE) {
        printf("ACL_ModelInputDataAdd failed, ret[%d]", ret);
    }
    //aclDataBuffer* buffer = aclmdlGetDatasetBuffer(input, index);
    //if (buffer == nullptr) {
    //  printf("fail to get dataset buffer with dynamic\n");
    //}
    //
    //void* devPtr = aclGetDataBufferAddr(buffer);
    //if (devPtr == nullptr) {
    //  printf("fail to get data buffer addr with dynamic\n");
    //}
    //
    //size_t memSize = aclGetDataBufferSize(buffer);
    //if (memSize == 0) {
    //  printf("fail to get data mem size with dynamic\n");
    //}
    ret = aclmdlSetDynamicHWSize(model_id, input.get(), index, 32, 64);
    if (ret != ACL_ERROR_NONE) {
      printf("aclmdlSetDynamicHWSize failed");
    }
    printf("aclmdlSetDynamicHWSize success");
    */
    shared_ptr<aclmdlDataset> output(aclmdlCreateDataset(),
                                     AclEngine::ReleaseAclModelOutput);
    if (!output) {
        printf("ACL_ModelOutputCreate failed\n");
        return 1;
    }

    for (size_t outIndex = 0; outIndex < output_count_; ++outIndex) {
        size_t buffer_size = output_buffer_sizes_[outIndex];
        void *outputBuffer = nullptr;
        ret =
            aclrtMalloc(&outputBuffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            printf("aclrtMallocHost failed, ret[%d]\n", ret);
            return 1;
        }

        aclDataBuffer *outputData =
            aclCreateDataBuffer(outputBuffer, buffer_size);
        if (outputData == nullptr) {
            printf("aclCreateDataBuffer failed\n");
            return 1;
        }

        ret = aclmdlAddDatasetBuffer(output.get(), outputData);
        if (ret != ACL_ERROR_NONE) {
            printf("ACL_ModelOutputDataAdd failed, ret[%d]\n", ret);
            return 1;
        }
    }

    // inference
    chrono::steady_clock::time_point start;
    chrono::steady_clock::time_point end;
    int64_t cost_time = 0;
    for (uint32_t i = 0; i < loop_num; ++i) {
        start = chrono::steady_clock::now();
        ret = aclmdlExecute(model_id, input.get(), output.get());
        end = chrono::steady_clock::now();
        if (ret != ACL_ERROR_NONE) {
            printf("aclmdlExecute failed, ret[%d]\n", ret);
            return 1;
        }

        cost_time =
            chrono::duration_cast<chrono::microseconds>(end - start).count();

        cost_times->push_back(cost_time);
    }

    std::string outputBinFileNameIdx;
    size_t output_buffer_count = aclmdlGetDatasetNumBuffers(output.get());
    for (size_t outIndex = 0; outIndex < output_buffer_count; ++outIndex) {
        aclDataBuffer *dataBuffer =
            aclmdlGetDatasetBuffer(output.get(), outIndex);
        void *data = aclGetDataBufferAddr(dataBuffer);
        uint32_t len = aclGetDataBufferSize(dataBuffer);

        void *outHostData = nullptr;
        ret = aclrtMallocHost(&outHostData, len);
        if (ret != ACL_ERROR_NONE) {
            printf("aclrtMallocHost failed, ret[%d]\n", ret);
            return 1;
        }

        ret =
            aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_ERROR_NONE) {
            printf("aclrtMemcpy failed, ret[%d] line: %d\n", ret, __LINE__);
            return 1;
        }

        size_t pos = 0;
        len /= params_.batch_size;
        for (const auto &input_file : inputs[0]) {
            outputBinFileNameIdx = output_path;
            outputBinFileNameIdx +=
                input_file.substr(input_file.find_last_of('/') + 1);
            outputBinFileNameIdx += "_output_";
            outputBinFileNameIdx += std::to_string(outIndex);
            outputBinFileNameIdx += ".bin";
            LOG_DEBUG("write file:%s\n", outputBinFileNameIdx.c_str());
            FILE *fop = fopen(outputBinFileNameIdx.c_str(), "wb+");
            if (fop == nullptr) {
                printf("open file %s failed.\n", outputBinFileNameIdx.c_str());
                return 1;
            }
            size_t len1 =
                fwrite(((char *)outHostData) + pos, sizeof(char), len, fop);
            if (len1 != len) {
                printf("write output bin file failed!\n");
            }
            pos += len;
            fclose(fop);
        }

        // save inter result end
        ret = aclrtFreeHost(outHostData);
        if (ret != ACL_ERROR_NONE) {
            printf("aclrtFreeHost failed, ret[%d]\n", ret);
            return 1;
        }
    }
    LOG_DEBUG("\n");

    return 0;
}

void AclEngine::split(const std::string &s, std::vector<std::string> *tokens,
                      const std::string &delimiters) {

    string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (string::npos != pos || string::npos != lastPos) {
        tokens->emplace_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}

bool AclEngine::is_dir(const std::string &path) {
    struct stat s;
    if (stat(path.c_str(), &s) == 0) {
        if (S_ISDIR(s.st_mode)) {
            return true;
        }
    }

    return false;
}

bool AclEngine::is_file(const std::string &path) {
    struct stat s;
    if (stat(path.c_str(), &s) == 0) {
        if (S_ISREG(s.st_mode)) {
            return true;
        }
    }

    return false;
}

std::vector<std::string> AclEngine::GetFiles(const std::string &path) {
    std::vector<std::string> files;
    if (is_file(path)) {
        files.push_back(path);
        return files;
    }

    DIR *dir;
    struct dirent *ptr;
    string dir_path = path;
    string pattern;
    if (!is_dir(dir_path)) {
        auto index = path.find_last_of('/');
        dir_path = path.substr(0, index);
        if (index != string::npos) {
            pattern = path.substr(index + 1);
        }
        printf("path:%s, pattern:%s\n", dir_path.c_str(), pattern.c_str());
    }

    if ((dir = opendir(dir_path.c_str())) == nullptr) {
        printf("Open dir %s error.\n", path.c_str());
        return files;
    }
    string file_name;

    while ((ptr = readdir(dir)) != nullptr) {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) {
            // current dir OR parrent dir
            continue;
        }

        if (ptr->d_type == 10) {
            // link file
            continue;
        }

        if (ptr->d_type == 4) {
            // dir
            continue;
        }

        if (ptr->d_type == 8) {
            // file
            file_name = ptr->d_name;
        }

        if (match(file_name, pattern)) {
            files.emplace_back(dir_path + "/" + file_name);
        }
        file_name.clear();
    }

    closedir(dir);
    std::sort(files.begin(), files.end());

    return files;
}

bool AclEngine::match(const std::string &path, const std::string &pattern) {
    if (pattern.empty()) {
        return true;
    }

    return regex_match(path, regex(pattern));
}

void AclEngine::PreProcess(const std::string &fileName, void *&p_imgBuf,
                           size_t &pos, size_t &fileSize) {
    string one_line;
    std::vector<vector<string>> sentences;
    std::ifstream fcin(fileName, std::ifstream::in); // read hotel.decode.txt

    while (getline(fcin, one_line)) {
        vector<string> words = split_chinese(one_line);
        // delete last word (number)
        int n = words.size();
        while (!(words[n - 1] == "。" || words[n - 1] == "!" ||
                 words[n - 1] == "?")) {
            words.pop_back();
            n--;
        }
        sentences.push_back(words);
    }
    fcin.close();

    string wordVocabFileName =
        "../../models/snapshots/word_vocab.json";
    Vocab<string, int> vocab(wordVocabFileName);

    const int batchSize = 16;
    const int maxSentenceLen = 500;
    int w[maxSentenceLen][batchSize] = {0};

    int numSentence = sentences.size();
    if (numSentence > batchSize) {
        cout << "numSentence>batchSize, error!!!" << endl;
        return;
    }

    for (int j = 0; j < numSentence; ++j) {
        for (int i = 0; i < sentences[j].size(); ++i) {
            int id = vocab.w2i(sentences[j][i]); // w2i
            w[i][j] = id;
        }
    }

    // copy data, convert 2D int-type array to 1D char-type array
    fileSize /= maxSentenceLen; // fileSize = 32000/500 = 64
    for (int i = 0; i < maxSentenceLen; ++i) {
        memcpy((char *)p_imgBuf + pos, w[i], fileSize);
        pos += fileSize;
    }
    fileSize *= maxSentenceLen;
}
