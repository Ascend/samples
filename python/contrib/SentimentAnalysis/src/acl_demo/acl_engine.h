/**
 * Copyright (C)  2020. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0. You may not use this file except in compliance with the License.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * @file acl_engine.h
 *
 * @brief
 *
 * @version 1.0
 *
 */

#ifndef INFERENCE__ACL_ENGINE_H_
#define INFERENCE__ACL_ENGINE_H_

#include <acl/acl.h>

#include <getopt.h>
#include <unistd.h>
#include <sys/stat.h>

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <cstdio>
#include <algorithm>
#include <map>

#include "./data_process/data_process.h"

class AclEngine {
  struct Params {
    std::vector<std::string> input_files;
    std::string output_path;
    std::string model_file;
    uint32_t batch_size = 1;
    uint32_t loops = 1;
    bool is_help = false;
    bool only_run_one_batch = false;
    bool is_debug = false;
  };

 public:
  ~AclEngine();
  bool ParseParams(int argc, char **argv);
  bool Init();
  bool Inference();

  bool IsHelp() const {
    return params_.is_help;
  }

  void Usage(const char *program_name) const {
    printf("%s --model=xx.om --inputFile=xx[,yy] --outputFile=xx [--batchSize batch_size] [--loopNum 1] [--onlyOneBatch] [--debug]\n",
           program_name);
  }

 private:
  char *ReadBinFile(const std::string &file_name, uint32_t *p_file_size);

  void PreProcess(const string& fileName,void* &p_imgBuf,size_t& pos,size_t& fileSize);
  
  aclError InitAclDeviceContext();

  void DestroyAclModelDesc(uint32_t modelId, aclmdlDesc *modelDesc);

  void DestroyAclDeviceContext(uint32_t devNum,
                               const std::vector<aclrtContext> &context_vec);

  void ReleaseAllAclModelResource(uint32_t modelId, aclmdlDesc *modelDesc,
                                  const std::vector<aclrtContext> &contex_vec);

  int LoadModel();

  aclmdlDesc *aclModelLoadAndIOGet(const void *model, size_t model_size,
                                   uint32_t *model_id, void *dev_ptr,
                                   size_t mem_size,
                                   void *weight_ptr, size_t weight_size);

  int ReadInputFiles(const std::vector<std::vector<std::string>> &inputs);

  int AclInferenceProcess(uint32_t model_id,
                          std::vector<void *> inputBuf,
                          const std::vector<std::vector<std::string>> &inputs,
                          std::vector<int64_t> *cost_times);

  static std::vector<std::string> GetFiles(const std::string& path);


  static void split(const std::string &s, std::vector<std::string> *tokens,
                    const std::string &delimiters = ";");

  static bool is_dir(const std::string &path);

  static bool is_file(const std::string &path);

  static void ReleaseAclModelInput(aclmdlDataset *input);

  static void ReleaseAclModelOutput(aclmdlDataset *output);

  static bool match(const std::string& path, const std::string& pattern);

 private:
  const uint32_t kDeviceNum = 1;
  uint32_t model_id_ = 0;
  size_t mem_size_ = 0;
  size_t weight_size_ = 0;
  std::vector<aclrtContext> contexts_;
  char *model_data_ptr_ = nullptr;
  aclmdlDesc *model_desc_ptr_ = nullptr;
  std::vector<void *> batch_dsts_;
  void *device_ptr_ = nullptr;
  void *weight_ptr_ = nullptr;
  Params params_;
  size_t input_count_ = 0;
  size_t output_count_ = 0;
  std::vector<uint32_t> used_devices_;
  std::vector<uint64_t> input_buffer_sizes_;
  std::vector<uint64_t> output_buffer_sizes_;
  std::vector<void*> input_buffers_;
};

#endif //INFERENCE__ACL_ENGINE_H_
