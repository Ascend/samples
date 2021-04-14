/**
 * Copyright 2020 Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "graph_utils.h"

#include <iostream>
#include <fstream>
#include <string.h>
#include <stdint.h>

static const std::string PATH = "../data/";
const int kDim1 = 16;
const int kDim2 = 64;
const string kVarConv2dWeight = "conv2d_weight";
const vector<std::string> kVarName = {"conv2d_weight", "accumulation", "learning_rate"};

// func defined by user.
uint32_t CallBack(uint32_t graph_id, const std::map<ge::AscendString, ge::Tensor>& params_list)
{
  map<ge::AscendString, ge::Tensor>::const_iterator iter;
  iter = params_list.find(AscendString(kVarConv2dWeight.c_str()));
  float step = 0.1;
  if (iter != params_list.end()) {
    float *gradient = const_cast<float*>(reinterpret_cast<const float*>(iter->second.GetData()));
    for (uint32_t i = 0; i< kDim1 * kDim2; ++i) {
      gradient[i] = (-1 - gradient[i])/step;
    }
    //  user can save gradient data to txt.
  } 
  return 0;
}

Status ReBuildGraph(uint32_t graph_id, ge::Session* session) {
  if (session == nullptr) {
    return FAILED;
  }

  TensorDesc desc(ge::Shape({16, 64, 1, 1}), FORMAT_NCHW,  DT_FLOAT);
  Status ret = FAILED;
  // check whether rebuild graph
  if (session->IsGraphNeedRebuild(graph_id))
  {
    ret = session->RemoveGraph(graph_id);
    if (ret != SUCCESS) {
      return FAILED;
    }
    Graph checkpoint_graph = Graph("CheckGraph2");
    ret = GenCheckpointGraph(checkpoint_graph, {{kVarConv2dWeight, desc}});
    if (ret != SUCCESS) {
      return FAILED;
    }
    ret = session->AddGraph(graph_id, checkpoint_graph);
    if (ret != SUCCESS) {
      return FAILED;
    }
  }
  std::vector<Tensor> input_check, output_check;
  ret = session->RunGraph(graph_id, input_check, output_check);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Session run checkpoint graph success."<<std::endl;
  return SUCCESS;
}

int main(int argc, char* argv[])
{
  // 0. System init
  std::map<AscendString, AscendString> config = {{"ge.exec.deviceId", "0"},
                                    {"ge.graphRunMode", "1"},
                                    {"ge.exec.precision_mode", "allow_fp32_to_fp16"}};
  Status ret = ge::GEInitialize(config);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Initialize ge success."<<std::endl;
  // 1. Generate graph
  TensorDesc desc(ge::Shape({16, 64, 1, 1}), FORMAT_NCHW,  DT_FLOAT);
  TensorDesc desc_label(ge::Shape({1,}), FORMAT_NCHW,  DT_FLOAT);
  std::vector<TensorDesc> var_tensor_desc =  {desc, desc, desc_label};
    // 1.1 init graph
  uint32_t init_graph_id = 0;
  Graph init_var_graph("InitVarGraph");
  ret = GenInitGraph(init_var_graph, var_tensor_desc, kVarName, {-1, 0.1, 0.1});
  if (ret != SUCCESS) {
    std::cout << "Generate init graph failed."<<std::endl;
    return FAILED;
  }
  std::cout<<"Generate init graph success."<<std::endl;
    // 1.2 checkpoint graph
  uint32_t checkpoint_graph_id = 1;
  Graph checkpoint_graph("CheckpointGraph");
  ret = GenCheckpointGraph(checkpoint_graph, {{kVarConv2dWeight, desc}});
  if (ret != SUCCESS) {
    std::cout << "Generate checkpoint graph failed."<<std::endl;
    return FAILED;
  }
  std::cout<<"Generate checkpoint graph success."<<std::endl;
    // 1.3 convolution graph
  uint32_t conv_graph_id = 2;
  Graph conv_graph("ConvolutionGraph");
  ret = GenConvGraph(conv_graph, var_tensor_desc, kVarName);
  if (ret != SUCCESS) {
    std::cout << "Generate convolution graph failed."<<std::endl;
    return FAILED;
  }
  std::cout<<"Generate convolution graph success."<<std::endl;
  // 2. creat session
  std::map < ge::AscendString, ge::AscendString > options;
  ge::Session *session = new Session(options);
  if (session == nullptr) {
    std::cout << "Create session failed." << std::endl;
    return FAILED;
  }
  std::cout<<"Create session success."<<std::endl;
  // users can register callback func with specific summary or checkpoint
  session->RegisterCallBackFunc("Save", CallBack);

  // 3. add graph
  ret = session->AddGraph(init_graph_id, init_var_graph);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Session add init graph success."<<std::endl;
  ret = session->AddGraph(checkpoint_graph_id, checkpoint_graph);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Session add checkpoint graph success."<<std::endl;
  ret = session->AddGraph(conv_graph_id, conv_graph);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Session add convolution graph success."<<std::endl;
  // 4. Run graph
  std::vector<Tensor> input_init, input_cov, input_check;
  std::vector<Tensor> output_init, output_cov, output_check;
  ret = session->RunGraph(init_graph_id, input_init, output_init);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Session run Init graph success."<<std::endl;
  ret = session->RunGraph(checkpoint_graph_id, input_check, output_check);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Session run checkpoint graph success."<<std::endl;
  vector<int64_t> input_shapes = {1, 64, 4, 4};
  string input_path = PATH + "ge_api_variable_input_x.bin";
  Tensor input_tensor;
  bool result = GetTensorFromBin(input_path, input_shapes, input_tensor);
  if (!result) {
    std::cout<<"Get tensor from bin failed."<<std::endl;
    return FAILED;
  }
  input_cov.push_back(input_tensor);
  ret = session->RunGraph(conv_graph_id, input_cov, output_cov);
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Session run convolution graph success."<<std::endl;

  // 5.Optional operation: If a graph is runned before, and want to run again, 
  // you need to check whether graph needed to rebuild,
  // so a graph is runned before, and needed to rebuild, user should remove it from GE first,
  // then add graph again and rebuild it.
  ret = ReBuildGraph(checkpoint_graph_id, session);
  if (ret != SUCCESS) {
    return FAILED;
  }

  delete session;

  // 6. system finalize
  ret = ge::GEFinalize();
  if (ret != SUCCESS) {
    return FAILED;
  }
  std::cout<<"Finalize ge success."<<std::endl;
  return 0;
}
