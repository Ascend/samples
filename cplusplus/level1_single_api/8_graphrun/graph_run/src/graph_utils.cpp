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

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace std;

uint32_t GetDataTypeSize(DataType dt)
{
    uint32_t dailation =1;
    if(dt == ge::DT_FLOAT)
        dailation = 4;
    else if(dt == ge::DT_FLOAT16)
        dailation = 2;
    else if(dt == ge::DT_INT16)
        dailation = 2;
    else if(dt == ge::DT_UINT16)
        dailation = 2;
    else if(dt == ge::DT_INT32)
        dailation = 4;
    else if(dt == ge::DT_UINT32)
        dailation = 4;
    else if(dt == ge::DT_INT64)
        dailation = 8;
    else if(dt == ge::DT_UINT64)
        dailation = 8;
    else if(dt == ge::DT_INT8)
        dailation = 1;

    return dailation;
}

// users can load data from data dir
bool GetTensorFromBin(string input_path, vector<int64_t> shapes, Tensor &input_tensor,
                      Format format, DataType data_type) {
  ifstream in_file(input_path.c_str(), std::ios::in | std::ios::binary);
  if (!in_file.is_open()) {
    std::cout << "Failed to open" << input_path.c_str() << '\n';
    return false;
  }
  in_file.seekg(0, ios_base::end);
  istream::pos_type file_size = in_file.tellg();
  in_file.seekg(0, ios_base::beg);

  size_t size = 1;
  for (uint32_t i = 0; i < shapes.size(); i++) {
    size *= shapes[i];
  }
  uint32_t data_len = size * GetDataTypeSize(data_type);
  if (data_len != file_size) {
    std::cout << "Invalid Param.len:" << data_len << " is not equal with binary size." << file_size << ")\n";
    in_file.close();
    return false;
  }
  char* pdata = new(std::nothrow) char[data_len];
  if (pdata == nullptr) {
    std::cout << "Invalid Param.len:" << data_len << " is not equal with binary size（" << file_size << ")\n";
    in_file.close();
    return false;
  }
  in_file.read(reinterpret_cast<char*>(pdata), data_len);
  TensorDesc input_tensor_desc = TensorDesc(ge::Shape(shapes), format, data_type);
  input_tensor_desc.SetRealDimCnt(shapes.size());
  input_tensor = Tensor(input_tensor_desc, reinterpret_cast<uint8_t*>(pdata), data_len);
  in_file.close();
  return true;
}

ge::Tensor GenTensor(std::vector<int64_t> tensor_shape, float value) {
  int64_t size = 1;
  for (size_t i = 0; i < tensor_shape.size(); i++) {
    size = size * tensor_shape[i];
  }

  float *data_value = new float[size];
  for (int i=0; i < size; i++) {
    *(data_value + i) = value;
  }
  Tensor tensor;
  TensorDesc input_tensor_desc = TensorDesc(ge::Shape(tensor_shape), FORMAT_NCHW);
  tensor.SetTensorDesc(input_tensor_desc);
  tensor.SetData((uint8_t*)data_value, size * 4);
  delete data_value;
  return tensor;
}

// generate init graph
int32_t GenInitGraph(Graph &graph, vector<TensorDesc> var_desc, const vector<string> &var_name,
                     const vector<float> &var_values) {
  std::vector<Operator> inputs{};
  std::vector<Operator> outputs{};

  for (size_t i = 0; i < var_desc.size(); i++) {
    var_desc[i].SetRealDimCnt(var_desc[i].GetShape().GetDimNum());
    auto tensor = GenTensor(var_desc[i].GetShape().GetDims(), var_values[i]);
    auto var_constant = op::Constant().set_attr_value(tensor);
    var_constant.update_output_desc_y(var_desc[i]);

    auto var_init = op::Variable(ge::AscendString(var_name[i].c_str()));
    var_init.update_output_desc_y(var_desc[i]);
    auto var_assign = op::Assign().set_input_ref(var_init).set_input_value(var_constant);
    inputs.push_back(var_init);
  }
  graph.SetInputs(inputs).SetOutputs(outputs);
  return SUCCESS;
}

// generate checkpoint graph
int32_t GenCheckpointGraph(Graph &graph, const map<string, TensorDesc> &variables) {
  std::vector<Operator> inputs{};
  std::vector<Operator> outputs{};

  for (auto &variable : variables) {
    auto var = op::Variable(ge::AscendString(variable.first.c_str()));
    var.update_output_desc_y(variable.second);
    inputs.push_back(var);
    graph.AddOp(var);
  }
  auto save = op::Save().create_dynamic_input_tensors(inputs.size());
  for (size_t i = 0; i < inputs.size(); i++) {
    save.set_dynamic_input_tensors(i, inputs[i]);
  }
  graph.SetInputs(inputs).SetOutputs(outputs);
  return SUCCESS;
}

void update_op_format(Operator op, Format format) {
  TensorDesc tensor_desc_x = op.GetInputDescByName("x");
  TensorDesc tensor_desc_y = op.GetInputDescByName("y");
  tensor_desc_x.SetFormat(format);
  tensor_desc_y.SetFormat(format);
  op.UpdateInputDesc("x", tensor_desc_x);
  op.UpdateOutputDesc("y", tensor_desc_y);
}

// generate convolution graph
int32_t GenConvGraph(Graph &graph, const vector<TensorDesc> &var_desc,
                     const vector<std::string> &var_name, Format format)
{
  auto data_x_shape = op::Data("xShape").set_attr_index(0);
  auto var_conv_weight = op::Variable(ge::AscendString(var_name[0].c_str()));
  auto var_accum = op::Variable(AscendString(var_name[1].c_str()));
  auto var_learn_rate = op::Variable(AscendString(var_name[2].c_str()));

  var_conv_weight.update_output_desc_y(var_desc[0]);
  var_accum.update_output_desc_y(var_desc[1]);
  var_learn_rate.update_output_desc_y(var_desc[2]);

  graph.AddOp(var_conv_weight);
  graph.AddOp(var_accum);
  graph.AddOp(var_learn_rate);

  auto conv2d = op::Conv2D().set_input_x(data_x_shape)
                            .set_input_filter(var_conv_weight)
                            .set_attr_strides({1, 1, 1, 1})
                            .set_attr_pads({0, 0, 0, 0});
  update_op_format(conv2d, format);
  ge::TensorDesc tensor_desc_w = conv2d.GetInputDescByName("filter");
  tensor_desc_w.SetFormat(format);
  conv2d.UpdateInputDesc("filter", tensor_desc_w);

  auto conv2d_grad = op::Conv2DBackpropFilterD("output_1").set_input_x(data_x_shape)
	               .set_attr_filter_size(var_desc[0].GetShape().GetDims())
		       .set_input_out_backprop(conv2d)
		       .set_attr_strides({1, 1, 1, 1})
		       .set_attr_pads({0, 0, 0, 0});
  update_op_format(conv2d_grad, format);
  graph.AddOp(conv2d_grad);
  auto optimizer = op::ApplyMomentum().set_input_accum(var_accum)
                                      .set_input_grad(conv2d_grad)
                                      .set_input_lr(var_learn_rate)
                                      .set_input_momentum(var_learn_rate)
                                      .set_input_var(var_conv_weight);
  graph.AddOp(optimizer);

  std::vector<Operator> inputs{data_x_shape};
  std::vector<Operator> outputs{conv2d};
  graph.SetInputs(inputs).SetOutputs(outputs);
  graph.AddOp(conv2d);

  return SUCCESS;
}

