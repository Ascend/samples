/* Copyright (c) Huawei Technologies Co., Ltd. 2021. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "add_block_cust.h"
#include <vector>
#include <string>
#include <iostream>

namespace ge {
bool InferShapeAndTypeAddBlock(Operator& op, const string& input_name1, const string& input_name2, const string& output_name) {

  TensorDesc vOutputDesc = op.GetOutputDescByName(output_name.c_str());

  DataType input_dtype = op.GetInputDescByName(input_name1.c_str()).GetDataType();
  Format input_format = op.GetInputDescByName(input_name1.c_str()).GetFormat();
  // 针对shape维度大小进行交换
  ge::Shape shapeX = op.GetInputDescByName(input_name1.c_str()).GetShape();
  ge::Shape shapeY = op.GetInputDescByName(input_name2.c_str()).GetShape();
  std::vector<int64_t> dimsX = shapeX.GetDims();
  std::vector<int64_t> dimsY = shapeY.GetDims();
  if (dimsX.size() < dimsY.size()) {
    std::vector<int64_t> dimsTmp = dimsX;
    dimsX = dimsY;
    dimsY = dimsTmp;
  }

  // 对小的shape进行1补齐
  if (dimsX.size() != dimsY.size()) {
    int dec = dimsX.size() - dimsY.size();
    for (int i = 0; i < dec; i++) {
      dimsY.insert(dimsY.begin(), (int64_t)1);
    }
  }

  // 设置输出的shape维度
  std::vector<int64_t> dimVec;
  for (size_t i = 0; i < dimsX.size(); i++) {
    if ((dimsX[i] != dimsY[i]) && (dimsX[i] != 1) && (dimsY[i] != 1) && (dimsX[i] != -1) && (dimsY[i] != -1)) {
      return false;
    }
    
    if ((dimsX[i] == -1) && (dimsY[i] != -1)) {
      if (dimsY[i] > 1) {
        int64_t dims = dimsX[i] > dimsY[i] ? dimsX[i] : dimsY[i];
        dimVec.push_back(dims);
      } else if (dimsY[i] == 1) {
        int64_t dims = dimsX[i] > dimsY[i] ? dimsX[i] : dimsY[i];
        dimVec.push_back(dims);
        dimVec[i] = -1;
      } else if ((dimsY[i] == 0) || (dimsX[i] == 0)) {
        dimVec.push_back(0);
      }
    } else if ((dimsX[i] != -1) && (dimsY[i] == -1)) {
      if (dimsX[i] > 1) {
        int64_t dims = dimsX[i] > dimsY[i] ? dimsX[i] : dimsY[i];
        dimVec.push_back(dims);
      } else if (dimsX[i] == 0) {
        dimVec.push_back(0);
      } else if (dimsX[i] == 1) {
        int64_t dims = dimsX[i] > dimsY[i] ? dimsX[i] : dimsY[i];
        dimVec.push_back(dims);
        dimVec[i] = -1;
      }
    } else {
      if ((dimsX[i] == -1) && (dimsY[i] == -1)) {
        int64_t dims = dimsX[i] > dimsY[i] ? dimsX[i] : dimsY[i];
        dimVec.push_back(dims);
        dimVec[i] = -1;
      } else {
        if (dimsY[i] == 0 || dimsX[i] == 0) {
          dimVec.push_back(0);
        } else {
          int64_t dims = dimsX[i] > dimsY[i] ? dimsX[i] : dimsY[i];
          dimVec.push_back(dims);
        }
      }
    }
  }
  ge::Shape outputShape = ge::Shape(dimVec);

  vOutputDesc.SetShape(outputShape);
  vOutputDesc.SetDataType(input_dtype);
  vOutputDesc.SetFormat(input_format);
  op.UpdateOutputDesc(output_name.c_str(), vOutputDesc);

  return true;
}

// Obtains the processing function of the output tensor description.
IMPLEMT_COMMON_INFERFUNC(AddBlockCustInfer)
{
  if(InferShapeAndTypeAddBlock(op, "x1", "x2", "y")) {
     return GRAPH_SUCCESS;
  }
  return GRAPH_FAILED;
}

COMMON_INFER_FUNC_REG(AddBlockCust, AddBlockCustInfer);
}
