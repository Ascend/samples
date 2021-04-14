/**
 * Copyright (C)  2019. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use this file except in compliance with the License.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * @file lstm_tik.cpp
 *
 * @brief
 *
 * @version 1.0
 *
 */
#include "./lstm_tik.h"
#include <string>
#include <vector>

namespace ge {

bool InferShapeAndTypeLSTMTik(Operator& op, const string& output_h_name, const string& output_c_name) {
  // simple example thus we just provided a fixed shape kernel
  const int64_t hiddenSize = 32;
  const int64_t featureSize = 32;
  const int64_t blockSize = 16;
  const int64_t featureBlockNum = featureSize / blockSize;
  const int64_t hiddenBlockSize = hiddenSize / blockSize;
  const int64_t batchSize = 32;
  const int64_t batchBlockNum = batchSize / blockSize;
  const int64_t numStep = 16;

  TensorDesc vOutputHDesc = op.GetOutputDescByName(output_h_name.c_str());
  std::vector<int64_t> dimHVec = {numStep, batchBlockNum, hiddenBlockSize, blockSize, blockSize};
  ge::Shape outputHShape = ge::Shape(dimHVec);

  vOutputHDesc.SetShape(outputHShape);
  vOutputHDesc.SetDataType(DT_FLOAT16);
  vOutputHDesc.SetFormat(FORMAT_ND);
  op.UpdateOutputDesc(output_h_name.c_str(), vOutputHDesc);

  TensorDesc vOutputCDesc = op.GetOutputDescByName(output_c_name.c_str());

  std::vector<int64_t> dimCVec = {batchBlockNum, hiddenBlockSize, blockSize, blockSize};
  ge::Shape outputCShape = ge::Shape(dimCVec);

  vOutputCDesc.SetShape(outputCShape);
  vOutputCDesc.SetDataType(DT_FLOAT16);
  vOutputCDesc.SetFormat(FORMAT_ND);
  op.UpdateOutputDesc(output_c_name.c_str(), vOutputCDesc);

  return true;
}

//----------------LSTMTik-------------------
IMPLEMT_VERIFIER(LSTMTik, LSTMTikVerify)
{
  return GRAPH_SUCCESS;
}

// Obtains the processing function of the output tensor description.
IMPLEMT_COMMON_INFERFUNC(LSTMTikInferShape)
{
  if(InferShapeAndTypeLSTMTik(op, "output_h", "output_c")) {
     return GRAPH_SUCCESS;
  }
  return GRAPH_FAILED;
}

//Registered inferfunction
COMMON_INFER_FUNC_REG(LSTMTik, LSTMTikInferShape);

//Registered verify function
VERIFY_FUNC_REG(LSTMTik, LSTMTikVerify);
//----------------LSTMTik-------------------
}
