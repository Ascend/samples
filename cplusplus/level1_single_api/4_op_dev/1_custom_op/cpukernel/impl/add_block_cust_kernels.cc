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
#include "add_block_cust_kernels.h"
#include <string>
#include <cmath>

#include "cpu_tensor.h"
#include "cpu_tensor_shape.h"
#include "cpu_types.h"

namespace {
const char* ADD_BLOCK_CUST = "AddBlockCust";
const uint32_t kFirstInputIndex = 0;
const uint32_t kSecondInputIndex = 1;
const uint32_t kFirstOutputIndex = 0;
const uint32_t SUCCESS = 0;
const uint32_t PARAM_INVAILD = 1;
const uint32_t ERROR = 2;
}

namespace aicpu {
uint32_t AddBlockCpuKernel::Compute(CpuKernelContext &ctx) {
  Tensor *input0 = ctx.Input(kFirstInputIndex);
  Tensor *input1 = ctx.Input(kSecondInputIndex);
  std::string opType = ctx.GetOpType();
  if (input0->GetDataSize() == 0 || input1->GetDataSize() == 0) {
    return SUCCESS;
  }

  auto data_type = static_cast<DataType>(input0->GetDataType());
  switch(data_type) {
    case DT_FLOAT:
      return AddCompute<float>(ctx);
    case DT_INT32:
      return AddCompute<int32_t>(ctx);
    case DT_INT64:
      return AddCompute<int64_t>(ctx);
    default:
      return PARAM_INVAILD;
  }
  return SUCCESS;
}

template<typename T> 
uint32_t AddBlockCpuKernel::AddCompute(CpuKernelContext &ctx)
{
  //Get blockid and blockdim
  uint32_t blockid;
  uint32_t blockdim;
  AttrValue *block_id_ptr = ctx.GetAttr("block_id");
  AttrValue *block_dim_ptr = ctx.GetAttr("block_num");
  //check block_id and block_num
  if (block_id_ptr == nullptr || block_dim_ptr == nullptr) {
    blockid = 0;
    blockdim = 1;
  } else {
    blockid = block_id_ptr->GetInt();
    blockdim = block_dim_ptr->GetInt();
  }
  if (blockid >= blockdim || blockid < 0) {
    blockid = 0;
    blockdim = 1;
  }

  return AddComputeWithBlock<T>(ctx, blockid, blockdim);
}
template<typename T> 
uint32_t AddBlockCpuKernel::AddComputeWithBlock(CpuKernelContext &ctx,
                                                uint32_t blockid, uint32_t blockdim)
{
  Tensor *input0 = ctx.Input(kFirstInputIndex);
  Tensor *input1 = ctx.Input(kSecondInputIndex);
  Tensor *output = ctx.Output(kFirstOutputIndex);

  T *x0 = reinterpret_cast<T *>(input0->GetData());
  if (x0 == nullptr) {
    return PARAM_INVAILD;
  }
  T *x1 = reinterpret_cast<T *>(input1->GetData());
  if (x1 == nullptr) {
    return PARAM_INVAILD;
  }
  T *y = reinterpret_cast<T *>(output->GetData());
  if (y == nullptr) {
    return PARAM_INVAILD;
  }

  // caculate per unit if blockdimByIndex = -1
  int64_t total = input0->NumElements();
  int64_t startpos = 0;
  int64_t len = total;
  if (blockdim != 1) {
    uint32_t per_unit = std::ceil(total / blockdim);
    startpos =  blockid * per_unit;
    len = blockid < blockdim - 1 ? per_unit : (total - per_unit * (blockdim - 1));
  }

  for (int i = startpos; i < startpos + len; i++) {
    y[i] = x0[i] + x1[i];
  }
  return SUCCESS;
}

REGISTER_CPU_KERNEL(ADD_BLOCK_CUST, AddBlockCpuKernel);

}  // namespace aicpu
