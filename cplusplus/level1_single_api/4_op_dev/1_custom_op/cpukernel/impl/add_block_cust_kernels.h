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
 
#ifndef _AICPU_ADD_BLOCK_CUST_KERNELS_H_
#define _AICPU_ADD_BLOCK_CUST_KERNELS_H_

#include "cpu_kernel.h"
#include "cpu_types.h"


namespace aicpu {
class AddBlockCpuKernel : public CpuKernel {
 public:
  ~AddBlockCpuKernel() = default;
  uint32_t Compute(CpuKernelContext &ctx) override;
  template<typename T>
  uint32_t AddCompute(CpuKernelContext &ctx);
  template<typename T>
  uint32_t AddComputeWithBlock(CpuKernelContext &ctx,
                               uint32_t blockid, uint32_t blockdim);
};
}  // namespace aicpu
#endif
