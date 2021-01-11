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

/*!
 * \file scope_batchmulticlass_nms_pass.h
 * \brief
 */
#ifndef OPS_BUILT_IN_FRAMEWORK_TF_SCOPE_FUSION_PASSES_SCOPE_BATCHMULTICLASS_NMS_PASS_H_
#define OPS_BUILT_IN_FRAMEWORK_TF_SCOPE_FUSION_PASSES_SCOPE_BATCHMULTICLASS_NMS_PASS_H_

#include <string>
#include <vector>

#include "register/scope/scope_fusion_pass_register.h"

namespace ge {
/**
 * @ingroup domi_omg
 * @brief ScopeBatchMultiClassNonMaxSuppressionPass
 */
class ScopeBatchMultiClassNonMaxSuppressionPass : public ScopeBasePass {
 protected:
  std::vector<ScopeFusionPatterns> DefinePatterns() override;
  std::string PassName() override;
  Status LastMatchScopesAndOPs(std::shared_ptr<ScopeGraph>& scope_graph, std::vector<ScopesResult>& results) override;
  void GenerateFusionResult(const std::vector<Scope*>& scopes, FusionScopesResult* fusion_rlt) override;

  void GenBoolmaskScopePatterns(ScopeFusionPatterns& patterns);
  void GenMapScopePatterns(ScopeFusionPatterns& patterns);
  void GenScopePatterns(ScopeFusionPatterns& patterns);
};
}  // namespace ge
#endif  // OPS_BUILT_IN_FRAMEWORK_TF_SCOPE_FUSION_PASSES_SCOPE_BATCHMULTICLASS_NMS_PASS_H_
