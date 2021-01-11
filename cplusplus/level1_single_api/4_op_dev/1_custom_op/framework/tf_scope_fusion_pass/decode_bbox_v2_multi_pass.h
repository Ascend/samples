/* Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.
 * You may not use this file except in compliance with the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef FRAMEWORK_TF_SCOPE_FUSION_PASS_DECODE_BBOX_V2_MULTI_PASS_H_
#define FRAMEWORK_TF_SCOPE_FUSION_PASS_DECODE_BBOX_V2_MULTI_PASS_H_

#include <string>
#include <vector>
#include "register/scope/scope_fusion_pass_register.h"

namespace ge {
    class DecodeBboxV2MultiScopeFusionPass : public ScopeBasePass {
    protected:
        std::vector <ScopeFusionPatterns> DefinePatterns() override;

        std::string PassName() override;

        Status
        LastMatchScopesAndOPs(std::shared_ptr <ScopeGraph> &scope_graph, std::vector <ScopesResult> &results) override;

        void GenerateFusionResult(const std::vector<Scope *> &scopes, FusionScopesResult *fusion_rlt) override;

    private:
        void GenScopePatterns(ScopeFusionPatterns &patterns);
    };
}  // namespace ge
#endif  // FRAMEWORK_TF_SCOPE_FUSION_PASS_DECODE_BBOX_V2_MULTI_PASS_H_
