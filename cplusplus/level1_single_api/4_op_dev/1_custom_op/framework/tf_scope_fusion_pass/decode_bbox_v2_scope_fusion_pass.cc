/**
 * Copyright 2020 Huawei Technologies Co., Ltd

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include "decode_bbox_v2_scope_fusion_pass.h"

#define OP_LOGE(OP_NAME, fmt, ...) printf("[ERROR]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#define OP_LOGW(OP_NAME, fmt, ...) printf("[WARN]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#define OP_LOGI(OP_NAME, fmt, ...) printf("[INFO]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)

namespace ge {
    namespace {
        const char *const kScopeType = "DecodeBboxV2FusionOp";
        const char *const kScopeTypeDecodeBboxV2 = "DecodeBboxV2";
        const char *const kOpType = "DecodeBboxV2";
    }  // namespace

    std::vector <ScopeFusionPatterns> DecodeBboxV2ScopeFusionPass::DefinePatterns() {
        std::vector <ScopeFusionPatterns> patterns_list;
        ScopeFusionPatterns pattern;
        GenScopePatterns(pattern);
        patterns_list.push_back(pattern);
        return patterns_list;
    }

    void DecodeBboxV2ScopeFusionPass::GenScopePatterns(ScopeFusionPatterns &patterns) {
        std::vector < ScopePattern * > batch;
        ScopePattern *decode_bbox_v2_pattern = new(std::nothrow) ScopePattern();
        if (decode_bbox_v2_pattern == nullptr) {
            OP_LOGE(kOpType, "Alloc an object failed.");
            return;
        }
        decode_bbox_v2_pattern->SetSubType(kScopeTypeDecodeBboxV2);
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Exp", 2, 0));        // Exp num is 2
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Mul", 4, 0));        // Mul num is 4
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Sub", 4, 0));        // Sub num is 4
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("RealDiv", 0, 2));    // RealDiv num is 2*n
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Unpack", 2, 0));     // Unpack num is 2
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Pack", 1, 0));       // Pack num is 1
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Transpose", 3, 0));  // Transpose num is 3
        decode_bbox_v2_pattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Softmax", -1, 0));   // doesn't have Softmax

        OP_LOGI(kOpType, "Add GenScopePatterns DecodeBboxV2.");
        batch.push_back(decode_bbox_v2_pattern);
        patterns.push_back(batch);
    }

    std::string DecodeBboxV2ScopeFusionPass::PassName() { return std::string("DecodeBboxV2ScopeFusionPass"); }

    Status DecodeBboxV2ScopeFusionPass::LastMatchScopesAndOPs(shared_ptr <ScopeGraph> &scope_graph,
                                                              std::vector <ScopesResult> &results) {
        OP_LOGI(kOpType, "LastMatchScopesAndOPs start.");
        if (scope_graph == nullptr) {
            OP_LOGE(kOpType, "Input params is nullptr.");
            return FAILED;
        }
        const ScopeTree *scope_tree = scope_graph->GetScopeTree();
        if (scope_tree == nullptr) {
            OP_LOGE(kOpType, "Scope tree is nullptr.");
            return FAILED;
        }
        const std::vector<Scope *> &scopes = scope_tree->GetAllScopes();

        for (auto &scope : scopes) {
            AscendString op_subtype;
            Status ret = scope->SubType(op_subtype);
            if (ret != SUCCESS) {
                return FAILED;
            }
            AscendString op_name;
            ret = scope->Name(op_name);
            if (ret != SUCCESS) {
                return FAILED;
            }
            // Class ScopeTree guarantees scope is not empty.
            if (op_subtype == kScopeTypeDecodeBboxV2) {
                OP_LOGI(kOpType, "DecodeBbox LastMatchScopesAndOPs match scope %s.", op_name.GetString());
                ScopesResult result;
                std::vector < Scope * > result_scopes;
                result_scopes.push_back(scope);
                result.SetScopes(result_scopes);
                results.push_back(result);
            }
        }
        return (!(results.empty())) ? SUCCESS : FAILED;
    }

    void DecodeBboxV2ScopeFusionPass::GenerateFusionResult(const std::vector<Scope *> &scopes,
                                                           FusionScopesResult *fusion_rlt) {
        if (fusion_rlt == nullptr) {
            return;
        }
        if (scopes.size() != 1) {
            fusion_rlt->SetType(kScopeInvalidType);
            return;
        }

        fusion_rlt->InsertInputs("transpose", {0, kFusionDisableIndex});
        fusion_rlt->InsertInputs("get_center_coordinates_and_sizes/transpose", {1, kFusionDisableIndex});
        fusion_rlt->InsertOutputs("transpose_1", {0});

        fusion_rlt->SetType(kScopeType);
        AscendString scope_name;
        Status ret = scopes[0]->Name(scope_name);
        if (ret != SUCCESS) {
            return ;
        }
        std::string str_scope_name;
        if (scope_name != nullptr) {
            str_scope_name = scope_name.GetString();
        }
        fusion_rlt->SetName(str_scope_name.substr(0, str_scope_name.length() - 1).c_str());
        fusion_rlt->SetDescription("");
        OP_LOGI(kOpType, "Set fusion result successfully.");
        return;
    }

    REGISTER_SCOPE_FUSION_PASS("DecodeBboxV2ScopeFusionPass", DecodeBboxV2ScopeFusionPass, false);
}  // namespace ge
