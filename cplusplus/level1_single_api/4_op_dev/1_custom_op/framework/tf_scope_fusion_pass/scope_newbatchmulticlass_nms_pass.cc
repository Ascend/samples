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
 * \file scope_newbatchmulticlass_nms_pass.cc
 * \brief scope fusion of ToAbsoluteBBox
 */
#include "scope_newbatchmulticlass_nms_pass.h"

#define OP_LOGE(OP_NAME, fmt, ...) printf("[ERROR]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#define OP_LOGW(OP_NAME, fmt, ...) printf("[WARN]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
//#define OP_LOGI(OP_NAME, fmt, ...) printf("[INFO]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#include "register/scope/scope_fusion_pass_register.h"
#include <string.h>

namespace ge {

std::vector<ScopeFusionPatterns> ScopeBatchMultiClassNMSPass::DefinePatterns() {
  std::vector<ScopeFusionPatterns> patterns;
  patterns.push_back(GenWhileScopePatterns());
  return patterns;
}

Status ScopeBatchMultiClassNMSPass::LastMatchScopesAndOPs(std::shared_ptr<ScopeGraph>& scope_graph,
                                                          std::vector<ScopesResult>& results) {
  if (!scope_graph) {
    OP_LOGE(kOpType.c_str(), "Input params is nullptr.");
    return domi::PARAM_INVALID;
  }

  const ScopeTree* scope_tree = scope_graph->GetScopeTree();
  if (scope_tree == nullptr) {
    OP_LOGE(kOpType.c_str(), "Scope tree is nullptr.");
    return domi::PARAM_INVALID;
  }

  const std::vector<Scope*>& scopes = scope_tree->GetAllScopes();
  for (auto& scope : scopes) {
    AscendString op_name;
    Status ret = scope->Name(op_name);
    if (ret != SUCCESS) {
        return FAILED;
    }
    if (op_name == "rootmap/") {
      continue;
    }
    ret = scope->LastName(op_name);
    if (ret != SUCCESS) {
        return FAILED;
    }
    if (op_name.GetString() == kScopeMap) {
      const Scope* fatherScope = scope->GetFatherScope();
      if (fatherScope == nullptr) {
        continue;
      }

      if (!MatchedSubScopes(fatherScope, {"map/", "while/", "MultiClassNonMaxSuppression/"})) {
        continue;
      }
      printf("find scope name: %s", op_name.GetString());
      ScopesResult result;
      std::vector<Scope*> result_scopes;
      result_scopes.push_back(const_cast<Scope*>(fatherScope));
      result.SetScopes(result_scopes);
      results.push_back(result);
    }
  }

  return (!(results.empty())) ? SUCCESS : FAILED;
}

void ScopeBatchMultiClassNMSPass::GenerateFusionResult(const std::vector<Scope*>& scopes,
                                                       FusionScopesResult* fusion_rlt) {
  if (fusion_rlt == nullptr) {
    OP_LOGE(kOpType.c_str(), "Input fusion_rlt is nullptr.");
    return;
  }
  for (auto& scope : scopes) {
    AscendString op_name;
    Status ret = scope->Name(op_name);
    // do insert input 0 boxes like map/TensorArrayUnstack/Shape
    char * scopeInputName0 = new char[200];
    memset(scopeInputName0, 0, sizeof(scopeInputName0));
    strncpy(scopeInputName0, op_name.GetString(), strlen(op_name.GetString())+1);
    strncat(scopeInputName0, "map/TensorArrayUnstack/Shape", strlen("map/TensorArrayUnstack/Shape")+1);
    fusion_rlt->InsertInputs(scopeInputName0, {0});

    // do insert input 1 scores like map/TensorArrayUnstack_1/Shape
    char * scopeInputName1 = new char[200];
    memset(scopeInputName1, 0, sizeof(scopeInputName1));
    strncpy(scopeInputName1, op_name.GetString(), strlen(op_name.GetString())+1);
    strncat(scopeInputName1, "map/TensorArrayUnstack_1/Shape", strlen("map/TensorArrayUnstack_1/Shape")+1);
    fusion_rlt->InsertInputs(scopeInputName1, {1});

    // do insert input 2 clip_to_window like map/TensorArrayUnstack_3/Shape
    if (MatchedSubScopes(scope, {"map/", "TensorArrayUnstack_3/"})) {
      char * scopeInputName2 = new char[200];
      memset(scopeInputName2, 0, sizeof(scopeInputName2));
      strncpy(scopeInputName2, op_name.GetString(), strlen(op_name.GetString())+1);
      strncat(scopeInputName2, "map/TensorArrayUnstack_3/Shape", strlen("map/TensorArrayUnstack_3/Shape")+1);
      fusion_rlt->InsertInputs(scopeInputName2, {2});
      if (MatchedSubScopes(scope, {"map/", "TensorArrayUnstack_4/"}) && !MatchedSubScopes(scope, {"ones/"})) {
        char * scopeInputName3 = new char[200];
        memset(scopeInputName3, 0, sizeof(scopeInputName3));
        strncpy(scopeInputName3, op_name.GetString(), strlen(op_name.GetString())+1);
        strncat(scopeInputName3, "map/TensorArrayUnstack_4/Shape", strlen("map/TensorArrayUnstack_4/Shape")+1);
        fusion_rlt->InsertInputs(scopeInputName3, {3});
      }
    }
    // do insert outputs
    if (MatchedSubScopes(scope, {"map/", "TensorArrayStack/"})) {
      char * scopeOutputName0 = new char[200];
      memset(scopeOutputName0, 0, sizeof(scopeOutputName0));
      strncpy(scopeOutputName0, op_name.GetString(), strlen(op_name.GetString())+1);
      strncat(scopeOutputName0, "map/TensorArrayStack/TensorArrayGatherV3", strlen(
                              "map/TensorArrayStack/TensorArrayGatherV3")+1);
      fusion_rlt->InsertOutputs(scopeOutputName0, {0});
    }
    if (MatchedSubScopes(scope, {"map/", "TensorArrayStack_1/"})) {
      char * scopeOutputName0 = new char[200];
      memset(scopeOutputName0, 0, sizeof(scopeOutputName0));
      strncpy(scopeOutputName0, op_name.GetString(), strlen(op_name.GetString())+1);
      strncat(scopeOutputName0, "map/TensorArrayStack_1/TensorArrayGatherV3", strlen(\
                              "map/TensorArrayStack_1/TensorArrayGatherV3")+1);
      fusion_rlt->InsertOutputs(scopeOutputName0, {1});
    }
    if (MatchedSubScopes(scope, {"map/", "TensorArrayStack_2/"})) {
      char * scopeOutputName0 = new char[200];
      memset(scopeOutputName0, 0, sizeof(scopeOutputName0));
      strncpy(scopeOutputName0, op_name.GetString(), strlen(op_name.GetString())+1);
      strncat(scopeOutputName0, "map/TensorArrayStack_2/TensorArrayGatherV3", strlen(
                              "map/TensorArrayStack_2/TensorArrayGatherV3")+1);
      fusion_rlt->InsertOutputs(scopeOutputName0, {2});
    }
    if (MatchedSubScopes(scope, {"map/", "TensorArrayStack_4/"})) {
      char * scopeOutputName0 = new char[200];
      memset(scopeOutputName0, 0, sizeof(scopeOutputName0));
      strncpy(scopeOutputName0, op_name.GetString(), strlen(op_name.GetString())+1);
      strncat(scopeOutputName0, "map/TensorArrayStack_4/TensorArrayGatherV3", strlen(
                              "map/TensorArrayStack_4/TensorArrayGatherV3")+1);
      fusion_rlt->InsertOutputs(scopeOutputName0, {3});
    }
    fusion_rlt->SetType(kOpType.c_str());
    fusion_rlt->SetDescription("");
    std::string scopeName = op_name.GetString();
    fusion_rlt->SetName((scopeName.substr(0, scopeName.length() - 1)).c_str());
  }
  printf("ScopeBatchMultiClassNonMaxSuppressionPass Scope fusion success.");
  return;
}

ScopeFusionPatterns ScopeBatchMultiClassNMSPass::GenWhileScopePatterns() {
  ScopePattern* while_cell = new (std::nothrow) ScopePattern();
  if (while_cell == nullptr) {
    OP_LOGE(kOpType.c_str(), "Alloc an object failed.");
    return ScopeFusionPatterns();
  }

  while_cell->SetSubType(kScopeResultType.c_str());
  while_cell->AddScopeFeature(ScopeFeature("", 1, "", "map"));
  while_cell->AddScopeFeature(ScopeFeature("", 1, "", "while"));
  while_cell->AddScopeFeature(ScopeFeature("", 1, "", "MultiClassNonMaxSuppression"));

  ScopeFusionPatterns while_scope_pattern = {{while_cell}};
  return while_scope_pattern;
}

string ScopeBatchMultiClassNMSPass::to_string(const std::vector<Scope*>& scopes) const {
  string result;
  for (auto& scope : scopes) {
    AscendString op_name;
    Status ret = scope->Name(op_name);
    result += op_name.GetString();
    result += " ";
  }
  return result;
}

bool ScopeBatchMultiClassNMSPass::MatchedSubScopes(const Scope* root_scope,
                                                   const std::vector<string> scopes2check) const {
  string full_name;
  auto root = root_scope;
  std::string Str;
  for (auto& scope_name : scopes2check) {
    AscendString op_name;
    Status ret = root->Name(op_name);
    full_name = op_name.GetString();
    full_name += scope_name;
    auto sub_scope = root->GetSubScope(full_name.c_str());
    if (sub_scope == nullptr) {
      printf("Get sub scope:%s failed, %s's sub scopes:%s", full_name.c_str(), op_name.GetString(),
        to_string(root->GetAllSubScopes()).c_str());
      return false;
    }
    root = sub_scope;
  }
  AscendString op_name;
  Status ret = root->Name(op_name);
  printf("MatchedSubScopes:%s success.", op_name.GetString());
  return true;
}

}  // namespace ge
