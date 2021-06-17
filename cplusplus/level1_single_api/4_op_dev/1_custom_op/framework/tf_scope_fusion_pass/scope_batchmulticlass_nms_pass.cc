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
 * \file scope_batchmulticlass_nms_pass.cc
 * \brief
 */
#include "scope_batchmulticlass_nms_pass.h"

#define OP_LOGE(OP_NAME, fmt, ...) printf("[ERROR]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#define OP_LOGW(OP_NAME, fmt, ...) printf("[WARN]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#define OP_LOGI(OP_NAME, fmt, ...) printf("[INFO]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#include "register/scope/scope_fusion_pass_register.h"

namespace ge {
namespace {
const char* const kScopeType = "BatchMultiClassNonMaxSuppression";
const char* const kScopeTypeBatchMultiClassNonMaxSuppression = "BatchMultiClassNonMaxSuppression";
const char* const kScopeTypeSecondBatchMultiClassNonMaxSuppression = "BatchSecondhMultiClassNonMaxSuppression";
const char* const kScopeTypeFaceBoxesBatchMultiClassNonMaxSuppression = "BatchFaceBoxeshMultiClassNonMaxSuppression";
const char* const kScopeTypeFiltereBatchMultiClassNonMaxSuppression = "BatchFiltereMultiClassNonMaxSuppression";
const char* const kOpType = "BatchMultiClassNonMaxSuppression";
}  // namespace

std::vector<ScopeFusionPatterns> ScopeBatchMultiClassNonMaxSuppressionPass::DefinePatterns() {
  OP_LOGI(kOpType, "ScopeBatchMultiClassNonMaxSuppressionPass start");
  std::vector<ScopeFusionPatterns> patterns_list;
  ScopeFusionPatterns pattern;
  GenScopePatterns(pattern);  // match fastrcnn Scope
  patterns_list.push_back(pattern);
  return patterns_list;
}

std::string ScopeBatchMultiClassNonMaxSuppressionPass::PassName() {
  return std::string("ScopeBatchMultiClassNonMaxSuppressionPass");
}

/**
 * @brief LastMatch for multiple scopes
 */
Status ScopeBatchMultiClassNonMaxSuppressionPass::LastMatchScopesAndOPs(std::shared_ptr<ScopeGraph>& scope_graph,
                                                                        std::vector<ScopesResult>& results) {
  if (scope_graph == nullptr) {
    OP_LOGE(kOpType, "Input params is nullptr.");
    return domi::PARAM_INVALID;
  }
  const ScopeTree* scope_tree = scope_graph->GetScopeTree();
  if (scope_tree == nullptr) {
    OP_LOGE(kOpType, "Scope tree is nullptr.");
    return domi::PARAM_INVALID;
  }
  const std::vector<Scope*>& scopes = scope_tree->GetAllScopes();

  for (auto& scope : scopes) {
    // Class ScopeTree guarantees scope is not empty.
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
    if (op_subtype == kScopeTypeBatchMultiClassNonMaxSuppression) {
      OP_LOGI(kOpType, "BatchMultiClassNonMaxSuppression LastMatchScopesAndOPs match SubType.");
      ScopesResult result;
      std::vector<Scope*> result_scopes;
      result_scopes.push_back(scope);
      result.SetScopes(result_scopes);
      results.push_back(result);
    }
    if (op_subtype == kScopeTypeSecondBatchMultiClassNonMaxSuppression) {
      OP_LOGI(kOpType, "SecondBatchMultiClassNonMaxSuppression LastMatchScopesAndOPs match SubType.");
      ScopesResult result;
      std::vector<Scope*> result_scopes;
      result_scopes.push_back(scope);
      result.SetScopes(result_scopes);
      results.push_back(result);
    }
    if (op_subtype == kScopeTypeFaceBoxesBatchMultiClassNonMaxSuppression) {
      OP_LOGI(kOpType, "FaceBoxesBatchMultiClassNonMaxSuppression LastMatchScopesAndOPs match SubType.");
      ScopesResult result;
      std::vector<Scope*> result_scopes;
      result_scopes.push_back(scope);
      result.SetScopes(result_scopes);
      results.push_back(result);
    }
    if (op_subtype == kScopeTypeFiltereBatchMultiClassNonMaxSuppression) {
      OP_LOGI(kOpType, "FiltereBatchMultiClassNonMaxSuppression LastMatchScopesAndOPs match SubType.");
      ScopesResult result;
      std::vector<Scope*> result_scopes;
      result_scopes.push_back(scope);
      result.SetScopes(result_scopes);
      results.push_back(result);
    }
  }
  return (!(results.empty())) ? SUCCESS : FAILED;
}

void ScopeBatchMultiClassNonMaxSuppressionPass::GenScopePatterns(ScopeFusionPatterns& patterns) {
  // match batchnorm
  std::vector<ScopePattern*> batch1;
  ScopePattern* ScopeBatchMultiClassNMSPattern = new (std::nothrow) ScopePattern();
  if (ScopeBatchMultiClassNMSPattern == nullptr) {
    OP_LOGE(kOpType, "Alloc an object failed.");
    return;
  }
  ScopeBatchMultiClassNMSPattern->SetSubType(kScopeTypeBatchMultiClassNonMaxSuppression);
  ScopeBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("NonMaxSuppressionV2", 1, 0));
  ScopeBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Maximum", 4, 0));
  OP_LOGI(kOpType, "Add GenScopePatterns ScopeBatchMultiClassNMSPattern.");
  batch1.push_back(ScopeBatchMultiClassNMSPattern);

  ScopePattern* ScopeSecondBatchMultiClassNMSPattern = new (std::nothrow) ScopePattern();
  if (ScopeSecondBatchMultiClassNMSPattern == nullptr) {
    OP_LOGE(kOpType, "Alloc an object failed.");
    return;
  }
  ScopeSecondBatchMultiClassNMSPattern->SetSubType(kScopeTypeSecondBatchMultiClassNonMaxSuppression);
  ScopeSecondBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("NonMaxSuppressionV2", 0, 1));
  ScopeSecondBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Merge", 11, 0));
  ScopeSecondBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Transpose", -1, 0));
  OP_LOGI(kOpType, "Add GenScopePatterns ScopeSecondBatchMultiClassNMSPattern.");
  batch1.push_back(ScopeSecondBatchMultiClassNMSPattern);

  ScopePattern* ScopeFaceBoxesBatchMultiClassNMSPattern = new (std::nothrow) ScopePattern();
  if (ScopeFaceBoxesBatchMultiClassNMSPattern == nullptr) {
    OP_LOGE(kOpType, "Alloc an object failed.");
    return;
  }
  ScopeFaceBoxesBatchMultiClassNMSPattern->SetSubType(kScopeTypeFaceBoxesBatchMultiClassNonMaxSuppression);
  ScopeFaceBoxesBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("NonMaxSuppressionV3", 0, 1));
  ScopeFaceBoxesBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Transpose", -1, 0));
  OP_LOGI(kOpType, "Add GenScopePatterns ScopeFaceBoxesBatchMultiClassNMSPattern.");
  batch1.push_back(ScopeFaceBoxesBatchMultiClassNMSPattern);

  ScopePattern* ScopeFilteredBatchMultiClassNMSPattern = new (std::nothrow) ScopePattern();
  if (ScopeFilteredBatchMultiClassNMSPattern == nullptr) {
    OP_LOGE(kOpType, "Alloc an object failed.");
    return;
  }
  ScopeFilteredBatchMultiClassNMSPattern->SetSubType(kScopeTypeFiltereBatchMultiClassNonMaxSuppression);
  ScopeFilteredBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("NonMaxSuppressionV3", 0, 1));
  ScopeFilteredBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Range", 0, 5));
  ScopeFilteredBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("ConcatV2", 0, 1));
  ScopeFilteredBatchMultiClassNMSPattern->AddNodeOpTypeFeature(NodeOpTypeFeature("Fill", 0, 80));

  OP_LOGI(kOpType, "Add GenScopePatterns ScopeFilteredBatchMultiClassNMSPattern.");
  batch1.push_back(ScopeFilteredBatchMultiClassNMSPattern);
  patterns.push_back(batch1);
}

void ScopeBatchMultiClassNonMaxSuppressionPass::GenerateFusionResult(const std::vector<Scope*>& scopes,
                                                                     FusionScopesResult* fusion_rlt) {
  if (fusion_rlt == nullptr) {
    OP_LOGE(kOpType, "Input fusion_rlt is nullptr.");
    return;
  }

  for (auto& scope : scopes) {
    AscendString op_subtype;
    Status ret = scope->SubType(op_subtype);
    if (op_subtype == kScopeTypeBatchMultiClassNonMaxSuppression) {
      fusion_rlt->InsertInputs("BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack/Shape", {0});
      fusion_rlt->InsertInputs("BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack_1/Shape", {1});
      fusion_rlt->InsertInputs("BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack_3/Shape", {2});
      fusion_rlt->InsertOutputs("BatchMultiClassNonMaxSuppression/map/TensorArrayStack/TensorArrayGatherV3", {0});
      fusion_rlt->InsertOutputs("BatchMultiClassNonMaxSuppression/map/TensorArrayStack_4/TensorArrayGatherV3", {3});
      fusion_rlt->SetType(kScopeType);
      fusion_rlt->SetDescription("");
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
    }
    if (op_subtype == kScopeTypeSecondBatchMultiClassNonMaxSuppression) {
      fusion_rlt->InsertInputs("SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack/Shape",
                               {0});
      fusion_rlt->InsertInputs(
          "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack_1/Shape", {1});
      fusion_rlt->InsertInputs(
          "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack_4/Shape", {3});
      fusion_rlt->InsertInputs(
          "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack_3/Shape", {2});
      fusion_rlt->InsertOutputs(
          "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayStack/TensorArrayGatherV3", {0});
      fusion_rlt->InsertOutputs(
          "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayStack_1/TensorArrayGatherV3", {1});
      fusion_rlt->InsertOutputs(
          "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayStack_2/TensorArrayGatherV3", {2});
      fusion_rlt->InsertOutputs(
          "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression/map/TensorArrayStack_4/TensorArrayGatherV3", {3});
      fusion_rlt->SetType(kScopeType);
      fusion_rlt->SetDescription("");
      AscendString scope_name;
      Status ret = scopes[0]->Name(scope_name);
      if (ret != SUCCESS) {
          return ;
      }
      std::string str_scope_name;
      if (scope_name != nullptr){
          str_scope_name = scope_name.GetString();
      }
      fusion_rlt->SetName(str_scope_name.substr(0, str_scope_name.length() - 1).c_str());
    }
    if (op_subtype == kScopeTypeFaceBoxesBatchMultiClassNonMaxSuppression) {
      fusion_rlt->InsertInputs("nms/map/Shape", {0});
      fusion_rlt->InsertInputs("nms/map/TensorArrayUnstack_1/Shape", {1});
      fusion_rlt->InsertOutputs("nms/map/TensorArrayStack/TensorArrayGatherV3", {0});
      fusion_rlt->InsertOutputs("nms/map/TensorArrayStack_1/TensorArrayGatherV3", {1});
      fusion_rlt->InsertOutputs("nms/map/TensorArrayStack_2/TensorArrayGatherV3", {3});
      fusion_rlt->SetType(kScopeType);
      fusion_rlt->SetDescription("");
      AscendString scope_name;
      Status ret = scopes[0]->Name(scope_name);
      if (ret != SUCCESS) {
          return ;
      }
      std::string str_scope_name;
      if (scope_name != nullptr){
          str_scope_name = scope_name.GetString();
      }
      fusion_rlt->SetName(str_scope_name.substr(0, str_scope_name.length() - 1).c_str());
    }
    if (op_subtype == kScopeTypeFiltereBatchMultiClassNonMaxSuppression) {
      fusion_rlt->InsertInputs("filtered_detections/map/Shape", {0});
      fusion_rlt->InsertInputs("filtered_detections/map/TensorArrayUnstack_1/Shape", {1});
      fusion_rlt->InsertOutputs("filtered_detections/map/TensorArrayStack/TensorArrayGatherV3", {0});
      fusion_rlt->InsertOutputs("filtered_detections/map/TensorArrayStack_1/TensorArrayGatherV3", {1});
      fusion_rlt->InsertOutputs("filtered_detections/map/TensorArrayStack_2/TensorArrayGatherV3", {2});
      fusion_rlt->SetType(kScopeType);
      fusion_rlt->SetDescription("");
      AscendString scope_name;
      Status ret = scopes[0]->Name(scope_name);
      if (ret != SUCCESS) {
          return ;
      }
      std::string str_scope_name;
      if (scope_name != nullptr){
          str_scope_name = scope_name.GetString();
      }
      fusion_rlt->SetName(str_scope_name.substr(0, str_scope_name.length() - 1).c_str());
    }
  }

  OP_LOGI(kOpType, "ScopeBatchMultiClassNonMaxSuppressionPass Scope fusion success.");
  return;
}
}  // namespace ge
