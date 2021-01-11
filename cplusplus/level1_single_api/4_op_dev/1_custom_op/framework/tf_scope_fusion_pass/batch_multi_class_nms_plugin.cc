/**
 * Copyright 2019 Huawei Technologies Co., Ltd
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
 * \file batch_multi_class_nms_plugin.cpp
 * \brief
 */
#include "register/register.h"
#include "graph/utils/attr_utils.h"
#include "graph/utils/op_desc_utils.h"
#include "proto/tensorflow/node_def.pb.h"

#include "tensorflow_fusion_op_parser_util.h"
#include "op_log.h"

using domi::tensorflow::NodeDef;
using domi::tensorflow::TensorProto;
using google::protobuf::Message;
using std::vector;

namespace domi {

static const char* const nms_v2 = "NonMaxSuppressionV2";
static const char* const nms_v3 = "NonMaxSuppressionV3";
static const char* const scoreConstKey = "map/while/MultiClassNonMaxSuppression/FilterGreaterThan/Greater";
static const char* const sizeConstKey = "map/while/MultiClassNonMaxSuppression/Minimum";
static const char* const sizeConstKeySec = "map/while/MultiClassNonMaxSuppression/Minimum_";

static const char* const scoreRetinanetConstKey = "map/while/Greater";
static const char* const sizeRetinanetConstKey = "map/while/Minimum";
static const char* const retinanet_out_num = "map/while/non_max_suppression_5/NonMaxSuppressionV3";

static const char* const SfscoreConstKey = "map/while/MultiClassNonMaxSuppression/GreaterEqual";
static const char* const scoreFaceboxConstKey = "map/while/GreaterEqual";
static const char* const sizeFaceboxConstKey = "map/while/non_max_suppression/NonMaxSuppressionV3";
static const char* const Facebox_out_num = "map/while/non_max_suppression/NonMaxSuppressionV3";

Status ParseFloatValueFromConstNms(const vector<const NodeDef*>& inputConstNodes, const string& names, float& value) {
  for (auto nodeDef : inputConstNodes) {
    string name = nodeDef->name();
    if (name == names) {
      if (ParseParamFromConst(nodeDef, value) != SUCCESS) {
        OP_LOGE("ParseParamFromConst data from const NodeDef %s failed", name.c_str());
        return PARAM_INVALID;
      }
      return SUCCESS;
    }
  }
  return FAILED;
}

Status ParseIntValueFromConstNms(const vector<const NodeDef*>& inputConstNodes, const string& names, int& value) {
  for (auto nodeDef : inputConstNodes) {
    string name = nodeDef->name();
    if (name == names) {
      if (ParseParamFromConst(nodeDef, value) != SUCCESS) {
        OP_LOGE("ParseParamFromConst data from const NodeDef %s failed", name.c_str());
        return PARAM_INVALID;
      }
      return SUCCESS;
    }
  }
  return FAILED;
}

Status BatchMultiClassNonMaxSuppressionParams(const std::vector<const google::protobuf::Message*> insideNodes,
                                              ge::Operator& op) {
  auto opDesc = ge::OpDescUtils::GetOpDescFromOperator(op);

  if (opDesc == nullptr) {
    OP_LOGE(op.GetName().c_str(), "Get op desc failed.");
    return FAILED;
  }

  vector<const NodeDef*> inputConstNodes;
  std::string input_attr_name;
  std::string iouConstNodeName = "";
  std::string scoreConstNodeName = "";
  std::string sizeClassConstNodeName = "";
  std::string sizeOutputNodeName = "";
  std::string sizeOutputConstNodeName = "";
  float attr_iou_threshold = 0.0;
  float attr_score_threshold = 0.0;
  int attr_max_size_per_class = 0;
  int attr_max_total_size = 0;
  bool isNormalization = false;
  for (auto node : insideNodes) {
    const NodeDef* nodeDef = reinterpret_cast<const NodeDef*>(node);
    std::string nodeName = nodeDef->name().c_str();
    if (nodeDef == nullptr) {
      OP_LOGE(op.GetName().c_str(), "Node_def is nullptr.");
      return FAILED;
    }
    inputConstNodes.push_back(nodeDef);
    // get iou const node name
    if ((iouConstNodeName == "") && ((nodeDef->op() == nms_v2) || (nodeDef->op() == nms_v3))) {
      iouConstNodeName = nodeDef->input(3);
      OP_LOGI(op.GetName().c_str(), "get NMS node name %s .", iouConstNodeName.c_str());
    }
    // get score const node name
    if ((scoreConstNodeName == "") && (nodeDef->op() == "Greater") &&
        ((nodeName.find(scoreRetinanetConstKey) != std::string::npos) ||
         (nodeName.find(scoreConstKey) != std::string::npos) ||
         (nodeName.find(scoreFaceboxConstKey) != std::string::npos))) {
      scoreConstNodeName = nodeDef->input(1);
      OP_LOGI(op.GetName().c_str(), "get score Greater node name %s .", scoreConstNodeName.c_str());
    }
    // get facebox_net score const node name
    if ((scoreConstNodeName == "") && (nodeDef->op() == "GreaterEqual") &&
        ((nodeName.find(scoreFaceboxConstKey) != std::string::npos) ||
         (nodeName.find(SfscoreConstKey) != std::string::npos))) {
      scoreConstNodeName = nodeDef->input(1);
      OP_LOGI(op.GetName().c_str(), "get facebox_net score GreaterEqual node name %s .", scoreConstNodeName.c_str());
    }

    // get max_size_per_class const node name
    if (((nodeName.find(sizeConstKey) != std::string::npos) ||
         (nodeName.find(sizeRetinanetConstKey) != std::string::npos)) &&
        (nodeDef->op() == "Minimum")) {
      OP_LOGI(op.GetName().c_str(), "get Minimum node name %s .", nodeName.c_str());
      if (nodeName.find(sizeConstKeySec) == std::string::npos) {
        sizeClassConstNodeName = nodeDef->input(0);
      }
      if (sizeOutputNodeName < nodeName) {
        sizeOutputNodeName = nodeName;
        sizeOutputConstNodeName = nodeDef->input(0);
      }
    }
    // get facebox max_size_per_class const node name
    if ((sizeClassConstNodeName == "") && (nodeDef->op() == nms_v3) &&
        ((nodeName.find(sizeFaceboxConstKey) != std::string::npos))) {
      sizeClassConstNodeName = nodeDef->input(2);
      OP_LOGI(op.GetName().c_str(), "get facebox max_size_per_class node name %s .", sizeClassConstNodeName.c_str());
    }

    // get retinanet max_total_size const node name
    if ((sizeOutputConstNodeName == "") && (nodeDef->op() == nms_v3) &&
        ((nodeName.find(retinanet_out_num) != std::string::npos) ||
         (nodeName.find(Facebox_out_num) != std::string::npos))) {
      sizeOutputConstNodeName = nodeDef->input(2);
      OP_LOGI(op.GetName().c_str(), "get retinanet Output Const node name %s .", sizeOutputConstNodeName.c_str());
    }

    // get isNormalization info
    if (!isNormalization && (nodeName.find("ChangeCoordinateFrame") != std::string::npos)) {
      OP_LOGI(op.GetName().c_str(), "get ChangeCoordinateFrame node name %s .", nodeName.c_str());
      isNormalization = true;
    }
  }
  // check whether success to get the required attr info
  if (iouConstNodeName.empty()) {
    OP_LOGE(op.GetName().c_str(), "can not find iou const node for BatchMultiClassNonMaxSuppression");
    return FAILED;
  }
  if (scoreConstNodeName.empty()) {
    OP_LOGE(op.GetName().c_str(), "can not find score const node for BatchMultiClassNonMaxSuppression");
    return FAILED;
  }
  if (ParseFloatValueFromConstNms(inputConstNodes, iouConstNodeName, attr_iou_threshold) != SUCCESS) {
    OP_LOGE(op.GetName().c_str(), "Convert attr_iou_threshold data failed");
    return PARAM_INVALID;
  }
  if (ParseFloatValueFromConstNms(inputConstNodes, scoreConstNodeName, attr_score_threshold) != SUCCESS) {
    OP_LOGE(op.GetName().c_str(), "Convert attr_score_threshold data failed");
    return PARAM_INVALID;
  }

  if (sizeClassConstNodeName.empty()) {
    OP_LOGE(op.GetName().c_str(), "can not find max_size_per_class for BatchMultiClassNonMaxSuppression");
    return FAILED;
  }

  if (ParseIntValueFromConstNms(inputConstNodes, sizeClassConstNodeName, attr_max_size_per_class) != SUCCESS) {
    OP_LOGE(op.GetName().c_str(), "Convert attr_max_size_per_class data failed");
    return PARAM_INVALID;
  }
  if (sizeOutputConstNodeName.empty()) {
    OP_LOGE(op.GetName().c_str(), "can not find max_total_size for BatchMultiClassNonMaxSuppression");
    return FAILED;
  }

  if (ParseIntValueFromConstNms(inputConstNodes, sizeOutputConstNodeName, attr_max_total_size) != SUCCESS) {
    OP_LOGE(op.GetName().c_str(), "Convert attr_max_total_size data failed");
    return PARAM_INVALID;
  }

  // set attr for Op BatchMultiClassNonMaxSuppression
  if (!ge::AttrUtils::SetFloat(opDesc, "iou_threshold", attr_iou_threshold)) {
    OP_LOGE(op.GetName().c_str(), "Set attr iou_threshold failed.");
    return FAILED;
  }
  OP_LOGI(op.GetName().c_str(), "Set attr iou_threshold %1.2f.", attr_iou_threshold);

  if (!ge::AttrUtils::SetFloat(opDesc, "score_threshold", attr_score_threshold)) {
    OP_LOGE(op.GetName().c_str(), "Set attr score_threshold failed.");
    return FAILED;
  }
  OP_LOGI(op.GetName().c_str(), "Set attr score_threshold %1.2f.", attr_score_threshold);

  if (!ge::AttrUtils::SetInt(opDesc, "max_size_per_class", attr_max_size_per_class)) {
    OP_LOGE(op.GetName().c_str(), "Set attr max_size_per_class failed.");
    return FAILED;
  }
  OP_LOGI(op.GetName().c_str(), "Set attr max_size_per_class %d.", attr_max_size_per_class);

  if (!ge::AttrUtils::SetInt(opDesc, "max_total_size", attr_max_total_size)) {
    OP_LOGE(op.GetName().c_str(), "Set attr max_total_size failed.");
    return FAILED;
  }
  OP_LOGI(op.GetName().c_str(), "Set attr max_total_size %d.", attr_max_total_size);

  if (!ge::AttrUtils::SetBool(opDesc, "change_coordinate_frame", isNormalization)) {
    OP_LOGE(op.GetName().c_str(), "Set attr change_coordinate_frame failed.");
    return FAILED;
  }
  OP_LOGI(op.GetName().c_str(), "Set attr change_coordinate_frame %d.", isNormalization);
  return SUCCESS;
}

REGISTER_CUSTOM_OP("BatchMultiClassNonMaxSuppression")
    .FrameworkType(TENSORFLOW)
    .OriginOpType("BatchMultiClassNonMaxSuppression")
    .FusionParseParamsFn(BatchMultiClassNonMaxSuppressionParams)
    .ImplyType(ImplyType::TVM);
}  // namespace domi
