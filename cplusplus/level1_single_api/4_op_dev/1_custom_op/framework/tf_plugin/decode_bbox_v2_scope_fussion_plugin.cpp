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
#include "register/register.h"
#include "graph/ascend_string.h"

#define OP_LOGE(OP_NAME, fmt, ...) printf("[ERROR]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#define OP_LOGW(OP_NAME, fmt, ...) printf("[WARN]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)
#define OP_LOGI(OP_NAME, fmt, ...) printf("[INFO]%s,%s:%u:" #fmt "\n", __FUNCTION__, __FILE__, __LINE__, ##__VA_ARGS__)

namespace domi {
    namespace {
        const char *const kBoxesUnpack = "/unstack";
        const char *const kBoxesDiv = "RealDiv";
        const size_t kRealDivInputSize = 2;
        const size_t kScaleSize = 4;
    }  // namespace

    Status ParseFloatFromConstNode(const ge::Operator *node, float &value) {
        if (node == nullptr) {
            return FAILED;
        }
        ge::Tensor tensor;
        auto ret = node->GetAttr("value", tensor);
        if (ret != ge::GRAPH_SUCCESS) {
            ge::AscendString op_name;
            ge::graphStatus get_name_status = node->GetName(op_name);
            if (get_name_status != ge::GRAPH_SUCCESS) {
                return FAILED;
            }
            OP_LOGE(op_name.GetString(), "Failed to get value from %s", op_name.GetString());
            return FAILED;
        }
        uint8_t *data_addr = tensor.GetData();
        value = *(reinterpret_cast<float *>(data_addr));
        return SUCCESS;
    }

    Status DecodeBboxV2ParseParams(const std::vector <ge::Operator> &inside_nodes, ge::Operator &op_dest) {
        std::map <std::string, std::string> scales_const_name_map;
        std::map<string, const ge::Operator *> node_map;
        for (const auto &node : inside_nodes) {
            ge::AscendString op_type;
            ge::graphStatus ret = node.GetOpType(op_type);
            if (ret != ge::GRAPH_SUCCESS) {
                return FAILED;
            }
            ge::AscendString op_name;
            ret = node.GetName(op_name);
            string str_op_name;
            if (op_name.GetString() != nullptr) {
                str_op_name = op_name.GetString();
            }
            if (op_type == kBoxesDiv) {
                if (node.GetInputsSize() < kRealDivInputSize) {
                    OP_LOGE(op_name.GetString(), "Input size of %s is invalid, which is %zu.", kBoxesDiv,
                            node.GetInputsSize());
                    return FAILED;
                }
                ge::AscendString input_unpack_name0;
                ret = node.GetInputDesc(0).GetName(input_unpack_name0);
                string str_input_unpack_name0;
                if (input_unpack_name0.GetString() != nullptr) {
                    str_input_unpack_name0 = input_unpack_name0.GetString();
                }
                ge::AscendString input_unpack_name1;
                ret = node.GetInputDesc(1).GetName(input_unpack_name1);
                string str_input_unpack_name1;
                if (input_unpack_name1.GetString() != nullptr) {
                    str_input_unpack_name1 = input_unpack_name1.GetString();
                }
                if (str_input_unpack_name0.find(kBoxesUnpack) != string::npos) {
                    scales_const_name_map.insert({str_op_name, str_input_unpack_name1});
                }
            }
            node_map[str_op_name] = &node;
        }

        std::vector<float> scales_list = {1.0, 1.0, 1.0, 1.0};
        if (scales_const_name_map.size() != kScaleSize) {
            ge::AscendString op_name;
            ge::graphStatus ret = op_dest.GetName(op_name);
            if (ret != ge::GRAPH_SUCCESS) {
                return FAILED;
            }
            OP_LOGI(op_name.GetString(), "Boxes doesn't need scale.");
        } else {
            size_t i = 0;
            for (const auto &name_pair : scales_const_name_map) {
                float scale_value = 1.0;
                auto ret = ParseFloatFromConstNode(node_map[name_pair.second], scale_value);
                if (ret != SUCCESS) {
                    return ret;
                }
                scales_list[i++] = scale_value;
            }
        }
        op_dest.SetAttr("scales", scales_list);
        return SUCCESS;
    }

    REGISTER_CUSTOM_OP("DecodeBboxV2")
    .FrameworkType(TENSORFLOW)
    .OriginOpType("DecodeBboxV2FusionOp")
    .FusionParseParamsFn(DecodeBboxV2ParseParams)
    .ImplyType(ImplyType::TVM);
}  // namespace domi
