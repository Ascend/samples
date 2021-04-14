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

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string.h>
#include "tensorflow_parser.h"
#include "caffe_parser.h"
#include "graph.h"
#include "types.h"
#include "tensor.h"
#include "attr_value.h"
#include "ge_error_codes.h"
#include "ge_api_types.h"
#include "ge_ir_build.h"
#include "all_ops.h"
#include <dlfcn.h>
#include <unistd.h>
//#include "add.h" // custom op ,if you have one new or different op defination with frame's,please
                   // add head file here.If same with frame , no need to add head file here

using namespace std;
using namespace ge;
using ge::Operator;

namespace {
static const int kArgsNum = 3;
static const int kSocVersion = 1;
static const int kGenGraphOpt = 2;
static const std::string kPath = "../data/";
}  // namespace

void PrepareOptions(std::map<AscendString, AscendString>& options) {
}

bool GetConstTensorFromBin(string path, Tensor &weight, uint32_t len) {
    ifstream in_file(path.c_str(), std::ios::in | std::ios::binary);
    if (!in_file.is_open()) {
        std::cout << "failed to open" << path.c_str() << '\n';
        return false;
    }
    in_file.seekg(0, ios_base::end);
    istream::pos_type file_size = in_file.tellg();
    in_file.seekg(0, ios_base::beg);

    if (len != file_size) {
        cout << "Invalid Param.len:" << len << " is not equal with binary size（" << file_size << ")\n";
        in_file.close();
        return false;
    }
    char* pdata = new(std::nothrow) char[len];
    if (pdata == nullptr) {
        cout << "Invalid Param.len:" << len << " is not equal with binary size（" << file_size << ")\n";
        in_file.close();
        return false;
    }
    in_file.read(reinterpret_cast<char*>(pdata), len);
    auto status = weight.SetData(reinterpret_cast<uint8_t*>(pdata), len);
    if (status != ge::GRAPH_SUCCESS) {
        cout << "Set Tensor Data Failed"<< "\n";
        delete [] pdata;
        in_file.close();
        return false;
    }
    in_file.close();
    return true;
}
bool GenGraph(Graph& graph)
{
    auto shape_data = vector<int64_t>({ 1,1,28,28 });
    TensorDesc desc_data(ge::Shape(shape_data), FORMAT_ND, DT_FLOAT16);

    // data op
    auto data = op::Data("data");
    data.update_input_desc_x(desc_data);
    data.update_output_desc_y(desc_data);
    // custom op ,using method is the same with frame internal op
    // [Notice]: if you want to use custom self-define op, please prepare custom op according to custum op define user guides
    auto add = op::Add("add")
        .set_input_x1(data)
        .set_input_x2(data);
    // AscendQuant
    auto quant = op::AscendQuant("quant")
        .set_input_x(data)
        .set_attr_scale(1.0)
        .set_attr_offset(0.0);

    // const op: conv2d weight
    auto weight_shape = ge::Shape({ 2,2,1,1 });
    TensorDesc desc_weight_1(weight_shape, FORMAT_ND, DT_INT8);
    Tensor weight_tensor(desc_weight_1);
    uint32_t weight_1_len = weight_shape.GetShapeSize();
    bool res = GetConstTensorFromBin(kPath+"Conv2D_kernel_quant.bin", weight_tensor, weight_1_len);
    if (!res) {
        cout << __LINE__ << "GetConstTensorFromBin Failed!" << endl;
        return -1;
    }
    auto conv_weight = op::Const("Conv2D/weight")
        .set_attr_value(weight_tensor);

    // conv2d op
    auto conv2d = op::Conv2D("Conv2d1")
        .set_input_x(quant)
        .set_input_filter(conv_weight)
        .set_attr_strides({ 1, 1, 1, 1 })
        .set_attr_pads({ 0, 1, 0, 1 })
        .set_attr_dilations({ 1, 1, 1, 1 });

    TensorDesc conv2d_input_desc_x(ge::Shape(), FORMAT_NCHW, DT_INT8);
    TensorDesc conv2d_input_desc_filter(ge::Shape(), FORMAT_HWCN, DT_INT8);
    TensorDesc conv2d_output_desc_y(ge::Shape(), FORMAT_NCHW, DT_INT8);
    conv2d.update_input_desc_x(conv2d_input_desc_x);
    conv2d.update_input_desc_filter(conv2d_input_desc_filter);
    conv2d.update_output_desc_y(conv2d_output_desc_y);
    // dequant scale
    TensorDesc desc_dequant_shape(ge::Shape({ 1 }), FORMAT_ND, DT_UINT64);
    Tensor dequant_tensor(desc_dequant_shape);
    uint64_t dequant_scale_val = 1;
    auto status = dequant_tensor.SetData(reinterpret_cast<uint8_t*>(&dequant_scale_val), sizeof(uint64_t));
    if (status != ge::GRAPH_SUCCESS) {
        cout << __LINE__ << "Set Tensor Data Failed" << "\n";
        return false;
    }
    auto dequant_scale = op::Const("dequant_scale")
        .set_attr_value(dequant_tensor);

    // AscendDequant
    auto dequant = op::AscendDequant("dequant")
        .set_input_x(conv2d)
        .set_input_deq_scale(dequant_scale);

    // const op: BiasAdd weight
    auto weight_bias_add_shape_1 = ge::Shape({ 1 });
    TensorDesc desc_weight_bias_add_1(weight_bias_add_shape_1, FORMAT_ND, DT_FLOAT);
    Tensor weight_bias_add_tensor_1(desc_weight_bias_add_1);
    uint32_t weight_bias_add_len_1 = weight_bias_add_shape_1.GetShapeSize() * sizeof(float);
    float weight_bias_add_value = 0.006448820233345032;
    status = weight_bias_add_tensor_1.SetData(reinterpret_cast<uint8_t*>(&weight_bias_add_value), weight_bias_add_len_1);
    if (status != ge::GRAPH_SUCCESS) {
        cout << __LINE__ << "Set Tensor Data Failed" << "\n";
        return false;
    }
    auto bias_weight_1 = op::Const("Bias/weight_1")
        .set_attr_value(weight_bias_add_tensor_1);
    // BiasAdd 1
    auto bias_add_1 = op::BiasAdd("bias_add_1")
        .set_input_x(dequant)
        .set_input_bias(bias_weight_1)
        .set_attr_data_format("NCHW");

    // const
    int32_t value[2] = {1,-1};

    auto value_shape = ge::Shape({ 2 });
    TensorDesc desc_dynamic_const(value_shape, FORMAT_ND, DT_INT32);
    Tensor dynamic_const_tensor(desc_dynamic_const);
    uint32_t dynamic_const_len = value_shape.GetShapeSize() * sizeof(int32_t);
    status = dynamic_const_tensor.SetData(reinterpret_cast<uint8_t*>(&(value[0])), dynamic_const_len);
    if (status != ge::GRAPH_SUCCESS) {
        cout << __LINE__ << "Set Tensor Data Failed" << "\n";
        return false;
    }
    auto dynamic_const = op::Const("dynamic_const").set_attr_value(dynamic_const_tensor);

    // ReShape op
    auto reshape = op::Reshape("Reshape")
        .set_input_x(bias_add_1)
        .set_input_shape(dynamic_const);
    // MatMul + BiasAdd
    // MatMul weight 1
    auto matmul_weight_shape_1 = ge::Shape({784,512});
    TensorDesc desc_matmul_weight_1(matmul_weight_shape_1, FORMAT_ND, DT_FLOAT);
    Tensor matmul_weight_tensor_1(desc_matmul_weight_1);
    uint32_t matmul_weight_1_len = matmul_weight_shape_1.GetShapeSize() * sizeof(float);
    res = GetConstTensorFromBin(kPath + "dense_kernel.bin", matmul_weight_tensor_1, matmul_weight_1_len);
    if (!res) {
        cout << __LINE__ << "GetConstTensorFromBin Failed!" << endl;
        return -1;
    }
    auto matmul_weight_1 = op::Const("dense/kernel")
        .set_attr_value(matmul_weight_tensor_1);
    // MatMul1
    auto matmul_1 = op::MatMul("MatMul_1")
        .set_input_x1(reshape)
        .set_input_x2(matmul_weight_1);
    // BiasAdd const 2
    auto bias_add_shape_2 = ge::Shape({ 512 });
    TensorDesc desc_bias_add_const_1(bias_add_shape_2, FORMAT_ND, DT_FLOAT);
    Tensor bias_add_const_tensor_1(desc_bias_add_const_1);
    uint32_t bias_add_const_len_1 = bias_add_shape_2.GetShapeSize() * sizeof(float);
    res = GetConstTensorFromBin(kPath + "dense_bias.bin", bias_add_const_tensor_1, bias_add_const_len_1);
    if (!res) {
        cout << __LINE__ << "GetConstTensorFromBin Failed!" << endl;
        return -1;
    }
    auto bias_add_const_1 = op::Const("dense/bias")
        .set_attr_value(bias_add_const_tensor_1);
    // BiasAdd 2
    auto bias_add_2 = op::BiasAdd("bias_add_2")
        .set_input_x(matmul_1)
        .set_input_bias(bias_add_const_1)
        .set_attr_data_format("NCHW");
    // Relu6
    auto relu6 = op::Relu6("relu6")
        .set_input_x(bias_add_2);
    // MatMul weight 2
    auto matmul_weight_shape_2 = ge::Shape({ 512, 10 });
    TensorDesc desc_matmul_weight_2(matmul_weight_shape_2, FORMAT_ND, DT_FLOAT);
    Tensor matmul_weight_tensor_2(desc_matmul_weight_2);
    uint32_t matmul_weight_2_len = matmul_weight_shape_2.GetShapeSize() * sizeof(float);
    res = GetConstTensorFromBin(kPath + "OutputLayer_kernel.bin", matmul_weight_tensor_2, matmul_weight_2_len);
    if (!res) {
        cout << __LINE__ << "GetConstTensorFromBin Failed!" << endl;
        return -1;
    }
    auto matmul_weight_2 = op::Const("OutputLayer/kernel")
        .set_attr_value(matmul_weight_tensor_2);
    // MatMul 2
    auto matmul_2 = op::MatMul("MatMul_2")
        .set_input_x1(relu6)
        .set_input_x2(matmul_weight_2);
    // BiasAdd const 3
    auto bias_add_shape_3 = ge::Shape({ 10 });
    TensorDesc desc_bias_add_const_3(bias_add_shape_3, FORMAT_ND, DT_FLOAT);
    Tensor bias_add_const_tensor_3(desc_bias_add_const_3);
    uint32_t bias_add_const_len_3 = bias_add_shape_3.GetShapeSize() * sizeof(float);
    res = GetConstTensorFromBin(kPath + "OutputLayer_bias.bin", bias_add_const_tensor_3, bias_add_const_len_3);
    if (!res) {
        cout << __LINE__ << "GetConstTensorFromBin Failed!" << endl;
        return -1;
    }
    auto bias_add_const_3 = op::Const("OutputLayer/bias")
        .set_attr_value(bias_add_const_tensor_3);
    // BiasAdd 3
    /*
     * When set input for some node, there are two methodes for you.
     * Method 1: operator level method. Frame will auto connect the node's output edge to netoutput nodes for user
     *   we recommend this method when some node own only one out node
     * Method 2: edge of operator level. Frame will find the edge according to the output edge name
     *   we recommend this method when some node own multi out nodes and only one out edge data wanted back
     */
    auto bias_add_3 = op::BiasAdd("bias_add_3")
        .set_input_x(matmul_2, "y")
        .set_input_bias(bias_add_const_3, "y")
        .set_attr_data_format("NCHW");
    // Softmax op
    auto softmax = op::SoftmaxV2("Softmax")
        .set_input_x(bias_add_3, "y");

    std::vector<Operator> inputs{ data };
    /*
     * The same as set input, when point net output ,Davince framework alos support multi method to set outputs info
     * Method 1: operator level method. Frame will auto connect the node's output edge to netoutput nodes for user
     *   we recommend this method when some node own only one out node
     * Method 2: edge of operator level. Frame will find the edge according to the output edge name
     *   we recommend this method when some node own multi out nodes and only one out edge data wanted back
     * Using method is like follows:
     */
    std::vector<Operator> outputs{ softmax, add };
    std::vector<std::pair<ge::Operator, std::string>> outputs_with_name = {{softmax, "y"}};

    graph.SetInputs(inputs).SetOutputs(outputs);

    return true;
}

// |o>-------------------------
// |o>              data
// |o>               |
// |o>        data  abs const
// |o>         |     |  /
// |o>        abs   add
// |o>          \   /
// |o>           add
// modify tf graph
bool ModifyGraph(Graph &graph) {
    /*   First, you need to know where to insert new node , and find src node and dest node of new node
     * by Node name from all nodes of graph;
     *   Second, remove edge between src node and dest node(data or control edge).
     *   Third, create new node by operator.
     *   Last, add edge(data or control) between src node and new node.
     * add edge between new node and dest node.
     *   Here, we will insert Abs between Add and Const.
     */
    // Option: If you need to know shape and type info of node, you can call infer shape interface:
    // aclgrphInferShapeAndType ,then view this info by graph file which generated by dump graph
    // interface: aclgrphDumpGraph.
    std::cout<<"Modify Graph Start."<<std::endl;
    const std::string CONST = "const1";
    const std::string ADD = "input3_add";
    GNode src_node;
    GNode dst_node;
    std::vector<GNode> nodes = graph.GetAllNodes();
    graphStatus ret = GRAPH_FAILED;
    for (auto &node : nodes) {
        ge::AscendString name;
        ret = node.GetName(name);
        if (ret != GRAPH_SUCCESS) {
            std::cout<<"Get node name failed."<<std::endl;
            return false;
        }
        std::string node_name(name.GetString());
        if (node_name == CONST) {
            src_node = node;
            std::cout<<"Find src node: const."<<std::endl;
        } else if (node_name == ADD) {
            dst_node = node;
            std::cout<<"Find dst node: add."<<std::endl;
        }
    }
    ret = graph.RemoveEdge(src_node, 0, dst_node, 1);
    if (ret != GRAPH_SUCCESS) {
        std::cout<<"Modify graph: remove edge failed between const and add."<<std::endl;
        return false;
    }
    auto abs = op::Abs("input3_abs");
    GNode node_abs = graph.AddNodeByOp(abs);
    ret = graph.AddDataEdge(src_node, 0, node_abs, 0);
    if (ret != GRAPH_SUCCESS) {
        std::cout<<"Modify graph: add data edge failed between const and abs."<<std::endl;
        return false;
    }
    ret = graph.AddDataEdge(node_abs, 0, dst_node, 1);
    if (ret != GRAPH_SUCCESS) {
        std::cout<<"Modify graph: add data edge failed between abs and add."<<std::endl;
        return false;
    }
    std::cout<<"Modify Graph Success."<<std::endl;

    return true;
}

int main(int argc, char* argv[])
{
    cout << "========== Test Start ==========" << endl;
    if (argc != kArgsNum) {
        cout << "[ERROR]input arg num must be 3! " << endl;
        cout << "The second arg stand for soc version! Please retry with your soc version " << endl;
        cout << "[Notice] Supported soc version as list:Ascend310 Ascend910 Ascend610 Ascend620 Hi3796CV300ES" << endl;
        cout << "The third arg stand for Generate Graph Options! Please retry with your soc version " << endl;
        cout << "[Notice] Supported Generate Graph Options as list:" << endl;
        cout << "    [gen]: GenGraph" << endl;
        cout << "    [tf]: generate from tensorflow origin model;" << endl;
        cout << "    [caffe]: generate from caffe origin model" << endl;
        return -1;
    }
    cout << argv[kSocVersion] << endl;
    cout << argv[kGenGraphOpt] << endl;

    // 1. Genetate graph
    Graph graph1("IrGraph1");
    bool ret;

    if (string(argv[kGenGraphOpt]) == "gen") {
        ret = GenGraph(graph1);
        if (!ret) {
            cout << "========== Generate Graph1 Failed! ==========" << endl;
            return -1;
        }
        else {
            cout << "========== Generate Graph1 Success! ==========" << endl;
        }
    } else if (string(argv[kGenGraphOpt]) == "tf") {
        std::string tfPath = "../data/tf_test.pb";
        auto tfStatus = ge::aclgrphParseTensorFlow(tfPath.c_str(), graph1);
        if (tfStatus != GRAPH_SUCCESS) {
            cout << "========== Generate graph from tensorflow origin model failed.========== " << endl;
            return 0;
        }
        cout << "========== Generate graph from tensorflow origin model success.========== " << endl;
        if (ModifyGraph(graph1)) {
          cout << "========== Modify tensorflow origin graph success.========== " << endl;
        }
    } else if (string(argv[kGenGraphOpt]) == "caffe") {
        std::string caffePath = "../data/caffe_test.prototxt";
        std::string weigtht = "../data/caffe_test.caffemodel";
        auto caffeStatus = ge::aclgrphParseCaffe(caffePath.c_str(), weigtht.c_str(), graph1);
        if (caffeStatus != GRAPH_SUCCESS) {
            cout << "========== Generate graph from caffe origin model failed.========== " << endl;
            return 0;
        }
        cout << "========== Generate graph from caffe origin model success.========== " << endl;
    }

    // 2. system init
    std::map<AscendString, AscendString> global_options = {
        {AscendString(ge::ir_option::SOC_VERSION), AscendString(argv[kSocVersion])}  ,
    };
    auto status = aclgrphBuildInitialize(global_options);
    // 3. Build Ir Model1
    ModelBufferData model1;
    std::map<AscendString, AscendString> options;
    PrepareOptions(options);

    status = aclgrphBuildModel(graph1, options, model1);
    if (status == GRAPH_SUCCESS) {
        cout << "Build Model1 SUCCESS!" << endl;
    }
    else {
        cout << "Build Model1 Failed!" << endl;
    }
    // 4. Save Ir Model
    status = aclgrphSaveModel("ir_build_sample1", model1);
    if (status == GRAPH_SUCCESS) {
        cout << "Save Offline Model1 SUCCESS!" << endl;
    }
    else {
        cout << "Save Offline Model1 Failed!" << endl;
    }

    // release resource
    aclgrphBuildFinalize();
    return 0;
}
