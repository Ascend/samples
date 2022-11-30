# ScopeBatchMultiClassNonMaxSuppressionPass Sample Reference<a name="EN-US_TOPIC_0298659575"></a>

Please refer to: [https://www.hiascend.com/document](https://www.hiascend.com/document) "TensorFlow Parser Scope Fusion Rule Development" manual to find details about the fusion pattern development workflow. The following describes only the key steps of this sample.

A TensorFlow neural network usually consists of a large number of operators. To optimize the inference performance, you can fuse the operators to larger ones to make full use of the hardware acceleration resources. Fusion patterns can be developed by implementing FE fusion or GE scope fusion. Scope fusion is a necessary mapping method for replacing operators in a scope with a larger operator while maintaining the functionality of the original scope.

The purpose of GE scope fusion is to leverage the compute acceleration advantages of TBE operators. Therefore, it is necessary to identify the TBE APIs to call as well as extract the arguments required by these API calls. The overall workflow can be divided into two phases.

![输入图片说明](https://images.gitee.com/uploads/images/2020/1211/174242_54eeff2b_8200958.png "zh-cn_image_0298659578.PNG")

In the scope identification phase, check whether the scope has corresponding TensorFlow Python APIs. If yes, extract the necessary arguments, including the config arguments and compute arguments. The config arguments determine the generation of the computational graph of the scope. The compute arguments are data involved in neural network inference, including the inputs, weight, and bias.

The core of fusion policy design is to determine the key features of the scope, which define the uniqueness of the scope. The key features need to be determined based on the Python API source code and corresponding calculation formulas. According to the generated computational graph, the key features are classified into the following types:

-   0-order features: The connections between operators inside and outside the scope are not considered. For example, the unconfigurable scope name, the number of typical operators, and the values of typical operator attributes.
-   1-order features: The 1-step connections between operators inside and outside the scope are considered. The scope input is connected to the ConcatV2 operator, for example.
-   2-order features: The 2-step connections between operators inside and outside the scope are considered. For example, the concat operator connects to the MatMul operator and then to the BiasAdd operator.
-   Higher-order features...

Generally, a higher feature order indicates heavier compute workloads involved in scope identification. Therefore, determining the fusion policy is mainly about determining the low-order features.

Note that the scope fusion patterns need to be analyzed based on the scope structure. Identify those special nodes in the scope and ensure that your developed scope fusion patterns are unique and can match your own network despite the possibility that your developed rules might mis-fuse other networks. It would be ideal if you can generalize your fusion patterns, which, however, is too complex to implement due to the graph diversity of neural networks.

## Sample Obtaining<a name="section122898421212"></a>

Click  [here](https://gitee.com/ascend/samples/tree/master/cplusplus/level1_single_api/4_op_dev/1_custom_op/framework/tf_scope_fusion_pass)  to obtain the fusion pattern code and plug-in code.

## Fusion Rule Design<a name="section184331455516"></a>

This document takes the "SecondStagePostprocessor/BatchMultiClassNonMaxSuppression" scope in the faster\_rcnn\_resnet101 network as an example to describe the fusion pattern design.

Click  [here](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md)  to download the .pb model file.

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221038_30b52046_8200958.png "zh-cn_image_0298659577.png")

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221052_0610d3a7_8200958.png "zh-cn_image_0298659576.png")

According to the source code, the scope implements postprocessing of the detection network \(mainly the top K result sorting and NMS operations\). Fortunately, the batch\_multi\_class\_nms\_topk operator with equivalent capability is available. 

Analyze the graph structure and key features in the graph:

Unfold the "BatchMultiClassNonMaxSuppression" structure layer by layer:

Unfold the "BatchMultiClassNonMaxSuppression" structure.

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221108_4dc008e3_8200958.png "zh-cn_image_0298659925.png")

Unfold the "BatchMultiClassNonMaxSuppression/map" structure.

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221121_07c4885b_8200958.png "zh-cn_image_0298659580.png")

Unfold the "BatchMultiClassNonMaxSuppression/map/while/MultiClassNonMaxSuppression" structure.

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221136_c64011cd_8200958.png "zh-cn_image_0298659582.png")

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221146_262e3b6e_8200958.png "zh-cn_image_0298659579.png")

## Key Feature Analysis<a name="section7435163495712"></a>

The "BatchMultiClassNonMaxSuppression" scope maps to the postprocessing operator batch\_multi\_class\_nms\_topk, which is implemented from the source code available at  [https://github.com/tensorflow/models](https://github.com/tensorflow/models). Its functions include  **topk**,  **nms**, and  **clip\_to\_window**.

The "BatchMultiClassNonMaxSuppression" scope is a subgraph in the TensorFlow graph. It has the following key features:

For details, please visit:

[https://gitee.com/ascend/samples/tree/master/cplusplus/level1\_single\_api/4\_op\_dev/1\_custom\_op/framework/tf\_scope\_fusion\_pass](https://gitee.com/ascend/samples/tree/master/cplusplus/level1_single_api/4_op_dev/1_custom_op/framework/tf_scope_fusion_pass)

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221202_4d663acf_8200958.png "zh-cn_image_0298659584.png")

-   Feature type: 0-order features
-   Feature details: one NonMaxSuppressionV2 operators; four Maximum operators

For accurate pattern matching, try to identify more 0-order features.

## Argument Extraction<a name="section131415810585"></a>

The following lists the compute arguments required by the batch\_multi\_class\_nms\_topk operator. These arguments are passed as attributes of the fused operator. \(These arguments are passed as attributes at the plug-in pass layer.\)

-   **iou\_threshold**

    Specifies the IoU threshold.

    Extracted from the fourth input of the "NonMaxSuppressionV2" node.

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221220_7766af63_8200958.png "zh-cn_image_0298659581.png")

-   **score\_threshold**

    Specifies the score threshold.

    Extracted from the first input of the "onMaxSuppression/FilterGreaterThan/Greater" node.

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221309_c91e92d9_8200958.png "zh-cn_image_0298659927.png")

-   **max\_size\_per\_class**

    Specifies the maximum number of output boxes per class.

    Extracted from the first input of the "Minimum" node.

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221254_5cc49916_8200958.png "zh-cn_image_0298659926.png")


## Plug-in Code Analysis<a name="section14493169171319"></a>

After extracting the attributes of the fused operator, you need to set these attributes to the plug-in. \(The preceding figures are for reference only.\)

1.  Find the following lines: 

    ```
    static const char* const nms_v3 = "NonMaxSuppressionV3";
    static const char* const scoreConstKey = "map/while/MultiClassNonMaxSuppression/FilterGreaterThan/Greater";
    ```

2.  Extract the required  **iou\_threshold**  and  **score\_threshold**  arguments from the specific inputs in the graph.

    ```
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
    ```

3.  Set the obtained arguments to the fused operator.

    ```
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
    ```


## Fusion Rule Code Analysis<a name="section118511811145912"></a>

1.  Include the following header file: scope_batchmulticlass_nms_pass.h

    ```
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
    ```

2.  Add the fusion pattern code: scope_batchmulticlass_nms_pass.cc

    -   Define  **Patterns**  and  **PassName**.

    ```
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
    ```

    -   Design the matching patterns based on the identified key features. The following shows the matching patterns of multiple network nodes, which are distinguished by the pattern name. In this sample, four patterns are available:
        ScopeBatchMultiClassNMSPattern、ScopeSecondBatchMultiClassNMSPattern、ScopeFaceBoxesBatchMultiClassNMSPattern、
        ScopeFilteredBatchMultiClassNMSPattern

    ```
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
    ```

    -   Determine the mapping between the matching patterns and branches. 
        kScopeTypeSecondBatchMultiClassNonMaxSuppression、kScopeTypeFaceBoxesBatchMultiClassNonMaxSuppression
        kScopeTypeFiltereBatchMultiClassNonMaxSuppression

    ```
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
        if (scope->SubType() == kScopeTypeBatchMultiClassNonMaxSuppression) {
          OP_LOGI(kOpType, "BatchMultiClassNonMaxSuppression LastMatchScopesAndOPs match SubType.");
          ScopesResult result;
          std::vector<Scope*> result_scopes;
          result_scopes.push_back(scope);
          result.SetScopes(result_scopes);
          results.push_back(result);
        }
        if (scope->SubType() == kScopeTypeSecondBatchMultiClassNonMaxSuppression) {
          OP_LOGI(kOpType, "SecondBatchMultiClassNonMaxSuppression LastMatchScopesAndOPs match SubType.");
          ScopesResult result;
          std::vector<Scope*> result_scopes;
          result_scopes.push_back(scope);
          result.SetScopes(result_scopes);
          results.push_back(result);
        }
        if (scope->SubType() == kScopeTypeFaceBoxesBatchMultiClassNonMaxSuppression) {
          OP_LOGI(kOpType, "FaceBoxesBatchMultiClassNonMaxSuppression LastMatchScopesAndOPs match SubType.");
          ScopesResult result;
          std::vector<Scope*> result_scopes;
          result_scopes.push_back(scope);
          result.SetScopes(result_scopes);
          results.push_back(result);
        }
        if (scope->SubType() == kScopeTypeFiltereBatchMultiClassNonMaxSuppression) {
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
    ```

    -   Set the inputs and outputs of the fused operator.

    Method 1:  **scope\_batchmulticlass\_nms\_pass.cc**

    ```
    void ScopeBatchMultiClassNonMaxSuppressionPass::GenerateFusionResult(const std::vector<Scope*>& scopes,
                                                                         FusionScopesResult* fusion_rlt) {
      if (fusion_rlt == nullptr) {
        OP_LOGE(kOpType, "Input fusion_rlt is nullptr.");
        return;
      }
    
      for (auto& scope : scopes) {
        if (scope->SubType() == kScopeTypeBatchMultiClassNonMaxSuppression) {
          fusion_rlt->InsertInputs("BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack/Shape", {0});
          fusion_rlt->InsertInputs("BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack_1/Shape", {1});
          fusion_rlt->InsertInputs("BatchMultiClassNonMaxSuppression/map/TensorArrayUnstack_3/Shape", {2});
          fusion_rlt->InsertOutputs("BatchMultiClassNonMaxSuppression/map/TensorArrayStack/TensorArrayGatherV3", {0});
          fusion_rlt->InsertOutputs("BatchMultiClassNonMaxSuppression/map/TensorArrayStack_4/TensorArrayGatherV3", {3});
          fusion_rlt->SetType(kScopeType);
          fusion_rlt->SetDescription("");
          std::string scope_name = scope->Name();
          fusion_rlt->SetName(scope_name.substr(0, scope_name.length() - 1));
        }
    ```

    Method 2:  **scope\_newbatchmulticlass\_nms\_pass.cc**

    ```
    void ScopeBatchMultiClassNMSPass::GenerateFusionResult(const std::vector<Scope*>& scopes,
                                                           FusionScopesResult* fusion_rlt) {
      if (fusion_rlt == nullptr) {
        OP_LOGE(kOpType.c_str(), "Input fusion_rlt is nullptr.");
        return;
      }
      for (auto& scope : scopes) {
        // do insert input 0 boxes like map/TensorArrayUnstack/Shape
        std::string scopeInputName0 = scope->Name() + "map/TensorArrayUnstack/Shape";
        fusion_rlt->InsertInputs(scopeInputName0, {0});
    
        // do insert input 1 scores like map/TensorArrayUnstack_1/Shape
        std::string scopeInputName1 = scope->Name() + "map/TensorArrayUnstack_1/Shape";
        fusion_rlt->InsertInputs(scopeInputName1, {1});
    
        // do insert input 2 clip_to_window like map/TensorArrayUnstack_3/Shape
        if (MatchedSubScopes(scope, {"map/", "TensorArrayUnstack_3/"})) {
          std::string scopeInputName2 = scope->Name() + "map/TensorArrayUnstack_3/Shape";
          fusion_rlt->InsertInputs(scopeInputName2, {2});
          if (MatchedSubScopes(scope, {"map/", "TensorArrayUnstack_4/"}) && !MatchedSubScopes(scope, {"ones/"})) {
            std::string scopeInputName3 = scope->Name() + "map/TensorArrayUnstack_4/Shape";
            fusion_rlt->InsertInputs(scopeInputName3, {3});
          }
        }
        // do insert outputs
        if (MatchedSubScopes(scope, {"map/", "TensorArrayStack/"})) {
          std::string scopeOutputName0 = scope->Name() + "map/TensorArrayStack/TensorArrayGatherV3";
          fusion_rlt->InsertOutputs(scopeOutputName0, {0});
        }
        if (MatchedSubScopes(scope, {"map/", "TensorArrayStack_1/"})) {
          std::string scopeOutputName0 = scope->Name() + "map/TensorArrayStack_1/TensorArrayGatherV3";
          fusion_rlt->InsertOutputs(scopeOutputName0, {1});
        }
        if (MatchedSubScopes(scope, {"map/", "TensorArrayStack_2/"})) {
          std::string scopeOutputName0 = scope->Name() + "map/TensorArrayStack_2/TensorArrayGatherV3";
          fusion_rlt->InsertOutputs(scopeOutputName0, {2});
        }
        if (MatchedSubScopes(scope, {"map/", "TensorArrayStack_4/"})) {
          std::string scopeOutputName0 = scope->Name() + "map/TensorArrayStack_4/TensorArrayGatherV3";
          fusion_rlt->InsertOutputs(scopeOutputName0, {3});
        }
        fusion_rlt->SetType(kOpType);
        fusion_rlt->SetDescription("");
        std::string scopeName = scope->Name();
        fusion_rlt->SetName(scopeName.substr(0, scopeName.length() - 1));
      }
      OP_LOGI(kOpType.c_str(), "ScopeBatchMultiClassNonMaxSuppressionPass Scope fusion success.");
      return;
    }
    ```


