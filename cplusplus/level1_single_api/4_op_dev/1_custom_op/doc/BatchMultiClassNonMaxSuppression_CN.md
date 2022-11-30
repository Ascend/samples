# ScopeBatchMultiClassNonMaxSuppressionPass样例参考<a name="ZH-CN_TOPIC_0298659575"></a>

## 概述<a name="section10369751135417"></a>

注：对融合规则无任何了解的，请先参看：[https://www.hiascend.com/document](https://www.hiascend.com/document)中的《TensorFlow Parser Scope融合规则开发》手册，上述手册给了完整的scope说明，下面仅给出当前样例的关键代码。

基于Tensorflow构建的神经网络其计算图通常由大量的小算子组成。为了实现高性能的网络推理计算，往往需要对这些小算子进行融合，使得融合后的大算子可以充分利用硬件加速资源。而融合规则开发时，当难以通过FE融合实现时，可考虑GE的scope融合实现（Scope融合是为了将某一个scope内的所有小算子进行替换成功能等价的大算子而提供的一个必要映射手段）。

GE的scope融合目标在于融合后的大算子可以调用TBE的算子进行计算加速。因此，既要识别出调用TBE的哪个API，又要提取出该API执行所需要的参数。从流程上来看，融合的步骤分为两个阶段：

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221017_0b190274_8200958.png "zh-cn_image_0298659578.png")

Scope辨识阶段需要确定scope是否对应Tensorflow的Python API，若是，则提取相应的参数。这里的参数包括两个方面：一是配置参数；二是计算参数。配置参数会影响scope下计算图的生成。计算参数是真正参与神经网络推理计算的数据，包括输入、权重、偏置等。

整个融合策略的核心在于确定scope的关键特征。这些关键特征表明了scope的唯一性。关键特征主要依据Python API源码和相应的计算公式确定。结合生成的计算图，关键特征大致分为以下几类：

-   0阶特征：不考虑scope内、外算子连接关系的特征。例如，用户不可修改的scope名称，典型算子数目，典型算子的属性值等；
-   1阶特征：考虑scope内、外算子的一步连接关系。例如，scope输入连接至ConcatV2算子；
-   2阶特征：考虑scope内、外算子的二步连接关系。例如，concat算子连接MatMul算子再连接BiasAdd算子；
-   以此类推，得到相应的高阶特征。

一般而言，特征的阶次越高，scope辨识所涉及的计算量就越大。因此，准确的找到合适的低阶特征是整个融合策略的难点也是核心所在。

注：Scope融合的规则需要按照该scope结构进行分析，去识别该scope内的一些特殊节点，保证开发的scope融合规则的唯一性\(能与你自己的网络匹配上即可，因为你开发的规则也有可能使其他网络误融合\)，如果有把握也可使其泛化\(但从目前的经验来看，不同的网络有不同的图结构，千差万别，很难做到泛化\)。

## 样例获取<a name="section122898421212"></a>

融合规则代码获取：
[https://gitee.com/ascend/samples/tree/master/cplusplus/level1\_single\_api/4\_op\_dev/1\_custom\_op/framework/tf\_scope\_fusion\_pass](https://gitee.com/ascend/samples/tree/master/cplusplus/level1_single_api/4_op_dev/1_custom_op/framework/tf_scope_fusion_pass)

## 融合规则设计思路<a name="section184331455516"></a>

本文融合规则设计以faster\_rcnn\_resnet101网络中SecondStagePostprocessor/BatchMultiClassNonMaxSuppression scope为例，该融合在整个网络中的结构如下：

具体pb模型获取路径：[https://github.com/tensorflow/models/blob/master/research/object\_detection/g3doc/tf1\_detection\_zoo.md](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md)

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221038_30b52046_8200958.png "zh-cn_image_0298659577.png")

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221052_0610d3a7_8200958.png "zh-cn_image_0298659576.png")

为什么要将scope进行融合呢？首先，经过源码分析，知道该scope的功能就是检测类网络的后处理（主要就是topk排序以及nms的功能），对此，我们提供能与之相同能力的算子batch\_multi\_class\_nms\_topk。那接下来就是如何替换的问题了。

分析图结构识别图中关键特征：

首先分析BatchMultiClassNonMaxSuppression这个scope的图结构：

BatchMultiClassNonMaxSuppression展开

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221108_4dc008e3_8200958.png "zh-cn_image_0298659925.png")

BatchMultiClassNonMaxSuppression/map展开

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221121_07c4885b_8200958.png "zh-cn_image_0298659580.png")

BatchMultiClassNonMaxSuppression/map/while/MultiClassNonMaxSuppression展开

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221136_c64011cd_8200958.png "zh-cn_image_0298659582.png")

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221146_262e3b6e_8200958.png "zh-cn_image_0298659579.png")

## 关键特征分析<a name="section7435163495712"></a>

BatchMultiClassNonMaxSuppression scope映射的是目前提供的后处理算子batch\_multi\_class\_nms\_topk，基于[https://github.com/tensorflow/models](https://github.com/tensorflow/models)上的源码实现，其功能主要包含\(topk,  nms, 以及clip\_to\_window\)，算子实现以及定义不在这里说明，只需要知道我们提供了与检测类网络后处理等价功能的算子即可。

BatchMultiClassNonMaxSuppression scope在Tensorflow计算图上的表现为一个子图，具有以下关键特征：

具体参见：
[https://gitee.com/ascend/samples/tree/master/cplusplus/level1\_single\_api/4\_op\_dev/1\_custom\_op/framework/tf\_scope\_fusion\_pass](https://gitee.com/ascend/samples/tree/master/cplusplus/level1_single_api/4_op_dev/1_custom_op/framework/tf_scope_fusion_pass)

![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221202_4d663acf_8200958.png "zh-cn_image_0298659584.png")

-   特征类型：0阶特征
-   特征名称：NonMaxSuppressionV2算子数目1个；Maximum算子数目4个

为了匹配更精确，可以多识别出一些0阶特征。

## 参数提取<a name="section131415810585"></a>

batch\_multi\_class\_nms\_topk算子所需要的计算参数如下，这几个参数都是作为替换后算子的属性传入\(在插件pass层以属性的形式传入\)：

-   名称：iou\_threshold

    说明：Iou阈值

    提取方式：从子图中名为NonMaxSuppressionV2的结点中提取，第四个输入

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221220_7766af63_8200958.png "zh-cn_image_0298659581.png")

-   名称：score\_threshold

    说明：分数阈值

    提取方式：Name为onMaxSuppression/FilterGreaterThan/Greater的第一个输入

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221309_c91e92d9_8200958.png "zh-cn_image_0298659927.png")

-   名称：max\_size\_per\_class

    说明：每一类最大输出框的个数

    提取方式：Minimum节点的第一个输入

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1207/221254_5cc49916_8200958.png "zh-cn_image_0298659926.png")


## 插件关键代码解析<a name="section14493169171319"></a>

从上图重获取了改融合算子的属性参数后需要将这些参数在plugin中进行设置\(并不是每个图的参数都如上图所示，多少有些差异，需要自己识别出来找到真的属性参数\)

1.  代码定义了

    ```
    static const char* const nms_v3 = "NonMaxSuppressionV3";
    static const char* const scoreConstKey = "map/while/MultiClassNonMaxSuppression/FilterGreaterThan/Greater";
    ```

2.  接下来，在图中找到节点，从其中的具体某一个输入位置去获取需要的iou\_threshold和score\_threshold，详见上表。

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

3.  然后再将由图中获取的参数设置到算子中去：

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


## 融合规则关键代码解析<a name="section118511811145912"></a>

1.  增加头文件：scope_batchmulticlass_nms_pass.h

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

2.  增加匹配规则代码：scope_batchmulticlass_nms_pass.cc

    -   定义Patterns及PassName：

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

    -   根据提取的关键特征，编写匹配模式，如下展示的是多种网络节点的匹配规则，以Pattern名字不同作区分，目前呈现的代码中有如下四种：
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

    -   由匹配规则判断匹配了哪一个分支，比如该scope规则里写了四种分支分别对应四种匹配规则：kScopeTypeBatchMultiClassNonMaxSuppression、
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

    -   匹配成功后，设置融合算子的输入输出

    写法1： scope\_batchmulticlass\_nms\_pass.cc

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

    写法2：scope\_newbatchmulticlass\_nms\_pass.cc

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


