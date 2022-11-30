

# 小样本目标检测 (FsDet)

----
FsDet包含ICML 2020论文的官方小样本目标检测实现                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   

[简单小样本目标检测](https://arxiv.org/abs/2003.06957)：
![TFA Figure](https://user-images.githubusercontent.com/7898443/76520006-698cc200-6438-11ea-864f-fd30b3d50cea.png)

除了以前的论文中所使用的基准外，我们在PASCAL VOC、COCO和LVIS这三个数据集上引入了新的基准。我们对多组小样本训练示例进行多次实验，并将基类和新类的评估结果写在报告中。这些在 [Data Preparation](#data-preparation)中有更详细的描述。

我们还为本文中的两阶段微调方法( TFA )提供了基准结果和预训练模型。在TFA中，我们首先在数据丰富的基类上训练整个目标检测器，然后只在一个小的平衡训练集中微调检测器的最后一层。请参阅 [Models](#models) 了解我们提供的模型，并参阅 [Getting Started](#getting-started) 了解有关训练和评估的说明。

FsDet具有良好的模块化，因此可以方便地添加自己的数据集和模型。这个存储库的目标是为可用于未来研究的小样本目标检测提供一个通用框架。

```angular2html
@article{wang2020few,
    title={Frustratingly Simple Few-Shot Object Detection},
    author={Wang, Xin and Huang, Thomas E. and  Darrell, Trevor and Gonzalez, Joseph E and Yu, Fisher}
    booktitle = {International Conference on Machine Learning (ICML)},
    month = {July},
    year = {2020}
}
```

----
## Updates

- [Few-Shot Object Detection (FsDet)](#few-shot-object-detection-fsdet)
  - [Updates](#updates)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
  - [Code Structure](#code-structure)
  - [Data Preparation](#data-preparation)
  - [Models](#models)
  - [Getting Started](#getting-started)
    - [Inference Demo with Pre-trained Models](#inference-demo-with-pre-trained-models)
    - [Training & Evaluation in Command Line](#training--evaluation-in-command-line)
    - [Multiple Runs](#multiple-runs)


## Installation

环境要求：
* Linux 环境 Python 版本 >= 3.7
* [PyTorch](https://pytorch.org/get-started/locally/) 版本 >= 1.5，转onnx模型环境需要Pytorch版本 >= 1.8
* 安装匹配 PyTorch 版本的 [torchvision](https://github.com/pytorch/vision/) 库
* GCC 版本>= 4.9
* ONNX 版本 = 1.8.0

**构建 FsDet**

推荐使用ModelArts提供的Notebook进行环境搭建，具体参考docs/TRAIN_ON_NOTEBOOK.md文件；



## Code Structure
- **docs**: 相关文档
- **configs**: 配置文件
- **datasets**: 数据集文件 [Data Preparation](#data-preparation)
- **detectron2**: 修改后的detectron2源代码
- **fsdet**
  - **checkpoint**: 检查点代码
  - **config**: 配置代码和默认配置
  - **engine**: 包含训练和评估的循环和挂钩
  - **layers**: 模型中神经网络不同层的实现
  - **modeling**: 模型的代码，包括Backbone,RPN等
- **output**: 输出结果的目标文件夹
- **tools**
  - **train_net.py**: 训脚本
  - **test_net.py**: 测试脚本
  - **ckpt_surgery.py**: 权重合并脚本
  - **run_experiments.py**: 进行c-way k-shot实验
  - **aggregate_seeds.py**: 集成多Seeds集的实验结果
  - **deploy**
    - **export_model.py**: 导出onnx模型文件
    - **fsdet.py**: om推理脚本文件
    - **metric.py**: 包含推理性能度量相关代码

## Data Preparation

我们在三个公开数据集及一个自定义数据集（金相数据）上评估我们的模型:

- [PASCAL VOC](http://host.robots.ox.ac.uk/pascal/VOC/): 我们使用PASCAL VOC 2007 + 2012的train / val集进行训练，使用PASCAL VOC 2007的test集进行评估。我们将20个对象类随机拆分为15个基类和5个新类，并考虑3个随机拆分。这些拆分可以在 [fsdet/data/builtin_meta.py](fsdet/data/builtin_meta.py)中找到。
- [COCO](http://cocodataset.org/): 我们使用COCO 2014并从val集中提取5千张图像用于评估，其余的用于训练。我们使用与PASCAL VOC相同的20个对象类作为新类，其余的作为基类。
- [LVIS](https://www.lvisdataset.org/): 我们将频繁类和常见类作为基类，稀有类作为新类。
- 工业焊缝缺陷金相数据集（PSCAL VOC文件结构）：由于只有两类缺陷，种类较少，且原始数据很少，因此此处采用泊松融合进行数据增强，后直接作为基类样本进行训练。

若由于工业缺陷样本太少，正常样本较多，可考虑进行泊松融合合成新的样本，从而扩充缺陷样本的数量，详情见 [PossionFusion/README.md](PossionFusion/README.md) .



对于Fsdet,训练前需要进行数据集注册操作：

​		a: 若数据集为PASCAL_VOC,COCO,LVIS则无需额外注册，代码中有相关注册代码，运行即可。

​		b: 若数据集为自己定义的普通数据集，则在训练前需要进行数据注册，请参阅[CUSTOM.md](docs/CUSTOM.md)说明。

## Models

我们在 [MODEL_ZOO.md](docs/MODEL_ZOO.md)中提供了一组可供下载的基准结果和预训练模型。


## Getting Started

### Inference Demo with Pre-trained Models

1. 从 [model zoo](fsdet/model_zoo/model_zoo.py)中选择一个模型及其配置文件，例如 `COCO-detection/faster_rcnn_R_101_FPN_ft_all_1shot.yaml`。
2. 我们提供 `demo.py`文件 ，它能够运行内置的标准模型。用它运行：

```
python3 -m demo.demo \
  --config-file configs/COCO-detection/faster_rcnn_R_101_FPN_ft_all_1shot.yaml \
  --input input1.jpg input2.jpg \
  --output output \
  --opts MODEL.WEIGHTS fsdet://coco/tfa_cos_1shot/model_final.pth\
  --confidence-threshold 0.5
 
 或着直接再demo.sh文件中进行参数修改，然后直接运行命令：bash demo.sh 即可
 
 注意：要保证config-file与MODEL.WEIGHT是同属于一个模型
```

配置是为了训练，因此需要将 `MODEL.WEIGHTS` 指定给model zoo的模型进行评估。该命令将对图片进行推理将结果保存在output文件中。

有关命令行参数的详细信息，请参阅 `demo.py -h` 或查看其源代码以了解其作用。一些常见的论点是：

* 运行 __on your webcam__, 要用 `--webcam`替代 `--input files` .
* 运行 __on a video__, 要用 `--video-input video.mp4`替代`--input files` .
* 运行 __on cpu__, 在 `--opts` 后添加 `MODEL.DEVICE cpu` (此处device默认为npu)
* 若要将输出保存到目录(图像类)或文件(webcam或视频类)，请使用 `--output`。 

### Training & Evaluation in Command Line

运行以下代码进行训练

```angular2html
python3 -m tools.train_net \
        --config-file configs/PascalVOC-detection/split1/faster_rcnn_R_101_FPN_base1.yaml
 
或者在train.sh中进行参数配置，然后直接运行命令：bash train.sh
```

运行以下代码来评估训练好的模型

```angular2html
python3 -m tools.test_net \
        --config-file configs/PascalVOC-detection/split1/faster_rcnn_R_101_FPN_ft_all1_1shot.yaml \
        --eval-only
        
或者在test.sh中进行参数配置，然后直接运行命令：bash test.sh
```

更多关于TFA训练流程的详细说明，见 [TRAIN_INST.md](docs/TRAIN_INST.md)。

### Multiple Runs

为了便于多次运行的训练和评估，我们在 `tools/`.中提供了几个有用的脚本。

您可以使用 `tools/run_experiments.py` 进行培训和评估。例如，运行以下代码在所有样本上对PascalVOC的第一次分割的30个种子进行实验：

```angular2html
python3 -m tools.run_experiments \
        --shots 1 2 3 5 10 --seeds 0 30 --split 1

或者在run_experiments.sh中进行参数配置，然后直接运行命令：bash run_experiments.sh
```

在训练和评估之后，您可以使用 `tools/aggregate_seeds.py` 将所有seed的结果进行汇集，以获得一组数据。运行以下代码来聚合上述命令的3 shot结果：

```angular2html
python3 -m tools.aggregate_seeds --shots 3 --seeds 30 --split 1 \
        --print --plot
        
或者在run_aggregate_seeds.sh中进行参数配置，然后直接运行命令行：bash run_aggregate_seeds.sh
```

### Deployment
#### Env Requirement：
目前仅支持使用CPU环境进行ONNX模型转换（按照指定的要求配置环境），其他硬件环境没有通过验证；
```
# Transform
pytorch==1.8
torchvision==0.9.0
onnx==1.8.0
numpy
opencv_python

# Deploy
tqdm
tabulate
```

#### PTH to ONNX：
由于Detectron2使用的caffe2框架目前在转onnx模型时支持还不完善，需要对detectron2源代码进行一定的修改（具体修改内容见fsdet_deploy.diff文件），部分算子仅支持部署环境。
[修改参考](https://gitee.com/wangjiangben_hw/ascend-pytorch-crowdintelligence-doc/blob/master/Ascend-PyTorch%E7%A6%BB%E7%BA%BF%E6%8E%A8%E7%90%86%E6%8C%87%E5%AF%BC/ONNX%E6%A8%A1%E5%9E%8B%E6%8E%A8%E7%90%86%E6%8C%87%E5%AF%BC/benchmark/cv/segmentation/%E5%9F%BA%E4%BA%8E%E5%BC%80%E6%BA%90detectron2%E8%AE%AD%E7%BB%83%E7%9A%84npu%E6%9D%83%E9%87%8D%E7%9A%84maskrcnn%E6%A8%A1%E5%9E%8B%E6%8E%A8%E7%90%86%E6%8C%87%E5%AF%BC.md)

1. 配置好模型转化环境
```bash
bash setup_deploy.sh
```
> 在NPU环境安装opencv参考：第三方依赖安装指导（python样例）
```bash
sudo apt-get install python3-pip
# 安装python库
python3.7 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
python3.7 -m pip install Cython numpy tornado==5.1.0 protobuf --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
# 安装python3-opencv
sudo apt-get install python3-opencv
```

2. 准备好训练好的pth权重文件

3. 修改对应的detectron2（v0.5）源码（fsdet_deploy.diff），直接使用下面的命令即可：
```bash
cd detectron2_deploy
git apply --stat ../fsdet_deploy.diff  # 检查diff文件
git apply --check ../fsdet_deploy.diff  # 检查能否应用成功
git apply ../fsdet_deploy.diff  # 打补丁
cd ..
```
```diff
diff --git a/detectron2/layers/__init__.py b/detectron2/layers/__init__.py
index c8bd1fb024d1cb911dda3f8a77f7ec3ad2e63798..f5fa9ea3186cb22b8e85f125fe3f22ec44180332 100644
--- a/detectron2/layers/__init__.py
+++ b/detectron2/layers/__init__.py
@@ -2,7 +2,7 @@
 from .batch_norm import FrozenBatchNorm2d, get_norm, NaiveSyncBatchNorm
 from .deform_conv import DeformConv, ModulatedDeformConv
 from .mask_ops import paste_masks_in_image
-from .nms import batched_nms, batched_nms_rotated, nms, nms_rotated
+from .nms import batched_nms, batch_nms_op, batched_nms_rotated, nms, nms_rotated
 from .roi_align import ROIAlign, roi_align
 from .roi_align_rotated import ROIAlignRotated, roi_align_rotated
 from .shape_spec import ShapeSpec
diff --git a/detectron2/layers/nms.py b/detectron2/layers/nms.py
index e753d6af0fcbc8c5d1ca8cfbc92922eaa7039452..ffc704e98840a5ae5a2142271c5b7155ed8e8759 100644
--- a/detectron2/layers/nms.py
+++ b/detectron2/layers/nms.py
@@ -6,6 +6,56 @@ import torch
 from torchvision.ops import boxes as box_ops
 from torchvision.ops import nms  # BC-compat
 
+class BatchNMSOp(torch.autograd.Function):
+    @staticmethod
+    def forward(ctx, bboxes, scores, score_threshold, iou_threshold, max_size_per_class, max_total_size):
+        """
+        boxes (torch.Tensor): boxes in shape (batch, N, C, 4).
+        scores (torch.Tensor): scores in shape (batch, N, C).
+        return:
+            nmsed_boxes: (1, N, 4)
+            nmsed_scores: (1, N)
+            nmsed_classes: (1, N)
+            nmsed_num: (1,)
+        """
+
+        # Phony implementation for onnx export
+        nmsed_boxes = bboxes[:, :max_total_size, 0, :]
+        nmsed_scores = scores[:, :max_total_size, 0]
+        nmsed_classes = torch.arange(max_total_size, dtype=torch.long)
+        nmsed_num = torch.Tensor([max_total_size])
+
+        return nmsed_boxes, nmsed_scores, nmsed_classes, nmsed_num
+
+    @staticmethod
+    def symbolic(g, bboxes, scores, score_thr, iou_thr, max_size_p_class, max_t_size):
+        nmsed_boxes, nmsed_scores, nmsed_classes, nmsed_num = g.op('BatchMultiClassNMS',
+            bboxes, scores, score_threshold_f=score_thr, iou_threshold_f=iou_thr,
+            max_size_per_class_i=max_size_p_class, max_total_size_i=max_t_size, outputs=4)
+        return nmsed_boxes, nmsed_scores, nmsed_classes, nmsed_num
+
+def batch_nms_op(bboxes, scores, score_threshold, iou_threshold, max_size_per_class, max_total_size):
+    """
+    boxes (torch.Tensor): boxes in shape (N, 4).
+    scores (torch.Tensor): scores in shape (N, ).
+    """
+
+    num_classes = bboxes.shape[1].numpy() // 4
+    if bboxes.dtype == torch.float32:
+        bboxes = bboxes.reshape(1, bboxes.shape[0].numpy(), -1, 4).half()
+        scores = scores.reshape(1, scores.shape[0].numpy(), -1).half()
+    else:
+        bboxes = bboxes.reshape(1, bboxes.shape[0].numpy(), -1, 4)
+        scores = scores.reshape(1, scores.shape[0].numpy(), -1)
+
+    nmsed_boxes, nmsed_scores, nmsed_classes, nmsed_num = BatchNMSOp.apply(bboxes, scores,
+        score_threshold, iou_threshold, max_size_per_class, max_total_size)
+    nmsed_boxes = nmsed_boxes.float()
+    nmsed_scores = nmsed_scores.float()
+    nmsed_classes = nmsed_classes.long()
+    dets = torch.cat((nmsed_boxes.reshape((max_total_size, 4)), nmsed_scores.reshape((max_total_size, 1))), -1)
+    labels = nmsed_classes.reshape((max_total_size, ))
+    return dets, labels
 
 def batched_nms(
     boxes: torch.Tensor, scores: torch.Tensor, idxs: torch.Tensor, iou_threshold: float
diff --git a/detectron2/modeling/box_regression.py b/detectron2/modeling/box_regression.py
index 12be0008b66bd4954a5139aeb6e07d71f8159caa..5aa3acea9d83b9cad1684339125e27b444536adc 100644
--- a/detectron2/modeling/box_regression.py
+++ b/detectron2/modeling/box_regression.py
@@ -87,20 +87,39 @@ class Box2BoxTransform(object):
         deltas = deltas.float()  # ensure fp32 for decoding precision
         boxes = boxes.to(deltas.dtype)
 
-        widths = boxes[:, 2] - boxes[:, 0]
-        heights = boxes[:, 3] - boxes[:, 1]
-        ctr_x = boxes[:, 0] + 0.5 * widths
-        ctr_y = boxes[:, 1] + 0.5 * heights
+        # widths = boxes[:, 2] - boxes[:, 0]
+        # heights = boxes[:, 3] - boxes[:, 1]
+        # ctr_x = boxes[:, 0] + 0.5 * widths
+        # ctr_y = boxes[:, 1] + 0.5 * heights
+        boxes_prof = boxes.permute(1, 0)
+        widths = boxes_prof[2, :] - boxes_prof[0, :]
+        heights = boxes_prof[3, :] - boxes_prof[1, :]
+        ctr_x = boxes_prof[0, :] + 0.5 * widths
+        ctr_y = boxes_prof[1, :] + 0.5 * heights
 
         wx, wy, ww, wh = self.weights
-        dx = deltas[:, 0::4] / wx
+        """dx = deltas[:, 0::4] / wx
         dy = deltas[:, 1::4] / wy
         dw = deltas[:, 2::4] / ww
-        dh = deltas[:, 3::4] / wh
+        dh = deltas[:, 3::4] / wh"""
+        denorm_deltas = deltas
+        if denorm_deltas.shape[1] > 4:
+            denorm_deltas = denorm_deltas.view(-1, 80, 4)
+            dx = denorm_deltas[:, :, 0:1:].view(-1, 80) / wx
+            dy = denorm_deltas[:, :, 1:2:].view(-1, 80) / wy
+            dw = denorm_deltas[:, :, 2:3:].view(-1, 80) / ww
+            dh = denorm_deltas[:, :, 3:4:].view(-1, 80) / wh
+        else:
+            dx = denorm_deltas[:, 0:1:] / wx
+            dy = denorm_deltas[:, 1:2:] / wy
+            dw = denorm_deltas[:, 2:3:] / ww
+            dh = denorm_deltas[:, 3:4:] / wh
 
         # Prevent sending too large values into torch.exp()
-        dw = torch.clamp(dw, max=self.scale_clamp)
-        dh = torch.clamp(dh, max=self.scale_clamp)
+        # dw = torch.clamp(dw, max=self.scale_clamp)
+        # dh = torch.clamp(dh, max=self.scale_clamp)
+        dw = torch.clamp(dw, min=-float('inf'), max=self.scale_clamp)
+        dh = torch.clamp(dh, min=-float('inf'), max=self.scale_clamp)
 
         pred_ctr_x = dx * widths[:, None] + ctr_x[:, None]
         pred_ctr_y = dy * heights[:, None] + ctr_y[:, None]
diff --git a/detectron2/modeling/poolers.py b/detectron2/modeling/poolers.py
index e5d72abf462ebc9c2ac9ad9dd7c6cc39eac4054c..382b7131f2e82fc7748454f57c2cf8976bbed1d3 100644
--- a/detectron2/modeling/poolers.py
+++ b/detectron2/modeling/poolers.py
@@ -202,6 +202,13 @@ class ROIPooler(nn.Module):
                 A tensor of shape (M, C, output_size, output_size) where M is the total number of
                 boxes aggregated over all N batch images and C is the number of channels in `x`.
         """
+        
+        if torch.onnx.is_in_onnx_export():
+            output_size = self.output_size[0]
+            pooler_fmt_boxes = convert_boxes_to_pooler_format(box_lists)
+            roi_feats = RoiExtractor.apply(x[0], x[1], x[2], x[3], pooler_fmt_boxes, 0, 56, output_size, output_size)
+            return roi_feats
+        
         num_level_assignments = len(self.level_poolers)
 
         assert isinstance(x, list) and isinstance(
@@ -248,3 +255,29 @@ class ROIPooler(nn.Module):
             output.index_put_((inds,), pooler(x[level], pooler_fmt_boxes_level))
 
         return output
+
+import torch.onnx.symbolic_helper as sym_help
+
+class RoiExtractor(torch.autograd.Function):
+    @staticmethod
+    def forward(self, f0, f1, f2, f3, rois, aligned=0, finest_scale=56, pooled_height=7, pooled_width=7,
+                         pool_mode='avg', roi_scale_factor=0, sample_num=0, spatial_scale=[0.25, 0.125, 0.0625, 0.03125]):
+        """
+        feats (torch.Tensor): feats in shape (batch, 256, H, W).
+        rois (torch.Tensor): rois in shape (k, 5).
+        return:
+            roi_feats (torch.Tensor): (k, 256, pooled_width, pooled_width)
+        """
+
+        # phony implementation for shape inference
+        k = rois.size()[0]
+        roi_feats = torch.ones(k, 256, pooled_height, pooled_width)
+        return roi_feats
+
+    @staticmethod
+    def symbolic(g, f0, f1, f2, f3, rois, aligned=0, finest_scale=56, pooled_height=7, pooled_width=7):
+        # TODO: support tensor list type for feats
+        #f_tensors = sym_help._unpack_list(feats)
+        roi_feats = g.op('RoiExtractor', f0, f1, f2, f3, rois, aligned_i=0, finest_scale_i=56, pooled_height_i=pooled_height, pooled_width_i=pooled_width,
+                         pool_mode_s='avg', roi_scale_factor_i=0, sample_num_i=0, spatial_scale_f=[0.25, 0.125, 0.0625, 0.03125], outputs=1)
+        return roi_feats
\ No newline at end of file
diff --git a/detectron2/modeling/proposal_generator/proposal_utils.py b/detectron2/modeling/proposal_generator/proposal_utils.py
index 7c00dc5d959beed890231a6ed6693a71ab1035da..94f57b5b3452e86b188b642f2ac76acf41e366c9 100644
--- a/detectron2/modeling/proposal_generator/proposal_utils.py
+++ b/detectron2/modeling/proposal_generator/proposal_utils.py
@@ -4,7 +4,8 @@ import math
 from typing import List, Tuple, Union
 import torch
 
-from detectron2.layers import batched_nms, cat
+#from detectron2.layers import batched_nms, cat
+from detectron2.layers import batch_nms_op, cat
 from detectron2.structures import Boxes, Instances
 
 logger = logging.getLogger(__name__)
@@ -68,15 +69,22 @@ def find_top_rpn_proposals(
     for level_id, (proposals_i, logits_i) in enumerate(zip(proposals, pred_objectness_logits)):
         Hi_Wi_A = logits_i.shape[1]
         if isinstance(Hi_Wi_A, torch.Tensor):  # it's a tensor in tracing
-            num_proposals_i = torch.clamp(Hi_Wi_A, max=pre_nms_topk)
+            #num_proposals_i = torch.clamp(Hi_Wi_A, max=pre_nms_topk)
+            num_proposals_i = torch.clamp(Hi_Wi_A, min=0, max=pre_nms_topk)
         else:
             num_proposals_i = min(Hi_Wi_A, pre_nms_topk)
 
         # sort is faster than topk: https://github.com/pytorch/pytorch/issues/22812
         # topk_scores_i, topk_idx = logits_i.topk(num_proposals_i, dim=1)
-        logits_i, idx = logits_i.sort(descending=True, dim=1)
+        num_proposals_i = num_proposals_i.item()
+        logits_i = logits_i.reshape(logits_i.size(1))
+        topk_scores_i, topk_idx = torch.topk(logits_i, num_proposals_i)
+        topk_scores_i = topk_scores_i.reshape(1, topk_scores_i.size(0))
+        topk_idx = topk_idx.reshape(1, topk_idx.size(0))
+
+        """logits_i, idx = logits_i.sort(descending=True, dim=1)
         topk_scores_i = logits_i.narrow(1, 0, num_proposals_i)
-        topk_idx = idx.narrow(1, 0, num_proposals_i)
+        topk_idx = idx.narrow(1, 0, num_proposals_i)"""
 
         # each is N x topk
         topk_proposals_i = proposals_i[batch_idx[:, None], topk_idx]  # N x topk x 4
@@ -108,7 +116,7 @@ def find_top_rpn_proposals(
             lvl = lvl[valid_mask]
         boxes.clip(image_size)
 
-        # filter empty boxes
+        """# filter empty boxes
         keep = boxes.nonempty(threshold=min_box_size)
         if _is_tracing() or keep.sum().item() != len(boxes):
             boxes, scores_per_img, lvl = boxes[keep], scores_per_img[keep], lvl[keep]
@@ -126,6 +134,11 @@ def find_top_rpn_proposals(
         res = Instances(image_size)
         res.proposal_boxes = boxes[keep]
         res.objectness_logits = scores_per_img[keep]
+        results.append(res)"""
+        dets, labels = batch_nms_op(boxes.tensor, scores_per_img, 0, nms_thresh, post_nms_topk, post_nms_topk)
+        res = Instances(image_size)
+        res.proposal_boxes = Boxes(dets[:, :4])
+        res.objectness_logits = dets[:, 4]
         results.append(res)
     return results
 
diff --git a/detectron2/modeling/proposal_generator/rpn.py b/detectron2/modeling/proposal_generator/rpn.py
index 99cd536d2f9880d2049390c45f73eb22335e1b82..425cbadd219640caa6f4652ef2893fbf1e5fb617 100644
--- a/detectron2/modeling/proposal_generator/rpn.py
+++ b/detectron2/modeling/proposal_generator/rpn.py
@@ -475,7 +475,8 @@ class RPN(nn.Module):
         else:
             losses = {}
         proposals = self.predict_proposals(
-            anchors, pred_objectness_logits, pred_anchor_deltas, images.image_sizes
+            #anchors, pred_objectness_logits, pred_anchor_deltas, images.image_sizes
+            anchors, pred_objectness_logits, pred_anchor_deltas, [(800, 800)]
         )
         return proposals, losses
 
@@ -526,7 +527,9 @@ class RPN(nn.Module):
             B = anchors_i.tensor.size(1)
             pred_anchor_deltas_i = pred_anchor_deltas_i.reshape(-1, B)
             # Expand anchors to shape (N*Hi*Wi*A, B)
-            anchors_i = anchors_i.tensor.unsqueeze(0).expand(N, -1, -1).reshape(-1, B)
+            # anchors_i = anchors_i.tensor.unsqueeze(0).expand(N, -1, -1).reshape(-1, B)
+            s = torch.zeros(N, anchors_i.tensor.unsqueeze(0).size(1), anchors_i.tensor.unsqueeze(0).size(2))
+            anchors_i = anchors_i.tensor.unsqueeze(0).expand_as(s).reshape(-1, B)
             proposals_i = self.box2box_transform.apply_deltas(pred_anchor_deltas_i, anchors_i)
             # Append feature map proposals with shape (N, Hi*Wi*A, B)
             proposals.append(proposals_i.view(N, -1, B))
diff --git a/detectron2/structures/boxes.py b/detectron2/structures/boxes.py
index 6d8762d60a873c6b6daa42e9e7fcac41eda32fec..75194cef01638de384d7d794200f56a54d013137 100644
--- a/detectron2/structures/boxes.py
+++ b/detectron2/structures/boxes.py
@@ -199,10 +199,15 @@ class Boxes:
         """
         assert torch.isfinite(self.tensor).all(), "Box tensor contains infinite or NaN!"
         h, w = box_size
-        x1 = self.tensor[:, 0].clamp(min=0, max=w)
+        """x1 = self.tensor[:, 0].clamp(min=0, max=w)
         y1 = self.tensor[:, 1].clamp(min=0, max=h)
         x2 = self.tensor[:, 2].clamp(min=0, max=w)
-        y2 = self.tensor[:, 3].clamp(min=0, max=h)
+        y2 = self.tensor[:, 3].clamp(min=0, max=h)"""
+        boxes_prof = self.tensor.permute(1, 0)
+        x1 = boxes_prof[0, :].clamp(min=0, max=w)
+        y1 = boxes_prof[1, :].clamp(min=0, max=h)
+        x2 = boxes_prof[2, :].clamp(min=0, max=w)
+        y2 = boxes_prof[3, :].clamp(min=0, max=h)
         self.tensor = torch.stack((x1, y1, x2, y2), dim=-1)
 
     def nonempty(self, threshold: float = 0.0) -> torch.Tensor:
```
4. 修改onnx代码，避免其缺少算子报错（把对应文件拷贝到指定地址）：
```bash
cp utils_modified.py /home/ma-user/anaconda3/envs/PyTorch-1.8/lib/python3.7/site-packages/torch/onnx/utils.py 
```
5. 运行export.sh脚本，将pth模型转换为onnx模型：
```bash
bash export.sh > export.log
```
export.sh脚本内容
```bash
#!/bin/bash
python3 tools/deploy/export_model.py \
        --config-file configs/Custom-detection/faster_rcnn_R_101_FPN_base1_jinxiang.yaml \
        --output ./output --export-method tracing \
        --format onnx MODEL.WEIGHTS checkpoints/voc/faster_rcnn/faster_rcnn_R_101_FPN_base1_jinxiang/model_final.pth
 MODEL.DEVICE cpu
mv output/model.onnx model_py1.8.onnx
```

#### ONNX to OM
目前仅支持在Ascend310处理器上使用ATC工具进行om模型转换。
1. 设置环境变量

CANN工具版本要求：
```angular2html
version=5.1.RC2.alpha007
innerversion=V100R001C82B220SPC008
arch=aarch64
os=linux
```
设置环境变量
```bash
export install_path=/usr/local/Ascend/ascend-toolkit/latest
export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
export PYTHONPATH=${install_path}/atc/python/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=${install_path}/atc/lib64:${install_path}/acllib/lib64:$LD_LIBRARY_PATH
export ASCEND_OPP_PATH=${install_path}/opp
export ASCEND_AICPU_PATH=/usr/local/Ascend/ascend-toolkit/latest/
```
2. 使用atc将onnx模型转换为om模型文件，工具使用方法可以参考[CANN V100R020C10 开发辅助工具指南 (推理) 01](https://support.huawei.com/enterprise/zh/doc/EDOC1100164868?idPath=23710424|251366513|22892968|251168373)
3. 需要指定输出节点以去除无用输出，使用netron开源可视化工具查看onnx模型的具体的输出节点名；
4. 使用如下命令转换模型：
```bash
atc --framework=5 --model=model_py1.8.onnx --output=fsdet_model --input_format=NCHW --input_shape="0:1,3,800,800" --out_nodes="Gather_1692:0;Cast_1689:0;Reshape_1683:0" --log=debug --soc_version=Ascend310
```

#### Running Deployment Script

1. 安装运行推理脚本所需的python库：
```bash
pip3 install tqdm
pip3 install tabulate
```

2. 根据上面的步骤准备好om模型，按照VOC文件格式准备好数据集，设置模型输入的图片尺寸以及置信度阈值和IOU阈值，然后调用fsdet_deploy.py脚本进行推理：
```bash
python3.6 tools/deploy/fsdet_deploy.py --model=fsdet_model.om \
       --height=800 --width=800 --mode=metric --txt=datasets/VOC-custom/ImageSets/Main/test.txt \ 
       --split=test --data_dir=datasets/VOC-custom/ --cf_thres=0.05 --iou_thres=0.5
```

金相数据结果：

| AP      | AP50   | AP75   |
|---------|--------|--------|
| 62.306  | 86.229 | 74.696 |

推理速度：0.195 s/img


推理结果分析：利用om模型进行推理精度下降，可能是由于转换模型是使用的部分替换算子以及固定模型输入尺寸导致，在该模型框架下的推理速度基本达标。

## Reference:

code:[ucbdrive/few-shot-object-detection: Implementations of few-shot object detection benchmarks (github.com)](https://github.com/ucbdrive/few-shot-object-detection)

paper:[Frustratingly Simple Few-Shot Object Detection](https://arxiv.org/abs/2003.06957)