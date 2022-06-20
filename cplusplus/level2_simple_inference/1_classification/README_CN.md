中文|[English](README.md)

# 分类样例

#### 介绍
本仓包含多种分类样例，以供用户参考。目录结构和具体说明如下。

- **googlenet系列样例**
  | 样例名称  | 样例说明  | 特性解析  | 支持芯片 |
  |---|---|---|---|
  | [googlenet_imagenet_picture](./googlenet_imagenet_picture)  | 图片分类样例  | 输入和输出均为图片  | Ascend310 |
  | [googlenet_imagenet_multi_batch](./googlenet_imagenet_multi_batch)  | 多batch分类样例  | 输入为bin文件，输出为推理后的结果，使用了多batch的特性  | Ascend310 |
  | [googlenet_imagenet_dynamic_batch](./googlenet_imagenet_dynamic_batch)  | 动态batch分类样例  | 输入为bin文件，输出为推理后的结果，使用了动态batch的特性  | Ascend310 |
  | [googlenet_imagenet_video](./googlenet_imagenet_video)  | 视频分类样例  | 输入mp4文件，输出为presenter界面展现  | Ascend310 |

- **resnet50系列样例**
  | 样例名称  | 样例说明  | 特性解析  | 支持芯片 |
  |---|---|---|---|
  | [resnet50_imagenet_classification](./resnet50_imagenet_classification)  | 图片分类样例  | 基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类(同步推理)的功能  | Ascend310，Ascend310P，Ascend910 |
  | [resnet50_async_imagenet_classification](./resnet50_async_imagenet_classification)  | 图片分类样例  | 基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类(异步推理)的功能  | Ascend310，Ascend310P，Ascend910 |
  | [vdec_resnet50_classification](./vdec_resnet50_classification)  | 图片分类样例  | 基于Caffe ResNet-50网络实现图片分类（视频解码+同步推理）  | Ascend310，Ascend310P，Ascend910 |
  | [vpc_jpeg_resnet50_imagenet_classification](./vpc_jpeg_resnet50_imagenet_classification)  | 图片分类样例  | 基于Caffe ResNet-50网络实现图片分类（图片解码+抠图缩放+图片编码+同步推理）  | Ascend310，Ascend310P，Ascend910 |
  | [vpc_resnet50_imagenet_classification](./vpc_resnet50_imagenet_classification) | 图片分类样例 | 基于Caffe ResNet-50网络实现图片分类（图片解码+缩放+同步推理）| Ascend310，Ascend310P，Ascend910 |