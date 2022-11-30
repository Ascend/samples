中文|[English](README.md)

# 分类样例

#### 介绍
本目录包含多种分类样例，以供用户参考。目录结构和具体说明如下。

- **googlenet系列样例**
  | 样例名称  | 样例说明  | 特性解析 | 支持芯片 | 
  |---|---|---|---|
  | [googlenet_imagenet_picture](./googlenet_imagenet_picture)  | 图片分类  | 输入输出均为JPG图片，模型为基于Caffe的GoogLeNet模型  | Ascend310 |
  | [googlenet_mindspore_picture](./googlenet_mindspore_picture)  | 图片分类  | 输入输出均为JPG图片，模型为基于MindSpore的GoogLeNet模型  | Ascend310 |
  | [googlenet_onnx_picture](./googlenet_onnx_picture)  | 图片分类  | 输入输出均为JPG图片，模型为基于pytorch的GoogLeNet模型  | Ascend310 |
  | [googlenet_imagenet_multi_batch](./googlenet_imagenet_multi_batch)  | 图片分类  | 输入输出均为JPG图片，模型为基于Caffe的GoogLeNet模型，使用了多batch的特性  | Ascend310 |

- **resnet50系列样例**
  | 样例名称  | 样例说明  | 特性解析 | 支持芯片 |
  |---|---|---|---|
  | [resnet50_imagenet_classification](./resnet50_imagenet_classification)  | 图片分类  | 输入为JPG图片，输出为屏幕打印。基于 Caffe ResNet-50 网络实现图片分类（同步推理）  | Ascend310，Ascend310P，Ascend910 |
  | [resnet50_async_imagenet_classification](./resnet50_async_imagenet_classification)  | 图片分类  | 输入为JPG图片，输出为屏幕打印。基于 Caffe ResNet-50 网络实现图片分类（异步推理）  | Ascend310，Ascend310P，Ascend910 |
  | [resnet50_mindspore_picture](./resnet50_mindspore_picture)  | 图片分类  | 输入输出均为JPG图片。使用基于MindSpore的resnet50模型对输入图片进行分类推理  |  Ascend310 |
  | [vdec_resnet50_classification](./vdec_resnet50_classification)  | 图片分类  | 输入为h264文件，输出为屏幕打印。基于Caffe ResNet-50网络实现图片分类（视频解码+同步推理）  | Ascend310，Ascend310P，Ascend910 |
  | [vpc_jpeg_resnet50_imagenet_classification](./vpc_jpeg_resnet50_imagenet_classification)  | 图片分类  | 输入为YUV图片，输出为屏幕打印/JPG图片。基于 Caffe ResNet-50 网络实现图片分类（图片解码+抠图缩放+图片编码+同步推理） | Ascend310，Ascend310P，Ascend910 |
  | [vpc_resnet50_imagenet_classification](./vpc_resnet50_imagenet_classification)  | 图片分类  | 输入为JPG图片，输出为屏幕打印。基于Caffe ResNet-50网络实现图片分类（图片解码+缩放+同步推理）  | Ascend310，Ascend310P，Ascend910 |
  | [resnet50_imagenet_dynamic_hw](./resnet50_imagenet_dynamic_hw) | 图片分类 | 输入为JPG图片，输出为屏幕打印。基于 TensorFlow ResNet-50 网络实现图片分类（同步推理），使用了动态分辨率的特性 | Ascend310 |
  
- **其他样例**
  | 样例 | 说明  | 支持芯片 |
  |---|---|---|
  | [inceptionv3_picture](./inceptionv3_picture)  | 基于Pytorch框架的 IncpetionV3模型的图片分类样例  | Ascend310 |
  | [lenet_mindspore_picture](./lenet_mindspore_picture)   | 基于mindspore的lenet模型的图片文本分类样例  | Ascend310 |
  | [vgg16_cat_dog_picture](./vgg16_cat_dog_picture)   | 基于caffe框架的vgg16模型的猫狗分类样例  | Ascend310 |
