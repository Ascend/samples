中文|[English](README_EN.md)

## 昇腾样例仓介绍
   
AscendCL（Ascend Computing Language）提供Device管理、Context管理、Stream管理、内存管理、模型加载与执行、算子加载与执行、媒体数据处理等C语言API库供用户开发深度神经网络应用，用于实现目标识别、图像分类等功能。用户可以通过第三方框架调用AscendCL接口，以便使用昇腾AI处理器的计算能力；用户还可以使用AscendCL封装实现第三方lib库，以便提供昇腾AI处理器的运行管理、资源管理能力。

昇腾样例仓就是以ACL接口进行开发，制作的一系列给开发者进行参考学习的样例。在开发者朋友们开发自己的样例时，也可以就样例仓的相关案例进行参考。

## 版本说明

**master分支样例版本适配情况请参见[样例表单及适配说明](#Version-of-samples),如果需要使用的案例不兼容您所需要的版本，请参考[表 版本说明](#Version-Description)下载对应发行版**

**表1** 版本说明<a name="Version-Description"></a>
| CANN版本 | samples仓是否维护 | samples获取方式 |
|---|---|---|
| [20.0-20.1](https://ascend.huawei.com/#/software/cann/download) | 是 | Release 0.1.0发行版，[点击跳转](https://gitee.com/ascend/samples/releases/v0.1.0) |

## 目录结构与说明

**./**   
├── [c++](./cplusplus)：**C++样例。**    
│   ├── common   
│   ├── contrib   
│   ├── level1_single_api   
│   ├── level2_simple_inference   
│   └── level3_multi_model   
├── [ci](./ci)：**自动构建脚本。**    
├── [doc](./doc)：**相关依赖安装文档。**    
│   ├── opencv_install   
│   ├── presenteragent_install   
│   └── python3_ENV_install   
├── [python](./python)：**python样例。**  
│   ├── common   
│   ├── contrib   
│   ├── level1_single_api   
│   ├── level2_simple_inference   
└   └── level3_multi_model   

## 样例表单及适配说明<a name="Version-of-samples"></a>

| 样例名称 | 语言 | 适配版本 | 适配产品 |
|---|---|---|---|
| [crop](./cplusplus/level1_single_api/1_acl/4_dvpp/crop) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [vdec](./cplusplus/level1_single_api/1_acl/4_dvpp/vdec) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [venc](./cplusplus/level1_single_api/1_acl/4_dvpp/venc) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [jpegd](./cplusplus/level1_single_api/1_acl/4_dvpp/jpegd) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [jpege](./cplusplus/level1_single_api/1_acl/4_dvpp/jpege) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [resize](./cplusplus/level1_single_api/1_acl/4_dvpp/resize) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [batchcrop](./cplusplus/level1_single_api/1_acl/4_dvpp/batchcrop) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [venc_image](./cplusplus/level1_single_api/1_acl/4_dvpp/venc_image) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [vdecandvenc](./cplusplus/level1_single_api/1_acl/4_dvpp/vdecandvenc) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [cropandpaste](./cplusplus/level1_single_api/1_acl/4_dvpp/cropandpaste) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [smallResolution_cropandpaste](./cplusplus/level1_single_api/1_acl/4_dvpp/smallResolution_cropandpaste) | c++ | 20.0/20.1 | Atlas200DK/Atlas300 |
| [gemm](./cplusplus/level1_single_api/1_acl/5_blas/gemm) |  c++ |20.0/20.1  | Atlas200DK/Atlas300  |
| [acl_execute_add](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_add) |  c++ |20.0/20.1  | Atlas200DK/Atlas300  |
| [acl_execute_batchnorm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_batchnorm) |  c++ |20.0/20.1  | Atlas200DK/Atlas300  |
| [acl_execute_conv2d](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_conv2d) |  c++ |20.0/20.1  | Atlas200DK/Atlas300  |
| [acl_execute_lstm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_lstm) |  c++ |20.0/20.1  | Atlas200DK/Atlas300  |
| [acl_execute_matmul](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_matmul) |  c++ |20.0/20.1  | Atlas200DK/Atlas300  |
| [acl_execute_reshape](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_reshape) |  c++ |20.0/20.1  | Atlas200DK/Atlas300  |
| [gpio](./cplusplus/level1_single_api/5_200dk_peripheral) |  c++ |20.0/20.1  | Atlas200DK |
| [i2c](./cplusplus/level1_single_api/5_200dk_peripheral/i2c) |  c++ |20.0/20.1  | Atlas200DK |
| [uart](./cplusplus/level1_single_api/5_200dk_peripheral/uart) |  c++ |20.0/20.1  | Atlas200DK |
| [ascendcamera](./cplusplus/level1_single_api/5_200dk_peripheral/ascendcamera) |  c++ |20.0/20.1  | Atlas200DK |
| [googlenet_imagenet_video](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_video) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [googlenet_imagenet_picture](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [vdec_resnet50_classification](./cplusplus/level2_simple_inference/1_classification/vdec_resnet50_classification) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [resnet50_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [resnet50_async_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_async_imagenet_classification) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [googlenet_imagenet_dynamic_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_dynamic_batch) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [googlenet_imagenet_multi_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [vpc_resnet50_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [vpc_jpeg_resnet50_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification) | c++ | 20.0/20.1  | Atlas200DK/Atlas300 |
| [face_detection_camera](./cplusplus/level2_simple_inference/2_object_detection/face_detection_camera) |  c++ |20.0/20.1  | Atlas200DK |
| [YOLOV3_coco_detection_video](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_coco_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV4_coco_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_picture) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_VOC_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_VOC_detection_picture) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_coco_detection_VENC](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_VENC) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [VGG_SSD_coco_detection_CV_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_with_AIPP) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [VGG_SSD_coco_detection_CV_without_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_without_AIPP) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [VGG_SSD_coco_detection_DVPP_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_coco_detection_dynamic_AIPP](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_dynamic_AIPP) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_dynamic_batch_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [faste_RCNN_VOC_detection_dynamic_resolution](./cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [WAV_to_word](./cplusplus/level2_simple_inference/5_nlp/WAV_to_word) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [colorization](./cplusplus/level2_simple_inference/6_other/colorization) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [colorization_video](./cplusplus/level2_simple_inference/6_other/colorization_video) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_coco_detection_multi_thread_VENC](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/YOLOV3_coco_detection_multi_thread_VENC) |  c++ |20.0/20.1  | Atlas200DK/Atlas300 |
| [face_recognition_camera](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera) |  c++ |20.1  | Atlas200DK |
| [googlenet_imagenet_multi_batch](./python/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) |  python |20.0/20.1  | Atlas200DK/Atlas300 |
| [googlenet_imagenet_picture](./python/level2_simple_inference/1_classification/googlenet_imagenet_picture) | python |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_coco_detection_picture](./python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) | python |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_coco_detection_video](./python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video) | python |20.0/20.1  | Atlas200DK/Atlas300 |
| [YOLOV3_mask_detection_camera](./python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_camera) | python |20.0/20.1  | Atlas200DK |
| [face_detection_camera](./python/level2_simple_inference/2_object_detection/face_detection_camera) | python |20.0/20.1  | Atlas200DK |
| [colorization_picture](./python/level2_simple_inference/6_other/colorization_picture) | python |20.0/20.1  | Atlas200DK/Atlas300 |
| [colorization_video](./python/level2_simple_inference/6_other/colorization_video) | python |20.0/20.1  | Atlas200DK/Atlas300 |

## 样例部署

   请按照各样例下的Readme进行样例部署运行。   

## 文档

参考官网[昇腾文档](https://support.huaweicloud.com/ascend/index.html)获取相关文档。

## 社区

昇腾社区鼓励开发者多交流，共学习。开发者可以通过以下渠道进行交流和学习。

昇腾官网：ascend.huawei.com

昇腾论坛：https://bbs.huaweicloud.com/forum/forum-726-1.html

昇腾官方qq群：965804873

## 贡献

欢迎参与贡献。更多详情，请参阅我们的[贡献者Wiki](./CONTRIBUTING.md)。

## 许可证
[Apache License 2.0](LICENSE)