中文|[English](README_EN.md)

## 昇腾CANN样例仓介绍
   
CANN AscendCL（Ascend Computing Language）提供Device管理、Context管理、Stream管理、内存管理、模型加载与执行、算子加载与执行、媒体数据处理等C语言API库供用户开发深度神经网络应用，用于实现目标识别、图像分类等功能。用户可以通过第三方框架调用AscendCL接口，以便使用昇腾AI处理器的计算能力；用户还可以使用AscendCL封装实现第三方lib库，以便提供昇腾AI处理器的运行管理、资源管理能力。

昇腾样例仓就是以CANN AscendCL接口进行开发，制作的一系列给开发者进行参考学习的样例。在开发者朋友们开发自己的样例时，也可以就样例仓的相关案例进行参考。

## 版本说明

**master分支样例版本适配情况请参见[样例表单及适配说明](#Version-of-samples),如果需要使用的案例不兼容您所需要的版本，请参考[表 版本说明](#Version-Description)下载对应发行版**

**表1** 版本说明<a name="Version-Description"></a>
| CANN版本 | cann-samples仓是否维护 | cann-samples获取方式 |
|---|---|---|
| [20.0-20.1](https://ascend.huawei.com/#/software/cann/download) | 是 | Release 0.1.0发行版，[点击跳转](https://gitee.com/ascend/samples/releases/v0.1.0) |
| [20.2](https://ascend.huawei.com/#/software/cann/download) | 是 | Release 0.2.0发行版，[点击跳转](https://gitee.com/ascend/samples/releases/v0.2.0) |

## 目录结构与说明

**./**   
├── [c++](./cplusplus)：**C++样例。**    
│   ├── common   
│   ├── contrib   
│   ├── level1_single_api   
│   ├── level2_simple_inference   
│   └── level3_multi_model      
├── [python](./python)：**python样例。**  
│   ├── common   
│   ├── contrib   
│   ├── level1_single_api   
│   ├── level2_simple_inference   
└   └── level3_multi_model   

## 样例表单及适配说明<a name="Version-of-samples"></a>

| 样例名称 | 语言 | 适配CANN版本 | 适配产品 | 简介 |
|---|---|---|---|---|
| [crop](./cplusplus/level2_simple_inference/0_data_process/crop) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的crop接口，实现图像裁剪功能。|
| [vdec](./cplusplus/level2_simple_inference/0_data_process/vdec) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的vdec接口，实现图片解码的功能。|
| [venc](./cplusplus/level2_simple_inference/0_data_process/venc) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的venc接口，实现视频编码功能。|
| [jpegd](./cplusplus/level2_simple_inference/0_data_process/jpegd) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的jpegd接口，实现图片解码的功能。|
| [jpege](./cplusplus/level2_simple_inference/0_data_process/jpege) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的jpege接口，实现图片编码的功能。|
| [resize](./cplusplus/level2_simple_inference/0_data_process/resize) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的resize接口，实现图像缩放功能。|
| [batchcrop](./cplusplus/level2_simple_inference/0_data_process/batchcrop) |  c++ |20.0/20.1  | A200DK/A300 | 从一张YUV图片中按指定区域抠出八张224*224子图。|
| [venc<br>_image](./cplusplus/level2_simple_inference/0_data_process/venc_image) |  c++ |20.0/20.1  | A200DK/A300 |将一张YUV图片连续编码生成H265格式的视频码流。|
| [vdecandvenc](./cplusplus/level2_simple_inference/0_data_process/vdecandvenc) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的venc和vdec接口，实现视频编码功能。|
| [cropandpaste](./cplusplus/level2_simple_inference/0_data_process/cropandpaste) |  c++ |20.0/20.1  | A200DK/A300 | 调用dvpp的cropandpaste接口，将图片指定位置大小图片粘贴到输出图片指定位置。|
| [smallResolution<br>_cropandpaste](./cplusplus/level2_simple_inference/0_data_process/smallResolution_cropandpaste) | c++ | 20.0/20.1 | A200DK/A300 | 对指定输入图片进行抠图（包括抠图区域小于10*6），再贴图到输出图片中。|
| [gemm](./cplusplus/level1_single_api/1_acl/4_blas/gemm) |  c++ |20.0/20.1  | A200DK/A300 | 实现矩阵-矩阵乘运算。|
| [acl_execute<br>_add](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_add) |  c++ |20.0/20.1  | A200DK/A300 | 自定义Add算子运行验证。|
| [acl_execute<br>_batchnorm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_batchnorm) |  c++ |20.0/20.1  | A200DK/A300 | 自定义batchnorm算子运行验证。|
| [acl_execute<br>_conv2d](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_conv2d) |  c++ |20.0/20.1  | A200DK/A300 | 自定义conv2d算子运行验证。|
| [acl_execute<br>_lstm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_lstm) |  c++ |20.0/20.1  | A200DK/A300 | 自定义lstm算子运行验证。|
| [acl_execute<br>_matmul](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_matmul) |  c++ |20.0/20.1  | A200DK/A300 | 自定义matmul算子运行验证。|
| [acl_execute<br>_reshape](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_reshape) |  c++ |20.0/20.1  | A200DK/A300 | 自定义reshape算子运行验证。|
| [gpio](./cplusplus/level1_single_api/5_200dk_peripheral/gpio) |  c++ |20.0/20.1  | A200DK| 对GPIO的引脚做配置。|
| [i2c](./cplusplus/level1_single_api/5_200dk_peripheral/i2c) |  c++ |20.0/20.1  | A200DK| 使用i2c读写数据。|
| [uart](./cplusplus/level1_single_api/5_200dk_peripheral/uart) |  c++ |20.0/20.1  | A200DK| 使用uart1串口收发数据。|
| [ascendcamera](./cplusplus/level1_single_api/5_200dk_peripheral/ascendcamera) |  c++ |20.0/20.1  | A200DK| 使用摄像头拍摄照片或视频。|
| [googlenet_imagenet<br>_video](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_video) |  c++ |20.0/20.1  | A200DK/A300 | 使用googlenet模型对输入视频进行分类推理。|
| [googlenet_imagenet<br>_picture](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture) | c++ | 20.0/20.1  | A200DK/A300 | 使用googlenet模型对输入图片进行分类推理。|
| [vdec_resnet50<br>_classification](./cplusplus/level2_simple_inference/1_classification/vdec_resnet50_classification) | c++ | 20.0/20.1  | A200DK/A300 | 基于Caffe ResNet-50网络实现对h265视频码流图片的分类推理。|
| [resnet50_imagenet<br>_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | 基于Caffe ResNet-50网络实现图片分类的功能。|
| [resnet50_async_imagenet<br>_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_async_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | 基于Caffe ResNet-50网络（单输入、单Batch）实现多图异步分类推理。|
| [googlenet_imagenet<br>_dynamic_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_dynamic_batch) | c++ | 20.0/20.1  | A200DK/A300 | 使用googlenet模型对输入图片进行分类推理，本案例采用了动态batch特性。|
| [googlenet_imagenet<br>_multi_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) | c++ | 20.0/20.1  | A200DK/A300 | 使用googlenet模型对输入图片进行分类推理，本案例采用了多batch特性。|
| [vpc_resnet50_imagenet<br>_classification](./cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | 基于Caffe ResNet-50网络实现图片分类（图片解码+缩放+同步推理）|
| [vpc_jpeg_resnet50<br>_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | 基于Caffe ResNet-50网络实现图片分类（图片解码+抠图缩放+图片编码+同步推理）|
| [face_detection<br>_camera](./cplusplus/level2_simple_inference/2_object_detection/face_detection_camera) |  c++ |20.0/20.1  | A200DK| 使用人脸检测模型对树莓摄像头中的即时视频进行人脸检测。|
| [YOLOV3_coco<br>_detection_video](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video) |  c++ |20.0/20.1  | A200DK/A300 | 检测视频中出现的物体，并在视频中给出预测结果。|
| [YOLOV3_coco<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | 使用yolov3模型对输入图片进行预测推理，并将结果打印到输出图片上。|
| [YOLOV4_coco<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | 使用yolov4模型对输入图片进行预测推理，并将结果打印到输出图片上。|
| [YOLOV3_VOC<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_VOC_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | 使用yolov3模型对输入图片进行预测推理，并将结果打印到输出图片上。（针对tensorflow模型，后处理由代码完成）|
| [YOLOV3_coco<br>_detection_VENC](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_VENC) |  c++ |20.0/20.1  | A200DK/A300 | 检测视频中出现的物体，并在视频中给出预测结果。|
| [VGG_SSD_coco_detection<br>_CV_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_with_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | 使用vgg_ssd模型对输入图片进行预测推理，并将结果打印到输出图片上。（使用opencv和aipp处理）|
| [VGG_SSD_coco_detection<br>_CV_without_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_without_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | 使用vgg_ssd模型对输入图片进行预测推理，并将结果打印到输出图片上。（使用opencv处理）|
| [VGG_SSD_coco_detection<br>_DVPP_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | 使用vgg_ssd模型对输入图片进行预测推理，并将结果打印到输出图片上。（使用dvpp和aipp处理）|
| [YOLOV3_coco_detection<br>_dynamic_AIPP](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_dynamic_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | 使用yolov3模型对输入图片进行预测推理，并将结果打印到输出图片上。（使用了动态aipp特性）|
| [YOLOV3_dynamic_batch<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | 基于Caffe YOLOv3网络实现目标检测。（动态Batch/动态分辨率）|
| [faste_RCNN_VOC_detection<br>_dynamic_resolution](./cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution) |  c++ |20.0/20.1  | A200DK/A300 | 使用faster_rcnn模型对输入图片进行预测推理，并将结果打印到输出图片上。|
| [WAV_to_word](./cplusplus/level2_simple_inference/5_nlp/WAV_to_word) |  c++ |20.0/20.1  | A200DK/A300 | 使用语音转换模型对输入语音进行推理。|
| [colorization](./cplusplus/level2_simple_inference/6_other/colorization) |  c++ |20.0/20.1  | A200DK/A300 | 使用colorization模型对输入的黑白图片进行上色推理。| 
| [colorization<br>_video](./cplusplus/level2_simple_inference/6_other/colorization_video) |  c++ |20.0/20.1  | A200DK/A300 | 使用黑白图像上色模型对输入的黑白视频进行推理。|
| [YOLOV3_coco_detection<br>_multi_thread_VENC](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/YOLOV3_coco_detection_multi_thread_VENC) |  c++ |20.0/20.1  | A200DK/A300 | 使用yolov3模型对输入视频进行分类推理。（多线程处理）|
| [face_recognition<br>_camera](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera) |  c++ |20.1  | A200DK| 通过摄像头对视频中的人脸信息进行预测，与已注册的人脸进行比对，预测出最可能的用户。|
| [googlenet_imagenet<br>_multi_batch](./python/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) |  python |20.0/20.1  | A200DK/A300 | 使用googlenet模型对输入图片进行分类推理。（多batch）|
| [googlenet_imagenet<br>_picture](./python/level2_simple_inference/1_classification/googlenet_imagenet_picture) | python |20.0/20.1  | A200DK/A300 | 使用googlenet模型对输入图片进行分类推理。|
| [YOLOV3_coco_detection<br>_picture](./python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) | python |20.0/20.1  | A200DK/A300 | 使用yolov3模型对输入图片进行预测推理，并将结果打印到输出图片上。|
| [YOLOV3_mask_detection<br>_picture](./python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_picture) | python |20.0/20.1  | A200DK/A300 | 实现了对图片中的口罩、人脸、人信息进行预测的功能。|
| [YOLOV3_mask_detection<br>_video](./python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_video) | python |20.0/20.1  | A200DK| 实现了对视频中的口罩、人脸、人信息进行预测的功能。|
| [face_detection<br>_camera](./python/level2_simple_inference/2_object_detection/face_detection_camera) | python |20.0/20.1  | A200DK| 使用人脸检测模型对树莓摄像头中的即时视频进行人脸检测。|
| [colorization<br>_picture](./python/level2_simple_inference/6_other/colorization_picture) | python |20.0/20.1  | A200DK/A300 | 使用colorization模型对输入的黑白图片进行上色推理。|
| [colorization<br>_video](./python/level2_simple_inference/6_other/colorization_video) | python |20.0/20.1  | A200DK/A300 | 使用黑白图像上色模型对输入的黑白视频进行推理。|

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

欢迎参与贡献。更多详情，请参阅我们的[贡献者Wiki](./CONTRIBUTING_CN.md)。

## 许可证
[Apache License 2.0](LICENSE)