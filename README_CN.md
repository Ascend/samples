中文|[English](README.md)

## 昇腾CANN样例仓介绍
   
CANN AscendCL（Ascend Computing Language）提供Device管理、Context管理、Stream管理、内存管理、模型加载与执行、算子加载与执行、媒体数据处理等C语言API库供用户开发深度神经网络应用，用于实现目标识别、图像分类等功能。用户可以通过第三方框架调用AscendCL接口，以便使用昇腾AI处理器的计算能力；用户还可以使用AscendCL封装实现第三方lib库，以便提供昇腾AI处理器的运行管理、资源管理能力。

昇腾样例仓就是以CANN AscendCL接口进行开发，制作的一系列给开发者进行参考学习的样例。在开发者朋友们开发自己的样例时，也可以就样例仓的相关案例进行参考。

## 版本说明

**master分支样例版本适配情况请参见[样例表单及适配说明](#Version-of-samples)。     
历史版本请参考[表 版本说明](#Version-Description)下载对应发行版**。

**表1** 版本说明<a name="Version-Description"></a>
| CANN版本 | 驱动版本 |cann-samples仓是否维护 | cann-samples获取方式 |
|---|---|---|---|
| [3.1.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.7.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.1.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.1.0) |
| [3.2.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.8.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.2.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.2.0) |
| [3.3.0.alpha001/3.3.0.alpha005/<br>3.3.0.alpha006/5.0.2.alpha001/<br>5.0.2.alpha002](https://ascend.huawei.com/#/software/cann/download) | [1.0.9.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.3.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.3.0) |
| [5.0.2.alpha003](https://ascend.huawei.com/#/software/cann/download) | [1.0.10.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.4.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.4.0) |

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
| [DVPP接口样例](./cplusplus/level2_simple_inference/0_data_process) |  c++ | >3.0.0  | A200DK/A300 | 调用dvpp的相关接口，实现图像处理。包含crop/vdec/venc/jpegd/jpege/resize/batchcrop/cropandpaste等功能 |
| [自定义算子样例](./cplusplus/level1_single_api/4_op_dev/2_verify_op) |  c++ | >3.0.0   | A200DK/A300 | 自定义算子运行验证，包含Add算子/batchnorm算子/conv2d算子/lstm算子/matmul算子/reshape算子等运行验证。|
| [200DK外设样例](./cplusplus/level1_single_api/5_200dk_peripheral) |  c++ | >3.0.0  | A200DK| 200DK外设接口相关案例，包含 对GPIO的引脚做配置/使用i2c读写数据/使用uart1串口收发数据/使用摄像头拍摄照片或视频 等功能。|
| [C++通用分类样例](./cplusplus/level2_simple_inference/1_classification) |  c++ | >3.0.0   | A200DK/A300 | 使用googlenet/ResNet-50模型对输入数据进行分类推理。包含 通用图片/通用视频/动态batch/多batch/视频码流/通用摄像头 等多种特性样例。|
| [C++通用检测样例](./cplusplus/level2_simple_inference/2_object_detection) |  c++ | >3.0.0   | A200DK/A300 | 使用人脸检测/yolov3/yolov4/vgg_ssd/faster_rcnn模型对输入数据进行检测。包含 通用图片/通用视频//视频码流/通用摄像头 等多种特性样例。|
| [gemm](./cplusplus/level1_single_api/1_acl/4_blas/gemm) |  c++ | >3.0.0   | A200DK/A300 | 实现矩阵-矩阵乘运算。|
| [WAV_to_word](./cplusplus/level2_simple_inference/5_nlp/WAV_to_word) |  c++ | >3.0.0 | A200DK/A300 | 使用语音转换模型对输入语音进行推理。|
| [colorization](./cplusplus/level2_simple_inference/6_other/colorization) |  c++ | >3.0.0 | A200DK/A300 | 使用colorization模型对输入的黑白图片进行上色推理。| 
| [colorization<br>_video](./cplusplus/level2_simple_inference/6_other/colorization_video) |  c++ | >3.0.0 | A200DK/A300 | 使用黑白图像上色模型对输入的黑白视频进行推理。|
| [DeRain](./cplusplus/level2_simple_inference/6_other/DeRain) |  c++ | >3.0.0 | A200DK/A300 | 通过读取本地雨天退化图像数据，对场景中的雨线、雨雾进行去除，实现图像增强效果。|
| [DeblurGAN_GOPRO<br>_Blur2Sharp](./cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp) |  c++ | >3.0.0 | A200DK/A300 | 输入一张模糊图片，使用DeblurGAN将其变清晰。|
| [YOLOV3_coco_detection<br>_multi_thread_VENC](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/YOLOV3_coco_detection_multi_thread_VENC) |  c++ | >3.0.0 | A200DK/A300 | 使用yolov3模型对输入视频进行分类推理。（多线程处理）|
| [multi_channels<br>_rtsp](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp) |  c++ |>3.0.0  | A200DK/A300| 使用人脸检测模型同时对两路视频流进行人脸检测。|
| [AI_painting](./cplusplus/contrib/AI_painting) |  c++ | >3.0.0  | A200DK/A300 | 使用AI_painting模型根据输入的类目和布局信息生成风景图画。|
| [AscendBot](./cplusplus/contrib/AscendBot) |  c++ | >3.0.0  | A200DK/A300 | 智能小车被手机APK遥控，实现物体跟随、车轨道循线、防跌落功能。|
| [HandWrite](./cplusplus/contrib/HandWrite) |  c++ | >3.0.0  | A200DK| 检测摄像头中的文字，并在presenter界面中给出检测结果。|
| [ar_shadowgan](./cplusplus/contrib/ar_shadowgan) |  c++ | >3.0.0  | A200DK/A300 | AI图片GAN阴影生成样例，使用GAN模型对输入图片进行推理。|
| [cartoonGAN_picture](./cplusplus/contrib/cartoonGAN_picture) |  c++ | >3.0.0  | A200DK/A300 | 卡通图像生成样例，使用cartoonGAN模型对输入图片进行卡通化处理。|
| [human_segmentation](./cplusplus/contrib/human_segmentation) |  c++ | >3.0.0  | A200DK/A300 | 视频人体语义分割样例，使用语义分割模型对输入的视频中人体进行语义分割推理。|
| [super_resolution](./cplusplus/contrib/super_resolution) |  c++ | >3.0.0  | A200DK/A300 | 超分辨率图像处理样例，使用SRCNN、FSRCNN、VDSR和ESPCN四种模型之一对输入图片进行图像超分辨率处理。|
| [python通用分类](./python/level2_simple_inference/1_classification) |  python | >3.0.0 | A200DK/A300 | 使用googlenet/inceptionv3/vgg16模型对输入数据进行分类推理。|
| [python通用检测](./python/level2_simple_inference/1_classification) |  python | >3.0.0 | A200DK/A300 |使用人脸检测/yolov3/yolov4模型对输入数据进行检测。|
| [deeplabv3_pascal<br>_pic](./python/level2_simple_inference/3_segmentation/deeplabv3_pascal_pic) | python | >3.0.0 | A200DK/A300 | 使用deeplabv3+模型对输入图片进行语义分割。|
| [bert_text_classification](./python/level2_simple_inference/5_nlp/bert_text_classification) | python | >3.0.0 | A200DK/A300 | 使用bert模型对文本进行分类。|
| [colorization<br>_picture](./python/level2_simple_inference/6_other/colorization_picture) | python | >3.0.0 | A200DK/A300 | 使用colorization模型对输入的黑白图片进行上色推理。|
| [colorization<br>_video](./python/level2_simple_inference/6_other/colorization_video) | python | >3.0.0 | A200DK/A300 | 使用黑白图像上色模型对输入的黑白视频进行推理。|
| [imageinpainting<br>_hifill](./python/level2_simple_inference/6_other/imageinpainting_hifill) | python | >3.0.0 | A200DK/A300 | 高清图像修复样例，对待修复的jpg图片以及对应的mask图片进行超高分辨率的图像修复，生成修复后的图片。|
| [YOLOV3_mask<br>_detection_e2e](./python/level2_simple_inference/n_e2e/YOLOV3_mask_detection_e2e) | python | >3.0.0 | A200DK/A300 | ModelArts+Ascend310 从modelarts到线下部署，开发口罩识别AI应用（图片输入+图片输出）|
| [garbage_e2e](./python/level2_simple_inference/n_e2e/garbage_e2e) | python | >3.0.0 | A200DK/A300 | MindSpore+Ascend310 从MindSpore到线下部署，开发垃圾分类AI应用（图片输入+图片输出）|
| [vgg16_cat<br>_dog_e2e](./python/level2_simple_inference/n_e2e/vgg16_cat_dog_e2e) | python | >3.0.0 | A200DK/A300 | ModelArts- jupyter +Ascend310 从ModelArts到线下部署，开发猫狗识别AI应用（图片输入+图片输出）|
| [Robotic_Arm_Object<br>_Following](./python/level3_multi_model/Robotic_Arm_Object_Following) | python | >3.0.0 | A200DK | 使用Atlas200DK运行Yolov3模型，对双目深度相机给出的RGB数据流进行推理，实时检测目标在图像中的位置。并结合相机的深度数据流，控制机械臂的姿态，使得机械臂跟随目标移动。|
| [3Dgesture<br>_recognition](./python/contrib/3Dgesture_recognition) | python | >3.0.0 | A200DK/A300 | 使用3DCNN模型对数据进行分类推理。|
| [SentimentAnalysis](./python/contrib/SentimentAnalysis) | python | >3.0.0 | A200DK | 实现了句子级情感极性分类网络的推理功能，输出每个类别的置信度。|
| [YOLOV3_plane<br>_detection](./python/contrib/YOLOV3_plane_detection) | python | >3.0.0 | A200DK | 实现对遥感图像中飞机目标检测的功能。|
| [cartoonGAN<br>_picture](./python/contrib/cartoonGAN_picture) | python | >3.0.0 | A200DK/A300  | 使用cartoonGAN模型对输入图片进行卡通化处理。|
| [crowd_count<br>_picture](./python/contrib/crowd_count_picture) | python | >3.0.0 | A200DK/A300  | 使用count_person.caffe模型对密集人群进行计数。|
| [dehaze<br>_picture](./python/contrib/crowd_count_picture) | python | >3.0.0 | A200DK/A300  | 使用deploy_vel模型对输入图片进行去雾。|
| [edge_detection<br>_picture](./python/contrib/edge_detection_picture) | python | >3.0.0 | A200DK/A300  | 使用RCF模型对输入图片进行边缘检测。|
| [garbage<br>_picture](./python/contrib/garbage_picture) | python | >3.0.0 | A200DK/A300  | 使用mobilenetV2模型对输入图片进行分类推理。|
| [gesture_recognition<br>_picture](./python/contrib/garbage_picture) | python | >3.0.0 | A200DK/A300  | 使用gesture_yuv模型对输入图片进行手势识别。|
| [human_protein_map<br>_classification](./python/contrib/human_protein_map_classification) | python | >3.0.0 | A200DK/A300  | 对蛋白质图像进行自动化分类评估，本案例由上海交通大学提供。|
| [image_HDR<br>_enhance](./python/contrib/image_HDR_enhance) | python | >3.0.0 | A200DK/A300  | 使用模型对曝光不足的输入图片进行HDR效果增强，本案例由深圳大学贡献。|
| [inceptionv2<br>_picture](./python/contrib/inceptionv2_picture) | python | >3.0.0 | A200DK/A300  | 使用InceptionV2模型对输入的踢脚线图片进行分类推理，本样例由尚艺良品贡献。|
| [portrait<br>_picture](./python/contrib/portrait_picture) | python | >3.0.0 | A200DK/A300  | 使用PortraitNet模型对输入图片中人像进行分割，然后与背景图像融合，实现背景替换，本样例为清华大学贡献。|

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