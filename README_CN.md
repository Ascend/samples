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
| [5.0.2.alpha005/5.0.3.alpha001/<br>5.0.3.alpha002/5.0.3.alpha003/<br>5.0.3.alpha005](https://ascend.huawei.com/#/software/cann/download) | [1.0.11.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.5.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.5.0) |
| [5.0.2.alpha003](https://ascend.huawei.com/#/software/cann/download) | [1.0.10.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.4.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.4.0) |
| [3.3.0.alpha001/3.3.0.alpha005/<br>3.3.0.alpha006/5.0.2.alpha001/<br>5.0.2.alpha002](https://ascend.huawei.com/#/software/cann/download) | [1.0.9.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.3.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.3.0) |
| [3.2.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.8.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.2.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.2.0) |
| [3.1.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.7.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | 是 | Release 0.1.0发行版，[点击跳转](https://github.com/Ascend/samples/releases/v0.1.0) |

## 目录结构与说明
| 目录 | 说明 |
|---|---|
| [common](./common) | samples仓公共文件目录 |
| [cplusplus](./cplusplus) | samples仓C++样例目录 |
| [python](./python) | samples仓python样例目录 |
| [st](./st) | samples仓样例测试用例目录 | 



## 使用指南

1.根据设备形态按按照如下步骤搭建合设环境

#### 200DK:

在Atlas 200 DK上搭建开发运行环境

https://support.huaweicloud.com/environment-deployment-Atlas200DK1012/atlased_04_0017.html

#### Ai1S 300:

(1)安装驱动和固件

https://support.huaweicloud.com/instg-cli-cann504-alpha002/atlasdeploy_03_0015.html

（2）安装开发套件包：
    
    安装依赖：https://support.huaweicloud.com/instg-cli-cann504-alpha002/atlasdeploy_03_0021.html
    
    安装套件包：https://support.huaweicloud.com/instg-cli-cann504-alpha002/atlasdeploy_03_0031.html


2.安装第三方依赖

（1）c++：https://github.com/Ascend/samples/tree/master/cplusplus/environment

（2）python：https://github.com/Ascend/samples/tree/master/python/environment

3.下载samples仓，运行样例

  git  clone https://github.com/Ascend/samples


## 样例表单&适配说明<a name="Version-of-samples"></a>

| 样例名称 | 语言 | 适配CANN版本 | 简介 |
|---|---|---|---|
| [DVPP接口样例](./cplusplus/level2_simple_inference/0_data_process) |  c++ | >=5.0.4 | 调用dvpp的相关接口，实现图像处理。包含crop/vdec/venc/jpegd/jpege/resize/batchcrop/cropandpaste等功能。 |
| [自定义算子样例](./cplusplus/level1_single_api/4_op_dev/2_verify_op) |  c++ | >=5.0.4 | 自定义算子运行验证，包含Add算子/batchnorm算子/conv2d算子/lstm算子/matmul算子/reshape算子等运行验证。 |
| [200DK外设样例](./cplusplus/level1_single_api/5_200dk_peripheral) |  c++ | >=5.0.4 | 200DK外设接口相关案例，包含 对GPIO的引脚做配置/使用i2c读写数据/使用uart1串口收发数据/使用摄像头拍摄照片或视频 等功能。 |
| [C++分类样例](./cplusplus/level2_simple_inference/1_classification) |  c++ | >=5.0.4 | 使用googlenet/ResNet-50模型对输入数据进行分类推理。包含 图片/视频/动态batch/多batch/视频码流/通用摄像头 等多种特性样例。 |
| [C++检测样例](./cplusplus/level2_simple_inference/2_object_detection) |  c++ | >=5.0.4 | 使用人脸检测/yolov3/yolov4/vgg_ssd/faster_rcnn模型对输入数据进行检测。包含 通用图片/通用视频//视频码流/通用摄像头 等多种特性样例。 |
| [C++自然语言处理样例](./cplusplus/level2_simple_inference/5_nlp) |  c++ | >=5.0.4 | 使用nlp模型对输入数据进行推理。 |
| [C++其它样例](./cplusplus/level2_simple_inference/6_other) |  c++ | >=5.0.4 | 其他模型推理样例，包含黑白图像上色，超分，图像增强等 | 
| [C++多线程样例](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread) |  c++ | >=5.0.4 | 使用yolov3，人脸检测等模型对输入数据进行多线程推理的样例。 |
| [C++用户贡献样例](./cplusplus/contrib) |  c++ | >=5.0.4 | 用户贡献的推理样例。|
| [python分类样例](./python/level2_simple_inference/1_classification) |  python | >=5.0.4 | 使用googlenet/inceptionv3/vgg16模型对输入数据进行分类推理。 |
| [python检测样例](./python/level2_simple_inference/1_classification) |  python | >=5.0.4 | 使用人脸检测/yolov3/yolov4模型对输入数据进行检测。 |
| [python分割样例](./python/level2_simple_inference/3_segmentation) | python | >=5.0.4 | 使用分割模型对输入图片进行分割。 |
| [python自然语言处理样例](./python/level2_simple_inference/5_nlp) | python | >=5.0.4 | 使用nlp模型对输入数据进行推理。 |
| [python其它样例](./python/level2_simple_inference/6_other) | python | >=5.0.4 | 其他模型推理样例，包含黑白图像上色，图像修复等。 |
| [从训练到推理端到端样例](./python/level2_simple_inference/n_e2e) | python | >=5.0.4 | 从训练到部署端到端样例指导，包含口罩识别，垃圾分类，猫狗大战等指导。 |
| [python行业样例](./python/level3_multi_model) | python | >=5.0.4 | 较为复杂的样例，结合硬件或使用多模型多线程样例。如去除图像的指定前景目标样例，机械臂样例等。 |
| [python用户贡献样例](./python/contrib) | python | >=5.0.4 | 用户贡献的推理样例。 |

## 环境搭建&样例部署
samples仓所有样例依赖异构计算架构CANN，如果还未安装，请参考[CANN软件安装 (开发&运行场景, 通过命令行方式)](https://www.hiascend.com/document?tag=community-developer)选择你使用的版本完成CANN安装。

然后按照各目录下的Readme进行样例选择，并根据所选样例下的Readme进行操作。   

## 文档

参考社区网站[昇腾文档](https://support.huaweicloud.com/ascend/index.html)获取相关文档。

## 社区

昇腾社区鼓励开发者多交流，共学习。开发者可以通过以下渠道进行交流和学习。

昇腾社区网站：hiascend.com

昇腾论坛：https://bbs.huaweicloud.com/forum/forum-726-1.html

昇腾官方qq群：965804873

## 贡献

欢迎参与贡献。更多详情，请参阅我们的[贡献者Wiki](./CONTRIBUTING_CN.md)。

## 许可证
[Apache License 2.0](LICENSE)