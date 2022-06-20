中文|[English](README.md)

## CANN样例仓介绍
   
CANN AscendCL（Ascend Computing Language）提供Device管理、Context管理、Stream管理、内存管理、模型加载与执行、算子加载与执行、媒体数据处理等C语言API库供用户开发深度神经网络应用，用于实现目标识别、图像分类等功能。用户可以通过第三方框架调用AscendCL接口，以便使用昇腾AI处理器的计算能力；用户还可以使用AscendCL封装实现第三方lib库，以便提供昇腾AI处理器的运行管理、资源管理能力。

昇腾样例仓就是以CANN AscendCL接口进行开发，制作的一系列给开发者进行参考学习的样例。在开发者朋友们开发自己的样例时，也可以就样例仓的相关案例进行参考。

## 版本说明

**请在[硬件平台页面](https://www.hiascend.com/hardware/firmware-drivers?tag=community)选择您使用的产品后，通过下拉框选择支持的CANN版本并查看配套关系。**

- **当前分支样例版本适配说明如下：**    
    | CANN版本 |
    |---|
    | [>=5.1.RC2.alpha006](https://www.hiascend.com/software/cann/community) |

- **历史版本请参考下表使用对应发行版：**      
    | CANN版本 | cann-samples获取方式 |
    |---|---|
    | [5.1.RC2.alpha005/5.1.RC2.alpha003<br>/5.1.RC2.alpha002/5.1.RC2.alpha001<br>/5.1.RC1.alpha006/5.1.RC1.alpha005](https://www.hiascend.com/software/cann/community) | [tag v0.7.0](https://github.com/Ascend/samples/tree/v0.7.0/)，[下载Release 0.7.0发行版](https://github.com/Ascend/samples/releases/v0.7.0) 
    | [5.0.4.alpha002/5.0.4.alpha003/<br>5.0.4.alpha005/5.0.5.alpha001/<br>5.1.RC1.alpha001/5.1.RC1.alpha002/<br>5.1.RC1.alpha003](https://www.hiascend.com/software/cann/community) | [tag v0.6.0](https://github.com/Ascend/samples/tree/v0.6.0/)，[下载Release 0.6.0发行版](https://github.com/Ascend/samples/releases/v0.6.0) |
    | [5.0.2.alpha005/5.0.3.alpha001/<br>5.0.3.alpha002/5.0.3.alpha003/<br>5.0.3.alpha005](https://www.hiascend.com/software/cann/community) | [tag v0.5.0](https://github.com/Ascend/samples/tree/v0.5.0/)，[下载Release 0.5.0发行版](https://github.com/Ascend/samples/releases/v0.5.0) |
    | [5.0.2.alpha003](https://www.hiascend.com/software/cann/community) | [tag v0.4.0](https://github.com/Ascend/samples/tree/v0.4.0/)，[下载Release 0.4.0发行版](https://github.com/Ascend/samples/releases/v0.4.0) |
    | [3.3.0.alpha001/3.3.0.alpha005/<br>3.3.0.alpha006/5.0.2.alpha001/<br>5.0.2.alpha002](https://www.hiascend.com/software/cann/community) | [tag v0.3.0](https://github.com/Ascend/samples/tree/v0.3.0/)，[下载Release 0.3.0发行版](https://github.com/Ascend/samples/releases/v0.3.0) |
    | [3.2.0.alpha001](https://www.hiascend.com/software/cann/community) | [tag v0.2.0](https://github.com/Ascend/samples/tree/v0.2.0/)，[下载Release 0.2.0发行版](https://github.com/Ascend/samples/releases/v0.2.0) |
    | [3.1.0.alpha001](https://www.hiascend.com/software/cann/community) | [tag v0.1.0](https://github.com/Ascend/samples/tree/v0.1.0/)，[下载Release 0.1.0发行版](https://github.com/Ascend/samples/releases/v0.1.0) |

- **历史版本操作说明**      
  **tag**：对某一时间点的代码仓打标签，在发布某个软件版本（比如 v0.1.0 等等）时使用tag标签，给仓库中的项目添加tag。可以理解为某一时刻的不会变化的分支。   
  **Realease**：基于tag，为tag添加更丰富的信息，一般是编译好的文件。     
  1. 用户可以在仓库的分支切换框中选择对应的标签（tag）从而查看对应版本的代码及readme。    
  2. 用户可以下载realease提供的编译好的文件（Source code）进行代码使用。    
  3. 如果需要在命令行访问tag代码，可以按照如下方式操作。
     ```
     # 命令行下载master代码
     git clone https://github.com/Ascend/samples.git   
     # 切换到历史tag，以v0.1.0举例
     git checkout v0.1.0
     ```

## 目录结构与说明
| 目录 | 说明 |
|---|---|
| [common](./common) | samples仓公共文件目录 |
| [cplusplus](./cplusplus) | samples仓C++样例目录 |
| [python](./python) | samples仓python样例目录 |
| [st](./st) | samples仓样例测试用例目录 | 

## 安装
**根据设备形态按按照如下步骤搭建环境：**    
   - Atlas200DK:     
     (1) 在[硬件产品文档](https://www.hiascend.com/document?tag=hardware)中选择**AI开发者套件**文档，点击**环境部署**进入文档。    
     (2) 根据**安装部署流程**章节了解整体流程并根据文档进行硬件及CANN软件安装。    
   
   - Atlas300（ai1s）:    
    (1) 在[开发者文档（社区版）](https://www.hiascend.com/document?tag=community-developer)中选择**环境部署**文档 ，点击**CANN软件安装**进入文档。     
    (2) 选择**安装驱动和固件**章节，按照要求进行固件与驱动的安装。    
    (3) 选择**安装开发环境**章节，按找要求根据需要安装依赖及CANN软件包。   

## 运行  
**根据以下表单，选择需要运行的样例，并按照readme进行第三方依赖的安装及样例下载运行**      
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