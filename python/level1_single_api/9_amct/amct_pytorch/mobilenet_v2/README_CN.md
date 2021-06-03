# MobileNet V2

## 1. 基于精度的自动量化

该接口会根据模型在数据集测试的精度和用户设定的精度损失要求来自动决定每一层是否进行量化，最终生成量化的混合模型；精度目标如果设置的比较苛刻，所有层均不量化才能达到目标则不会生成量化模型，可以适当降低精度损失的目标，继续调用该接口进行模型的量化。

### 1.1 量化前提

- **安装依赖**

  本 sample 依赖 numpy,  opencv-python 和 torchvison 包：

  安装命令：

  ```
  pip3 install numpy
  pip3 install opencv-python
  pip3 install torchvision==0.6.0
  ```
  torchvison 0.6.0 是 torch 1.5.0 的配套版本，如果使用其他版本的torch，则安装配套的 torchvison；
  如果需要使用 GPU 版本的 torch, 则安装和 CUDA 环境对应的 torch, torchvision 版本即可，详细安装指导可以参考 torch 官网；

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-101_nuq/images.zip)，解压后将 “images” 文件夹放到 [data](./data/) 目录下。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。本 sample 校准集与数据集相同。

### 1.2 量化示例

请在当前目录执行如下命令运行示例程序：

```bash
python ./src/mobilenet_v2_accuracy_based_auto_calibration.py
```

### 1.3 量化结果

量化成功后会生成量化 fake quant 和 deploy 的 onnx 模型文件和量化层敏感度的信息文件, 量化因子记录文件等；

文件的相关说明如下：

| 目录     | 文件或目录                                        | 说明                                                         |
| -------- | ------------------------------------------------- | ------------------------------------------------------------ |
| amct_log | amct_pytorch.log                                  | amct_pytorch 工具在执行过程中相关日志文件                    |
|          | accuracy_based_auto_calibration_record.json       | 基于精度的自动量化回退过程中的量化配置记录文件               |
| results  | mobilenet_v2_fake_quant_model.onnx                | 生成的 fake quant onnx 模型文件                              |
|          | mobilenet_v2_deploy_model.onnx                    | 生成的 deploy onnx 模型文件                                  |
|          | accuracy_based_auto_calibration_final_config.json | 基于精度的自动量化回退最终搜索得到的量化配置文件             |
|          | accuracy_based_auto_calibration_ranking_info.json | 基于精度的自动量化回退过程中记录的每层量化敏感度信息文件     |
|          | tempxxxxx/ 目录                                   | 里面保存了隐藏层的 feature map 和其他临时性模型文件，当执行完毕后就可以删除 |
| tmp      | config.json                                       | 量化过程中的量化配置文件                                     |
|          | scale_offset_record.txt                           | 量化因子记录文件                                             |
