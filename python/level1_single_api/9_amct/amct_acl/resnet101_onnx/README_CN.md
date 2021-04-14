# 量化示例

本文档为amct_acl工具量化ONNX ResNet101模型的使用示例。

## 环境要求

用户需要先搭建好Ascend910环境，包括安装Driver/Firmware/FwkACLlib等软件包和设置环境变量，然后在该环境上安装晟腾模型压缩工具。环境安装方法请参考《晟腾模型压缩工具使用指南（ACL方式）》。

## 模型准备

请下载[ResNet101模型文件](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/resnet101_v11.onnx)。下载后将resnet101_v11.onnx放到当前目录的model子目录中。处理后的model目录结构如下：

```shell
└─model
    └─resnet101_v11.onnx
```

## 数据准备

+ **校准集准备**

计算量化因子的过程被称为“校准（calibration）”。校准过程需要使用一部分测试图片来执行模型推理，在推理过程中计算量化因子。请下载[校准图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/classification/imagenet_calibration.tar.gz)，并解压到data文件夹中。处理后的data目录结构如下：

```shell
└─data
    ├─iamges
    |    ├─absorbent-3509508__340.jpg
    |    ├─......(省略其余图片名字)
    |    └─image_label.txt
    └─process_data.py
```

## 量化示例

进入scripts子目录，执行run_calibration.sh脚本。

```shell
bash run_calibration.sh
```

若出现如下信息则说明量化成功：

```shell
amct_acl generate deploy air success.
```

## 量化结果

量化成功后，会在scripts目录生成以下文件：

+ amct_log/amct_acl.log：量化日志文件，记录晟腾模型压缩工具量化过程的日志信息。
+ results/resnet101_v11.air：量化后的模型文件。
+ kernel_meta：算子编译生成的文件目录。
+ （可选）dump/record.txt：量化因子文件，如果执行量化时设置了生成量化因子的环境变量，则量化后会生成该目录。关于环境变量的设置和量化因子的详细说明，请参考《晟腾模型压缩工具使用指南（ACL方式）》。
