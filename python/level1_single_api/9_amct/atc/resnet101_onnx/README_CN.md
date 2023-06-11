# ONNX 框架示例

ONNX ResNet-101 分类网络模型量化

## 1. 准备模型

请下载 [ResNet-101](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/amct_acl/resnet101_v11.onnx)。下载后将 resnet101_v11.onnx 放到当前目录的 [model](./model/) 子目录中。

## 2. 准备校准数据集

计算量化因子的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来执行模型推理，在推理过程中计算量化因子。请下载[校准图片](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/amct_acl/classification/imagenet_calibration.tar.gz)，解压后将 “images” 文件夹放到 [data](./data/) 目录下。

## 3. 执行量化

执行如下命令进行量化：

```bash
bash ./scripts/run_calibration.sh
```

若出现如下信息则说明量化成功：

```none
ATC run success
```

## 4. 量化结果

量化成功后，会在当前目录生成以下文件：

+ [amct_log_时间戳](./amct_log_时间戳/)
  + [amct_acl.log](./amct_log_时间戳/amct_acl.log): 量化日志文件，记录昇腾模型压缩工具量化过程的日志信息。
+ [fusion_result.json](./fusion_result.json): 模型编译过程中使用的融合规则。
+ [outputs](./outputs/)
  + [resnet101_v11.om](./outputs/resnet101_v11.om): 量化后的模型文件。

## 5. 芯片类型
scripts/run_calibration.sh中atc命令里的soc_version参数为量化后模型的芯片类型。
config/compression_opt.config中的soc_version参数为推理使用的芯片类型。
