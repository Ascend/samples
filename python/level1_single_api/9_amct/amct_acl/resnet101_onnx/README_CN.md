# ONNX 框架示例

ONNX ResNet-101 分类网络模型量化

## 1. 准备模型

请下载 [ResNet-101](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/resnet101_v11.onnx)。下载后将 resnet101_v11.onnx 放到当前目录的 [model](./model/) 子目录中。

## 2. 准备校准数据集

计算量化因子的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来执行模型推理，在推理过程中计算量化因子。请下载[校准图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/classification/imagenet_calibration.tar.gz)，并解压到 [data](./data/) 文件夹中。

## 3. 执行量化

进入 [scripts](./scripts/) 子目录，执行如下命令：

```none
bash run_calibration.sh
```

若出现如下信息则说明量化成功：

```none
amct_acl generate deploy air success.
```

## 4. 量化结果

量化成功后，会在 [scripts](./scripts/) 目录生成以下文件：

+ [amct_log](./scripts/amct_log/)
  + [amct_acl.log](./scripts/amct_log/amct_acl.log): 量化日志文件，记录昇腾模型压缩工具量化过程的日志信息。
+ [fusion_result.json](./scripts/fusion_result.json): 模型编译过程中使用的融合规则。
+ [kernel_meta](./scripts/kernel_meta/): 算子编译生成的文件目录。
+ [outputs](./scripts/outputs/)
  + [resnet101_v11.air](./scripts/outputs/resnet101_v11.air): 量化后的模型文件。
