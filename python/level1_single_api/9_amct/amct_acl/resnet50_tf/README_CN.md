# TensorFlow 框架示例

TensorFlow 框架 ResNet-50 分类网络模型量化

## 1. 准备模型

请至[昇腾社区](https://ascend.huawei.com/zh/#/software/modelzoo/detail/1/7548422b6b9c4a809114435f6b128bb6)下载 ResNet-50 模型文件。下载后解压并将其中的 resnet_v1_50.pb 放到当前目录的 [model](./model/) 子目录中。

## 2. 准备校准数据集

计算量化因子的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来执行模型推理，在推理过程中计算量化因子。请下载[校准图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/classification/calibration.rar)，并解压到 [data](./data/) 文件夹中。

## 3. 执行量化

进入 scripts 子目录，执行 run_calibration.sh 脚本。

```bash
bash run_calibration.sh
```

若出现如下信息则说明量化成功：

```none
amct_acl generate deploy air success.
```

## 4. 量化结果

量化成功后，会在 [scripts](./scripts/) 目录生成如下文件：

+ [amct_log](./scripts/amct_log/)
  + [amct_acl.log](./scripts/amct_log/amct_acl.log): 量化日志文件，记录昇腾模型压缩工具量化过程的日志信息。
+ [fusion_result.json](./scripts/fusion_result.json): 模型编译过程中使用的融合规则。
+ [kernel_meta](./scripts/kernel_meta/): 算子编译生成的文件目录。
+ [outputs](./scripts/outputs)
  + [resnet_v1_50.air](./scripts/outputs/resnet_v1_50.air): 量化后的模型文件。
