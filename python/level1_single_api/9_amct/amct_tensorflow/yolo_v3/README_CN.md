# YOLO V3

## 1. 均匀量化

### 1.1 量化前提

+ **模型准备**  
请至[昇腾社区-ModelZoo](https://ascend.huawei.com/zh/#/software/modelzoo/detail/C/210261e64adc42d2b3d84c447844e4c7)下载 YOLOv3 模型文件。解压并将其中的 single/pb/yolov3_tensorflow_1.5.pb 放到 [model](./data/) 目录下。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolo_v3_calibration/detection.jpg)，并将图片放到 [data](./data/) 目录下。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。  
计算量化参数的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来针对性计算量化参数，使用一个或多个 batch 对量化后的网络模型进行推理即可完成校准。请下载[校准集](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolo_v3_calibration/calibration.jpg)，并将图片放到 [data](./data/) 目录下。

### 1.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [data](./data/)
  + [calibration.jpg](./data/calibration.jpg)
  + [COCO_labels.txt](./data/COCO_labels.txt)
  + [detection.jpg](./data/detection.jpg)
+ [model](./model/)
  + [yolov3_tensorflow_1.5.pb](./model/yolov3_tensorflow_1.5.pb)
+ [src](./src/)
  + [yolo_quant.cfg](./src/yolo_quant.cfg)
  + [yolo_v3_calibration.py](./src/yolo_v3_calibration.py)

在当前目录执行如下命令运行示例程序：

```bash
python ./src/yolo_v3_calibration.py
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/yolo_v3_quantized.pb
origin.png save successfully!
quantize.png save successfully!
```

### 1.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [outputs](./outputs/) 文件夹，该文件夹内包含以下内容：

+ [config.json](./outputs/config.json): 量化配置文件，描述了如何对模型中的每一层进行量化。
+ [record.txt](./outputs/record.txt): 量化因子记录文件，记录量化因子。关于该文件的原型定义请参见[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。
+ [yolo_v3_quant.json](./outputs/yolo_v3_quant.json): 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。
+ [yolo_v3_quantized.pb](./outputs/yolo_v3_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。

> 对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。
