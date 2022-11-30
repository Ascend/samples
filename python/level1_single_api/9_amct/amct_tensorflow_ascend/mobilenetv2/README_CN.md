# 训练后量化-mobilenetv2样例
**功能**：使用模型压缩工具**amct_tensorflow_ascend**对Tensorflow的mobilenetv2进行**训练后量化**   
**使用场景**：基于Tensorflow框架，在昇腾AI处理器上做在线推理时，调用模型压缩工具，完成压缩。

#### 前提条件
##### 环境准备
请按照手册准备好环境并安装好amct_tensorflow_ascend工具包。
##### 模型准备
请至
[Tensorflow-models](https://storage.googleapis.com/mobilenet_v2/checkpoints/mobilenet_v2_1.0_224.tgz)
下载 MobileNetV2 模型文件。解压并将其中的 mobilenet_v2_1.0_224_frozen.pb 文件放入[pre_model](./pre_model)文件夹中。
##### 数据集准备
可以对量化前后的模型进行推理，以测试量化对精度的影响，推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/mobilenet_v2_calibration/classification.jpg)，并将该图片放到 [data](./data/) 目录下。
##### 校准集准备
计算量化因子（scale, offset, shift_bit）的过程被称为“校准(calibration)”，使用一个或多个 batch 对量化后的网络模型进行推理即可完成校准，校准过程使用的数据为校准集。校准集要与模型匹配才能保证精度，一般选用测试集的一部分。请下载
[校准集](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/mobilenet_v2_calibration/calibration.rar)，解压后将 “calibration” 文件夹放到 [data](./data/) 目录下。

#### 目录结构与说明
执行量化示例前，请先检查当前目录下是否包含以下文件及目录：  
**./**   
├─data：mobilenetv2的推理时所用的数据集，可选用mobilenetv2的测试集。    
├─pre_model： 存放mobilenetv2量化前的模型  
&emsp;├─mobilenet_v2_1.0_224_frozen.pb  
├─src：处理脚本   
&emsp;├─mobilenetv2_calibration.py：执行量化的python脚本。   
├─README_zh：说明文档
└─requirements.txt: 需要安装的python库

#### 量化步骤
在当前目录执行如下命令运行示例程序：
```python
python3.7.5 src/mobilenetv2_calibration.py [-h] [--cfg_define CFG_DEFINE]
```
参数解释说明   
--h： 可选。显示帮助信息   
--cfg_define CFG_DEFINE：可选，简易配置文件路径

若出现如下信息则说明模型量化成功
```bash
INFO - [AMCT]:[save_model]: The model is saved in $HOME/amct_tensorflow_ascend/mobilenetv2/result/MobileNetV2_ascend_quantized.pb
Origin Model Prediction:
        category index: 443
        category prob: 0.384
Quantized Model Prediction:
        category index: 443
        category prob: 0.477
```
用户如需通过仿真的方式快速检验量化对自己模型精度的影响，可以比较量化前后的模型在其测试集上的表现。本示例展示了使用量化前后的模型对同一张图片做在线推理预测的结果，如上信息（预测结果根据用户环境会有所不同，请以实际显示的为准）。量化前后预测结果十分接近，说明在仿真情况下，量化对该图片的预测影响很小。

#### 量化结果说明
量化成功后，在量化后模型的同级目录下还会生成如下文件：   
**./**   
├─amct_log：   
&emsp;├─amct_tensorflow.log：量化日志文件，记录了量化过程的日志信息。  
├─calibration_tmp：  
&emsp;├─config.json: 量化配置文件，描述了如何对模型中的每一层进行量化。   
&emsp;├─record.txt: 量化因子记录文件。     
├─result：   
&emsp;├─MobileNetV2_quantized.pb： 量化后的模型，可在TensorFlow环境进行精度仿真,也可在昇腾AI处理器部署。   
&emsp;├─MobileNetV2_quant.json： 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型比对使用.   
└─tmp：    
&emsp;├─check_result.tf.json：调用TF_Adapter在NPU上在线推理时产生的文件，记录推理图中不支持在NPU上运行的算子。  
&emsp;├─fusion_result.json：调用TF_Adapter在NPU上在线推理时产生的文件，记录使用的图融合、UB融合规则。   

重新执行该脚本，对模型重新进行量化时，上述结果文件均会被覆盖。

#### 后续处理
如果用户需要将量化后的模型，转换为适配昇腾AI处理器的离线模型，则请参见《CANN 开发辅助工具指南》中的“ATC工具使用指南”章节。