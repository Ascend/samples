- [1. 模型端到端推理指导](#1-模型端到端推理指导)
  - [1.1. 功能说明](#11-功能说明)
  - [1.2. 环境说明](#12-环境说明)
  - [1.3. 模型转换](#13-模型转换)
    - [1.3.1. pth转onnx模型](#131-pth转onnx模型)
    - [1.3.2. onnx转om模型](#132-onnx转om模型)
  - [1.4. 数据预处理](#14-数据预处理)
    - [1.4.1. 数据集获取](#141-数据集获取)
    - [1.4.2. 数据集预处理](#142-数据集预处理)
  - [1.5. 样例运行](#15-样例运行)
    - [1.5.1. 样例](#151-样例)
    - [1.5.2. 结果](#152-结果)
    - [1.5.3. 结果说明](#153-结果说明)
    - [1.5.1. 离线推理精度](#151-离线推理精度)

# 1. 模型端到端推理指导

## 1.1. 功能说明

功能：使用CNN模型对输入音频进行预测推理，并将结果打印出来。
样例输入：原始音频wav文件。
样例输出：输出推理结果标签。

## 1.2. 环境说明

```
CANN 6.0.RC1
tf2onnx == 1.12.1
numpy == 1.19.5
onnxruntime == 1.10.0
keras == 2.6.0
onnxmltools == 1.11.1
```

 **说明：**

- 需安装ffmpeg、openSMLIE工具。
- 模型转换时需要用到[MagicONNX工具](https://gitee.com/Ronnie_zheng/MagicONNX)，安装在inference目录下。

## 1.3. 模型转换

### 1.3.1. pth转onnx模型

1. 得到权重文件：

```
# 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。
cd ${HOME}/samples/best_practices/contrib/depression_detection/inference/model
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_2.h5
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_3.h5
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_23.h5
```

2. 执行h52onnx.py脚本，生成onnx模型文件：

```
python3 h52onnx.py –h5_path ${h5_path} –onnx_path ${onnx_path}
```

或者直接获取onnx模型，见1.3.2。

### 1.3.2. onnx转om模型

1. 设置环境变量

```
source /usr/local/Ascend/ascend-toolkit/set_env.sh
```

该命令中使用CANN默认安装路径(/usr/local/Ascend/ascend-toolkit)中的环境变量，使用过程中请按照实际安装路径设置环境变量。

2. 也可直接获取onnx文件：

```
cd ${HOME}/samples/best_practices/contrib/depression_detection/inference/model
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_1.onnx
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_2.onnx
wget https://sharedata.obs.cn-north-4.myhuaweicloud.com/depression_detection/model_3.onnx
```

3. 使用atc将onnx模型转换为om模型文件，工具使用方法可以参考[CANN V100R020C10 开发辅助工具指南 (推理) 01](https://support.huawei.com/enterprise/zh/doc/EDOC1100164868?idPath=23710424%7C251366513%7C22892968%7C251168373)

   ${chip_name}可通过`npu-smi info`指令查看，例：310P3

![Image](https://sharedata.obs.myhuaweicloud.com/depression_detection/310P3.png)

三个om模型，对应的生成命令为：

```
atc --framework=5 --model=${HOME}/samples/best_practices/contrib/depression_detection/inference/model/model_1.onnx --output= ${HOME}/samples/best_practices/contrib/depression_detection/inference/model/tf_model_1 --input_format=NCHW --input_shape="conv2d_4_input:1,1,20,1" --log=error --soc_version=Ascend${chip_name}

atc --framework=5 --model=${HOME}/samples/best_practices/contrib/depression_detection/inference/model/model_2.onnx --output= ${HOME}/samples/best_practices/contrib/depression_detection/inference/model/tf_model_2 --input_format=NCHW --input_shape="conv2d_8_input:1,1,20,1" --log=error --soc_version=Ascend${chip_name}

atc --framework=5 --model=${HOME}/samples/best_practices/contrib/depression_detection/inference/model/model_3.onnx --output= ${HOME}/samples/best_practices/contrib/depression_detection/inference/model/tf_model_3 --input_format=NCHW --input_shape="conv2d_88_input:1,1,20,1" --log=error --soc_version=Ascend${chip_name}
```

## 1.4. 数据预处理


### 1.4.1. 数据集获取

> 该模型使用MODMA数据集（app重新采样）。

```
# 为了方便下载，在这里直接给出数据集下载命令,可以直接拷贝执行。
cd ${HOME}/samples/best_practices/contrib/depression_detection/inference/data
mkdir 01
cd 01
wget https://sharedata.obs.myhuaweicloud.com/depression_detection/tset_audio/01/02.wav
wget https://sharedata.obs.myhuaweicloud.com/depression_detection/tset_audio/01/03.wav
wget https://sharedata.obs.myhuaweicloud.com/depression_detection/tset_audio/01/23.wav

cd ..
mkdir 02
cd 02
wget https://sharedata.obs.myhuaweicloud.com/depression_detection/tset_audio/02/02.wav
wget https://sharedata.obs.myhuaweicloud.com/depression_detection/tset_audio/02/03.wav
wget https://sharedata.obs.myhuaweicloud.com/depression_detection/tset_audio/02/23.wav
```

### 1.4.2. 数据集预处理

​	执行预处理脚本preprocess.py，生成数据集预处理后的txt文件

```
python3 preprocess.py –-smile_file ${smile_file_path} –-config_file ${config_file_path} –audio_path ${audio_path} –output_path ${output_path}

示例：
python3.6 preprocess.py --smile_file /home/HwHiAiUser/opensmile/build/progsrc/smilextract/SMILExtract --config_file /home/HwHiAiUser/opensmile/config/emobase/emobase2010.conf --audio_path ./data/ --output_path ./dataout/
```

参数说明：

--smile_file：为opensmile工具SMILExtract文件路径。

--config_file：为opensmile工具emobase2010.conf文件路径。

--audio_path：音频文件路径。

--output_path：输出保存路径。

## 1.5. 样例运行

### 1.5.1. 样例

调用postprocess.py脚本与数据集标签比对，可以获得Accuracy数据。

```
python3 postprocess.py --feature_dir ${feature_dir} --model_path ${model_path}

示例：
python3.6 postprocess.py --feature_dir ./dataout/ --model_path ./model/
```

参数说明：

--feature_dir：特征txt文件目录路径，即之前预处理输出保存的路径。

--model_path：om模型所在文件夹。

### 1.5.2. 结果

![](https://sharedata.obs.myhuaweicloud.com/depression_detection/result.png)

### 1.5.3. 结果说明

​		本次运行两个样本，分别为抑郁样本和健康样本。推理结果为[1, 0]，其中1为1抑郁标签，0为健康标签，结果正确。一次推理时间约为0.6 ms。

