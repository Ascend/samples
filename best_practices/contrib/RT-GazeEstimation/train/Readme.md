# 基于Atlas200DK的实时视线估计之GPU训练

## 目录

- [目录](#目录)
- [功能描述](#功能描述)
- [代码结构](#代码结构)
- [GPU复现](#GPU复现)
  - [数据集准备及数据预处理](#数据集准备及数据预处理)
  - [GPU复现依赖](#GPU复现依赖)
  - [GPU训练](#GPU训练)
  - [GPU验证](#GPU验证)

## 功能描述

本部分用于视线估计的GPU训练和验证

## 代码结构

- train：GPU复现训练代码文件夹
  - parameters：训练日志及模型保存目录
  - reader/reader.py：数据集加载
  - config_eth.yaml：训练超参数
  - ctools.py、gtools.py：计算训练时长、视线角度误差等工具代码
  - train.py：训练脚本
  - valid_ETH_GPU.py：.pth模型在GPU上

## GPU复现

### 数据集准备及数据预处理

**1、获取原始数据**

​    本项目使用[ETH-XGaze数据集](https://link.springer.com/chapter/10.1007/978-3-030-58558-7_22)作为项目的训练集，ETH-XGaze是一个超过100万张图像的视线估计数据集

**2、预处理**

​    参考[GazeHub](http://phi-ai.buaa.edu.cn/Gazehub/3D-dataset/#eth-xgaze) 对ETH-XGaze数据集的预处理

### GPU复现依赖

```python
pytorch          # 1.10.0     
torchvision      # 0.11.0
numpy            # 1.21.5
opencv-python    # 4.5.5.64
easydict         # 1.9
timm             # 0.6.7
```

### GPU训练

使用Python3.7.5，进入项目的train目录下，先打开train目录下的config_eth.yaml修改经过预处理后的训练集和测试集的路径

```python
# 训练集路径
data:
    image: "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Image/train"
    label: "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Label/train_chaifen.label"
# 测试集路径
val:
    image: "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Image/train"
    label: "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Label/validation.label"
```

运行train.py进行GPU复现，**训练的日志和.pth模型文件保存在parameters目录下**

```python
python3 train.py -s config_eth.yaml
```

### GPU验证

拿到上面预处理后的数据集，更改valid_ETH_GPU.py中的.pth路径和测试集的image与label路径，

```python
pth = "../data/resnet18/Iter_10_resnet18.pth"
val_data = edict({"image": "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Image/valid_train",
                  "label": "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Label/validation.label",
                  "header": True,
                  "name": "eth",
                  "isFolder": False
                  })
batch_size_valid = 64
```

执行该脚本文件

```
python3 valid_ETH_GPU.py
```

