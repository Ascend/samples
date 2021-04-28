# AMCT_ACL ResNet-50示例

该示例包含 ResNet-50的均匀量化示例。

## 1， 准备工作

### 1.1 原始模型(Pre-trained model)

本样例依赖指定的原始模型定义文件(.prototxt) 与预训练权重文件(.caffemodel)。通过执行以下命令进行下载：
```bash
python3.7.5 ./src/download_models.py --close_certificate_verify
```
若执行成功，将会下载 `ResNet-50-deploy.prototxt` 与 `ResNet-50-model.caffemodel` 到该样例根目录的`model`路径下。

若用户环境无法连接网络,请先在可连通网络的服务器下载相应文件后上传到`model` 路径下：
```bash
wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet_50/ResNet-50-deploy.prototxt
wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet_50/ResNet-50-model.caffemodel
```

### 1.2 示例数据集下载及预处理

校准集用来产生量化因子，保证精度。计算量化参数的过程被称为“校准(calibration)”。校准过程需要使用一部分图片来针对性计算量化参数，使用一个或多个batch对量化后的网络模型进行推理即可完成校准。为了保证量化精度，校准集与测试精度的数据集来源应一致。

该示例提供了一组样例校准集用于量化校准，可切换到该样例根目录下执行以下命令获取校准数据与标签:
```bash
cd data
mkdir image && cd image
wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/classification/calibration.rar
unrar e calibration.rar
```
如果环境中没有安装`urar`,可以通过以下命令获取:
```
sudo apt-get unrar
```

执行如下命令将image目录下的*.jpg图片转换为bin格式数据集
```
python3.7.5 ./src/process_data.py
```
执行完成后，在image目录生成calibration.bin数据集

## 2， 执行样例

### 2.1 执行均匀量化

```bash
bash ./scripts/run_calibration.sh 
```


若出现如下信息则说明量化成功：

```
amct_acl generate deploy air success!
```