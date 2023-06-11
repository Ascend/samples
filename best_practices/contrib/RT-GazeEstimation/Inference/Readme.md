# 基于Atlas200DK的实时视线估计之Atlas200DK端

## 目录

- [目录](#目录)

- [功能描述](#功能描述)
- [前置条件](#前置条件)
- [代码结构](#代码结构)
- [基于Atlas200DK的推理与运行](#基于Atlas200DK的推理与运行)
  - [.pth转.om模型](#.pth转.om模型)
  - [Atlas200DK端requirements.txt](#Atlas200DK端requirements.txt)
  - [项目运行](#项目运行)
-  [运行结果](#运行结果)

## 功能描述

基于Atlas200DK的实时视线估计推理之Atlas200DK端，加载视线估计的om模型，对客户端（PC端）上传的人脸图像进行推理，并将推理结果返回给客户端。

## 前置条件

| 条件               | 要求                                            | 备注                                                         |
| ------------------ | ----------------------------------------------- | ------------------------------------------------------------ |
| 硬件要求           | Atlas200DK、笔记本自带摄像头或USB外接网络摄像头 | 准备一台支持NPU推理的Atlas200DK硬件，并参考[Atlas200DK开发者套件](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha001/softwareinstall/instg/atlasdeploy_03_0024.html)在Atlas200DK上部署环境，本案例用的是1.0.13版本；摄像头用来捕捉人脸图像 |
| CANN版本           | 6.0.RC1.alpha001                                | 在Atlas200DK的制卡、配置网络连接等操作完成后，登录账号进入Atlas200DK，并参考 [CANN社区版](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha001/softwareinstall/instg/atlasdeploy_03_0024.html)在Atlas200DK上安装CANN社区版本6.0.RC1.alpha001 |
| Python及第三方依赖 | Python3.7.5、python-acllite                     | Python3.7.5的安装配置可在 [CANN社区版](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha001/softwareinstall/instg/atlasdeploy_03_0024.html)中可以参考，第三方依赖请参考Ascend/samples给出的 [第三方依赖安装指导（python样例）](https://github.com/Ascend/samples/tree/master/python/environment)选择性安装python-acllite等依赖 |
| Python Web框架     | Django3                                         | Atlas200DK环境中，使用命令pip3 install django安装Django框架，用来进行前后端交互 |

## 代码结构

- Inference：Django项目服务端，Atlas200DK接收请求，将推理结果返回客户端
  - common：python-acllite依赖
  - inference：项目主文件
  - model：om等模型文件目录
  - atlas：Django应用

**Atlas200DK端项目为Inference**

## 基于Atlas200DK的推理与运行

### .pth转.om模型

**1、.pth文件转.onnx文件**

.pth转.onnx可参考[Ascend PyTorch模型离线推理指导](https://gitee.com/wangjiangben_hw/ascend-pytorch-crowdintelligence-doc/blob/master/Ascend-PyTorch%E7%A6%BB%E7%BA%BF%E6%8E%A8%E7%90%86%E6%8C%87%E5%AF%BC/PyTorch%E7%A6%BB%E7%BA%BF%E6%8E%A8%E7%90%86-%E7%A6%BB%E7%BA%BF%E6%8E%A8%E7%90%86%E6%8C%87%E5%AF%BC.md#211-%E5%AF%BC%E5%87%BAonnx%E6%96%87%E4%BB%B6) ，通过GPU复现可得到训练好的.pth权重文件（或直接使用data/resnet18中已经训练好的.pth文件）转换成.onnx文件，运行以下python/pth2onnx.py脚本即可，进入python目录，执行以下命令

```python
# 在用户命令行使用下面命令下载.pth文件：
cd ${HOME}/samples/best_practices/contrib/RT-GazeEstimation/Inference/model
wget https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/inference_server/Iter_10_resnet18.pth

# 将.pth转换为.onnx
python3 pth2onnx.py
# 或者在用户命令行中直接执行以下命令
wget https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/inference_server/resnet18.onnx
```

**2、.onnx文件转.om文件**

目前仅支持在Ascend310处理器上使用ATC工具进行om模型转换。

（1）前提条件	

​	Atlas200DK上安装CANN社区版6.0.RC1.alpha001

（2）设置环境变量，.bashrc添加如下语句，执行source ~/.bashrc使环境生效

```python
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/var/davinci/driver/lib64:/var/davinci/driver/lib64/common:/var/davinci/driver/lib64/driver
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/fwkacllib/lib64:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/atc/lib64
export PYTHONPATH=$PYTHONPATH:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/fwkacllib/python/site-packages:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/toolkit/python/site-packages:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/atc/python/site-packages:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/pyACL/python/site-packages/acl
export PATH=$PATH:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/fwkacllib/ccec_compiler/bin:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/fwkacllib/bin:/home/HwHiAiUser/Ascend/ascend-toolkit/latest/atc/bin
export ASCEND_AICPU_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
export ASCEND_OPP_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/opp
export TOOLCHAIN_HOME=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/toolkit
```

（3）激活CANN-toolkit环境

```python
bash /home/HwHiAiUser/Ascend/ascend-toolkit/set_env.sh
```

（4）使用atc工具将.onnx模型文件转为.om模型文件，工具使用方法可以参考[CANN V100R020C10 开发辅助工具指南 (推理) 01](https://gitee.com/link?target=https%3A%2F%2Fsupport.huawei.com%2Fenterprise%2Fzh%2Fdoc%2FEDOC1100164868%3FidPath%3D23710424%7C251366513%7C22892968%7C251168373)

（5）需要指定输出节点以去除无用输出，使用netron开源可视化工具查看onnx模型的具体的输出节点名；

（6）可使用如下命令将onnx文件转为bach size为1的om文件

```python
# 了方便下载，在这里直接给出模型转换命令,可以直接拷贝执行
atc --framework=5 --model=resnet18.onnx --output=resnet18_batch1 --input_format=NCHW --input_shape="input:1,3,224,224" --log=debug --soc_version=Ascend310
```

### Atlas200DK端requirements.txt

**1、安装pytorch 和torchvision**

Atlas200DK是基于arm架构的硬件，安装pytorch1.8.1需要安装arm版本，可参考[PyTorch安装指南](https://github.com/Ascend/pytorch/blob/v1.8.1-3.0.rc3/docs/zh/PyTorch%E5%AE%89%E8%A3%85%E6%8C%87%E5%8D%97/PyTorch%E5%AE%89%E8%A3%85%E6%8C%87%E5%8D%97.md#cpu%E6%9E%B6%E6%9E%84%E4%B8%BAarm%E6%9E%B6%E6%9E%84%E6%97%B6%E7%94%B1%E4%BA%8E%E7%A4%BE%E5%8C%BA%E6%9C%AA%E6%8F%90%E4%BE%9Barm%E6%9E%B6%E6%9E%84cpu%E7%89%88%E6%9C%AC%E7%9A%84torch%E5%8C%85%E6%97%A0%E6%B3%95%E4%BD%BF%E7%94%A8pip3%E5%91%BD%E4%BB%A4%E5%AE%89%E8%A3%85pytorch181%E9%9C%80%E8%A6%81%E4%BD%BF%E7%94%A8%E6%BA%90%E7%A0%81%E7%BC%96%E8%AF%91%E5%AE%89%E8%A3%85) 安装pytorch1.8.1的CPU版本和torchvision0.9.1

**2、安装Python Web框架：Django**

在Atlas200DK中使用 pip3 install django 命令安装Django框架

**3、安装第三方依赖python-acllite**

请参考Ascend/samples给出的 [第三方依赖安装指导（python样例）](https://github.com/Ascend/samples/tree/master/python/environment)选择性安装python-acllite等依赖

**4、Atlas200DK推理端整体所需依赖如下（见server_requirements.txt）：**

```python
pytroch==1.8.1
torchvision==0.9.1
numpy
opencv-python
Pillow
easydict
```

### 项目运行

**注：**本项目运行时的客户端（笔记本电脑）的摄像头在屏幕上方中央位置，屏幕大小为15.6寸，分辨率为1920*1080，若不满足该几个条件，需要根据笔记本实际参数修改项目中的某些参数！还需注意头部与摄像头位置齐平（视线估计可更加精确）。

**保证Atlas200DK和客户端设备都在一个局域网内**

1、进入Atlas200DK端中的Inference项目的目录下，**model**目录下是存放.om模型文件的，请用户按照前面的命令将转换好的.om文件放在该model目录下即可，先执行下面指令启动Django项目Inference

```python
python3 manage.py runserver 0.0.0.0:8000
```

2、进入Client/data/imgs目录使用下面的命令下载校准用的红点图像

```python
# 进入Client/data目录下载校准用的红点图片
cd ${HOME}/samples/best_practices/contrib/RT-GazeEstimation/Client/data/imgs
wget https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/path870.png
```

在PC端执行案例中Client目录下的Calibtration_4P.py

```python
python3 Calibtration_4P.py
```

由于人脸外观差异的存在，通过人脸图像估计眼睛视线会因人而异存在不同的视线偏差，为了使估计的视线更加精准，在执行客户端的脚本进行实时视线估计时，需先进行视线校准，可按**Alt+Tab**间切换至校准界面进行视线校准，视线校准需要眼睛看着界面的红点再点击鼠标左键（鼠标点击过程请看着红点，且不能眨眼），即可完成一个红点的校准（总共需要4个红点的校准）。

若想结束程序，请将面部图像展示窗口切换出来，按q即可结束程序。

**注：**在客户端代码执行前，需保证Atlas200DK端的项目已运行，且需要核对Atlas200DK的IP地址是否为192.168.1.2，若不是该IP地址，请在Client/asset.py脚本中找到gazeByAtlas200DK方法，将**atlas_url**的默认的IP地址192.168.1.2更换为实际的IP地址

```python
def gazeByAtlas200DK(self,face):
    atlas_url = "http://192.168.1.2:8000/inferenceByom/"
```

3、不执行步骤2的实时视线估计，使用已存在的录制视频运行项目

先进入Client/data目录，使用下面命令下载提供的人脸视频

```python
# 进入Client/data目录下载提供的人脸视频
cd ${HOME}/samples/best_practices/contrib/RT-GazeEstimation/Client/data
wget https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/face.avi
```

执行Client目录下的test_oneVedio.py，不需要校准

```python
python3 test_oneVedio.py
```

## 运行结果

执行 python3 manage.py runserver 0.0.0.0:8000命令结果

![Image](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/inference_server/atlas_begin.png)

整个项目运行时，Atlas200DK端收到的POST请求

![Image](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/time2.png)