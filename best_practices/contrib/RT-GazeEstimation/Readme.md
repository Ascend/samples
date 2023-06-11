# 基于Atlas200DK的实时视线估计

## 目录

- [目录](#目录)
- [案例描述](#案例描述)
- [前置条件](#前置条件)
- [代码结构](#代码结构)
- [GPU复现](#GPU复现)
  - [数据集准备及数据预处理](#数据集准备及数据预处理)
  - [GPU复现依赖](#GPU复现依赖)
  - [GPU训练](#GPU训练)
  - [GPU验证](#GPU验证)
- [基于Atlas200DK的推理与部署](#基于Atlas200DK的推理与部署)
  - [.pth转.om模型](#.pth转.om模型)
  - [requirements.txt](#requirements.txt)
  - [项目运行](#项目运行)
- [运行结果](#运行结果)
  - [Atlas200DK端](#Atlas200DK端)
  - [客户端](#客户端)
- [精度及性能](#精度及性能)
  - [精度](#精度)
  - [性能](#性能)

## 案例描述

使用Atlas200DK运行用于视线估计的resnet18模型，对网络摄像头给出的RGB面部图像进行推理，实时估计受试者看屏幕的的视线位置，屏幕上的视线位置随着受试者眼睛移动而相应改变。

## 前置条件

| 条件               | 要求                                            | 备注                                                         |
| ------------------ | ----------------------------------------------- | ------------------------------------------------------------ |
| 硬件要求           | Atlas200DK、笔记本自带摄像头或USB外接网络摄像头 | 准备一台支持NPU推理的Atlas200DK硬件，并参考[Atlas200DK开发者套件](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha001/softwareinstall/instg/atlasdeploy_03_0024.html)在Atlas200DK上部署环境，本案例用的是1.0.13版本；摄像头用来捕捉人脸图像 |
| CANN版本           | 6.0.RC1.alpha001                                | 在Atlas200DK的制卡、配置网络连接等操作完成后，登录账号进入Atlas200DK，并参考 [CANN社区版](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha001/softwareinstall/instg/atlasdeploy_03_0024.html)在Atlas200DK上安装CANN社区版本6.0.RC1.alpha001 |
| Python及第三方依赖 | Python3.7.5、python-acllite                     | Python3.7.5的安装配置可在 [CANN社区版](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha001/softwareinstall/instg/atlasdeploy_03_0024.html)中可以参考，第三方依赖请参考Ascend/samples给出的 [第三方依赖安装指导（python样例）](https://github.com/Ascend/samples/tree/master/python/environment)选择性安装python-acllite等依赖 |
| Python Web框架     | Django3                                         | Atlas200DK环境中，使用命令pip3 install django安装Django框架，用来进行前后端交互 |

## 代码结构

- Client
  
  - data

    - imgs：校准用的图像文件夹
    - eth-xgaze.yaml、sample_params.yaml 项目配置文件
  
  - Calibtration_4P.py：项目运行文件，4点校准
  - asset.py、face_cam.py：项目逻辑实现代码
  - pth2onnx.py：.pth模型文件转.onnx模型文件
  - test_oneVedio.py：使用已有的人脸视频进行视线估计
  
- train：GPU复现训练代码文件夹
  - parameters：训练日志及模型保存目录
  - reader/reader.py：数据集加载
  - config_eth.yaml：训练超参数
  - ctools.py、gtools.py：计算训练时长、视线角度误差等工具代码
  - train.py：训练脚本
  - valid_ETH_GPU.py：.pth模型在GPU上的精度计算
  
- Inference：Django项目服务端，Atlas200DK接收请求，将推理结果返回客户端
  - common：python-acllite依赖
  - inference：项目主文件
  - model：om模型文件目录
  - atlas：Django应用
  
- valid：项目交互
  - hw_15P.py：实时视线估计的精度测试
  - valid_ETH_GPU.py：.pth模型在GPU上的精度计算
  - valid_ETH_NPU.py：.om模型在NPU上的精度计算

**在PC端的项目包含data和python目录即可，Atlas200DK端项目为demo1，GPU复现代码为train目录**，**交互件目录valid**

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

使用Python3.7.5，进入项目的train目录下，先打开train目录下的config_eth.yaml修改训练集和测试集的路径

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

```
pth = "../data/resnet18/Iter_10_resnet18.pth"
val_data = edict({"image": "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Image/valid_train",
                  "label": "F:/DeepLearning/datasets/Gaze/ETH-XGaze/Label/validation.label",
                  "header": True,
                  "name": "eth",
                  "isFolder": False
                  })
batch_size_valid = 64
```

执行该文件

```
python3 valid_ETH_GPU.py
```

## 基于Atlas200DK的推理与部署

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

### requirements.txt

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

**5、PC客户端整体所需依赖如下（见client_requirements.txt）**

```python
pytorch==1.10.0
torchvision==0.11.0
numpy
opencv-python
Pillow
mediapipe
scipy
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

由于人脸外观差异的存在，通过人脸图像估计眼睛视线会因人而异存在不同的视线偏差，为了使估计的视线更加精准，在执行客户端的脚本进行实时视线估计时，需先进行视线校准，可按**Alt+Tab**键切换至校准界面进行视线校准，视线校准需要眼睛看着界面的红点再点击鼠标左键（鼠标点击过程请看着红点，且不能眨眼），即可完成一个红点的校准（总共需要4个红点的校准）。

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

### Atlas200DK端

执行 python3 manage.py runserver 0.0.0.0:8000命令结果

![Image](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/inference_server/atlas_begin.png)

整个项目运行时，Atlas200DK端收到的POST请求

![Image](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/time2.png)



###  客户端

**视线校准：**

Alt+Tab键切换至校准界面进行视线校准

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/xz_start.png)

校准过程，红点为校准测试者视线需要看的点，测试者看准红点后，用鼠标点击该红点即可完成一次校准

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/show2.png)

点击“校准结束”，完成校准

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/xz_end.png)

**视线估计结果：**

1920*1080的屏幕，黑点为通过摄像头捕捉人脸图像推理得到的视线点

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/show1.png)

## 精度及性能

目录valid中是有关精度和性能评估的文件目录

### 精度

精度计算：真实视线方向和估计视线方向的夹角（单位：degree，即°），夹角越小模型精度越高

.pth和.om模型文件的精度测试均用**ETH-XGaze**数据集分出来的测试集进行验证，测试精度的数据集在下面链接处，请自行下载

**百度网盘链接：https://pan.baidu.com/s/1NJ3k_RGpXklaN_8vkSk7iw** 
**提取码：wb00**

valid_ETH_GPU.py和valid_ETH_NPU.py两个脚本分别验证.pth和.om模型的精度，验证时，需要先通过reader.py加载数据

1、GPU精度

在GPU上验证时，请直接将reader.py和valid_ETH_GPU.py放置同一个文件目录中，打开valid_ETH_GPU.py，将下面**pth**换成真实的.pth路径，**image**和**label**对应路径请换成链接下载的验证数据集的图像和标签所在路径，然后执行命令python3 valid_ETH_GPU.py运行valid_ETH_GPU.py脚本，执行完成后在控制台查看打印结果

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

2、NPU精度

前提：安装好第三方依赖，请参考Ascend/samples给出的 [第三方依赖安装指导（python样例）](https://github.com/Ascend/samples/tree/master/python/environment)

将valid_ETH_NPU.py与reader.py两个放在Atlas200DK端的Inference/atlas目录中，参考GPU精度验证过程，修改**model_path**和**image**和**label**的对应真实路径

```
model_path = "/home/HwHiAiUser/WB/om/resnet18_batch1.om"
val_data = edict({"image": "/home/HwHiAiUser/WB/ETH-XGaze/Image/valid_train",
                  "label": "/home/HwHiAiUser/WB/ETH-XGaze/Label/validation.label",
                  "header": True,
                  "name": "eth",
                  "isFolder": False
                  })
batch_size_valid = 1
```

3、模型精度对比

用上述给出的ETH-XGaze数据集的验证集分别对.pth和.om模型进行精度测试，测试结果如下：

**GPU精度：5.216°**

**NPU精度：5.221°**

精度相差0.005°，可以忽略不计，精度达标

4、实时精度测试：

实时精度测试，为了保证测量的准确性，若测试者佩戴了眼镜，请摘掉眼镜（眼镜反光、折射等会对视线估计造成重大干扰）后再进行测试，测试时，保持身体距屏幕50cm-60cm，头部姿势尽可能保持不动，仅让眼球移动时效果更夹。

**注：**本项目运行的笔记本的摄像头在屏幕上方中央位置，屏幕大小为15.6寸，分辨率为1920*1080，若不满足该几个条件，需要根据笔记本实际参数修改项目中的某些参数！还需注意头部与摄像头位置齐平（视线估计可更加精确）。

（1）启动精度测试程序与视线校准

将hw_15P.py脚本放置客户端的python目录下，python3 hw_15P.py执行该脚本，待程序启动后，Alt+Tab切换至界面如下图

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/xz_start.png)

点击“开始校准”，会在界面上下左右依次出现4个校准用于个人校准，测试者需从一而至保持身体和头部姿态尽可能不动，眼睛看准红点后持续1s左右，鼠标左键点击红点（会自动跳转到下一个红点），眼睛看红点时请不要眨眼（避免出现校准偏差），点击红点后方可眨眼，再按该方法用剩下的三个红点进行视线校准。第一个用于校准的红点如下：

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/show2.png)

校准完成后，出现下面界面：

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/xz_end.png)

若是测试者校准期间未参照上述做法导致视线不准，可点击“重新校准”再次校准，校准结束后点击“校准结束”进入视线精度测试流程。

（2）视线精度测试流程

校准结束流程结束后，界面会依次出现已知坐标的红点（总共15个不同坐标的红点）来测试，测试者需和视线校准流程一样，眼睛看清红点持续1秒左右，鼠标左键点击红点即可完成1个红点的测试，依次的15个红点测试结束后，在界面点击“测试结束”。控制台会将每个红点的视线测试结果打印出来，包括眼睛看红点时的真实视线和模型估计视线的视线误差角度、真实红点的位置和估计红点的位置的距离误差、15个点的加权平均角度误差、位置距离误差。

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/cs_end.png)

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/cs_result.png)

注：视线误差受人的面部外观、校准是否符合上述流程、光照强度、身体与屏幕的距离等因素影响，若是测试不够精准，请重新校准并测试。

**实时精度结果：**在正常的室内光照条件，受试者正对着笔记本屏幕且距离约60cm，头部姿势在水平和垂直角度均<10°的小范围活动的条件下，经过视线校准后，空间中3D视线估计角度误差**<5°**，基于笔记本电脑屏幕的2D距离误差**<3.5cm**；

### 性能

**om模型性能：**不到**3ms**

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/inference_server/om_xn.png)

**项目实时估计性能：**

方法1

执行项目时，在python目录下的Calibration_4P.py执行后，若想退出程序，可按Alt+Tab将人脸界面切换出来，按键Q可结束程序运行，成功启动一次后，关闭程序，再重新启动，运行一段时间后按Q结束，控制台会打印出性能指标，即1秒钟能进行多少帧的实时交互。经过多次测试，每进行1次交互，需要的交互时间不超过100ms（平均每秒交互10-11次）

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/time1.png)

方法2：

Atlas200DK端启动demo1项目后（python3 manage.py runserver 0.0.0.0:8000），在实时视线估计的交互过程中（python3 Calibtration_4P.py），会有POST请求记录，每秒POST请求的数可自行数

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/time2.png)

**实时性能：<100ms**

