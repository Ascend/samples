# 基于Atlas200DK的实时视线估计之客户端

## 目录

- [目录](#目录)
- [功能描述](#功能描述)
- [代码结构](#代码结构)
- [PC客户端requirements.txt](#PC客户端requirements.txt)
- [客户端运行](#客户端运行)
  - [编写POST请求函数](#编写POST请求函数)
  - [客户端项目运行](#客户端项目运行)
-  [运行结果](#运行结果)
  - [视线校准](#视线校准)
  - [视线估计结果](#视线估计结果)

## 功能描述

基于Atlas200DK的实时视线估计推理的客户端，从客户端（PC端）采集人脸图像，对人脸图像做预处理并将预处理后的人脸图像上传至Atlas200DK端进行推理，返回的推理结果用于视线实时校准与视线点的显示。

## 代码结构

- Client
  - data
    - imgs：校准用的图像文件夹
    - eth-xgaze.yaml、sample_params.yaml 项目配置文件
  - Calibtration_4P.py：项目运行文件，4点校准
  - asset.py、face_cam.py：项目逻辑实现代码
  - pth2onnx.py：.pth模型文件转.onnx模型文件
  - test_oneVedio.py：使用已有的人脸视频进行视线估计

**在PC端的项目包含data和data目录即可**

## PC客户端requirements.txt

```python
pytorch==1.10.0
torchvision==0.11.0
numpy
opencv-python
Pillow
mediapipe
scipy
```

## 客户端运行

### 编写POST请求函数

Atlas200DK默认ip地址为192.168.1.2，经过Atlas200DK端的步骤配置了请求地址，则服务端的请求地址为 http://192.168.1.2:8000/inferenceByom/

**注：**若Atlas200DK的实际ip不是192.168.1.2，需将Client/asset.py中的gazeByAtlas200DK方法里的**atlas_url**变量的默认ip更换成实际ip。

通过下面gazeByAtlas200DK方法向Atlas200DK服务端不断发送带有面部图像数据的Post请求，并实时得到服务端请求处理结果

```python
def gazeByAtlas200DK(self,face):
    atlas_url = "http://192.168.1.2:8000/inferenceByom/"

    _, encoded_image = cv2.imencode(".jpg", face.normalized_image)  # 对图像进行编码
    byte_image = encoded_image.tobytes()  # 将数组转为bytes流
    data = {"face": byte_image} 
  
    response = self.session.post(atlas_url, files=data)  # 发送Post请求，将图像发送给后端 进行推理
    predict_gaze = response.text.split(" ")  # 对返回内容（视线估计结果）
    predict_gaze = np.array([float(predict_gaze[0]), float(predict_gaze[1])])

    return predict_gaze
```

### 客户端项目运行

**注：**本项目运行时的客户端（笔记本电脑）的摄像头在屏幕上方中央位置，屏幕大小为15.6寸，分辨率为1920*1080，若不满足该几个条件，需要根据笔记本实际参数修改项目中的某些参数！还需注意头部与摄像头位置齐平（视线估计可更加精确）。

**保证Atlas200DK和客户端设备都在一个局域网内**

1、进入Atlas200DK端中的Inference项目的目录下，先执行下面指令启动Django项目Inference

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

### 视线校准

Alt+Tab键切换至校准界面进行视线校准

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/xz_start.png)

校准过程，红点为校准测试者视线需要看的点，测试者看准红点后，用鼠标点击该红点即可完成一次校准

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/show2.png)

点击“校准结束”，完成校准

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/xz_end.png)

### 视线估计结果

1920*1080的屏幕，黑点为通过摄像头捕捉人脸图像推理得到的视线点

![](https://sharedata.obs.myhuaweicloud.com/RT-GazeEstimation/camera_client/show1.png)
