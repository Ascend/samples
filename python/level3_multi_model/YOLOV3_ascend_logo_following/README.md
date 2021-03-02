**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.1及以上版本，支持产品为Atlas200DK。**

## YOLOV3_ascend_logo_following样例

功能：使用yolov3模型对双目相机[IntelRealsense D435](https://www.intelrealsense.com/depth-camera-d435/ )的RGB数据进行推理，检测Ascend_logo的位置，并利用Ascend_logo的位置控制机械臂[Dobot-Magician](https://cn.dobot.cc/dobot-magician/product-overview.html )的移动，使双目相机对准Ascend_logo。

### 前提条件
部署此Sample前，需要准备好双目相机[IntelRealsense D435](https://www.intelrealsense.com/depth-camera-d435/ )以及机械臂[Dobot-Magician](https://cn.dobot.cc/dobot-magician/product-overview.html )：

- 请确认已按照[环境准备和依赖安装](../../environment)准备好环境。

- 已完成200DK的开发环境和运行环境安装，并配置200DK开发板联网。
  
- 已完成[IntelRealsense D435](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide?_ga=2.126501389.1287839862.1614240661-1973899397.1612341743 )SDK的安装。

- 完成[Dobot机械臂](https://github.com/luismesas/pydobot )Python API的安装。

    首先参考[USB to UART Bridge VCP Drivers](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers )进行驱动安装；

    然后下载机械臂[Dobot-Magician](https://cn.dobot.cc/dobot-magician/product-overview.html )的Python API：
    
    **git clone https://github.com/luismesas/pydobot.git**

    下载完成后，进入pydobot文件夹，修改dobot.py，在line275后添加：
    ````python
    def move_by_angle(self, j1, j2, j3, j4, wait=False):
        self._set_ptp_cmd(j1, j2, j3, j4, mode=PTPMode.MOVL_ANGLE, wait=wait)
    ````
  
    执行安装命令完成API的安装：
  
    **python3.7.5 setup.py build**
    
    **python3.7.5 setup.py install**

    **说明：**  
    > - 由于机械臂Dobot-Magician的官方Python API没有提供角度控制模式，所以需要手动添加move_by_angle模式。

- 将双目相机IntelRealsense固定在机械臂的末端中心，如下图：

![IntelRealsense](IntelRealsense.png)

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

        开发环境，非root用户命令行中执行以下命令下载源码仓。

       **cd $HOME**

       **git clone https://gitee.com/ascend/samples.git**

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

        2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

        3. 开发环境中，执行以下命令，解压zip包。

            **cd $HOME**

            **unzip ascend-samples-master.zip**

2. 获取此应用中所需要的原始网络模型。
 
    参考下表获取此应用中所用到的模型，并将其存放到开发环境普通用户下的工程目录：   
 **cd $HOME/samples/python/level3_multi_model/YOLOV3_ascend_logo_following/model** 
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  yolov3_ascend_logo| 基于Tensorflow-YOLOV3的ascend_logo检测模型。  |  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3_darknet53/ATC_yolov3_darknet53_tf_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3_darknet53/ATC_yolov3_darknet53_tf_AE )目录中README.md下载原始模型章节下载模型和权重文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中已经提供了转换好的om模型，可以直接使用。  

### 样例说明

1. 软件架构说明。
   
   机械臂跟随ascend_logo样例的整个框架如下图，200DK运行环境与Ubuntu服务器开发环境通过路由器进行交互，200DK负责yolov3目标检测模型的推理，Ubuntu服务器通过路由器向200DK发送IntelRealsense的RGB数据，同时接收200DK的推理结果，利用推理结果对机械臂进行控制。

![Architecture](architecture.png)


2. 机械臂的控制逻辑。
   
   Ubuntu服务器根据推理结果中ascend_logo在RGB图像中的位置，计算机械臂j1、j2、j3轴的移动角度，计算过程如下：

    - 待补充


### 样例运行
1. 测试机械臂Dobot-Magician和双目相机IntelRealsense是否可以正常工作。

    测试机械臂是否可以正常工作在move_by_angle模式：
   
    **cd samples/python/level3_multi_model/YOLOV3_ascend_logo_following/src** 

    **python3.7.5 dobot_test.py**    

    双目相机IntelRealsense的测试可以参考[IntelRealSense-python-examples](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples )。
   
2. 执行以下命令，将开发环境的 **samples** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    
    **scp -r $HOME/samples/ HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2。

3. 在运行环境运行可执行文件，等待接收IntelRealsense的RGB数据进行推理。

    **cd $HOME/samples/python/level3_multi_model/YOLOV3_ascend_logo_following/src**
    
    切换目录后，执行以下命令运行样例。
    
    **python3.6 yolov3_ascend_detect.py**
    

4.  在开发环境运行可执行程序，发送IntelRealsense数据到运行环境，并接收运行环境的推理结果来控制Dobot-Magician机械臂。

    **cd $HOME/samples/python/level3_multi_model/YOLOV3_ascend_logo_following/src/**
    
    切换目录后，执行以下命令运行样例。
    
    **python3.7.5 demo.py**


### 查看结果
运行成功后，可以用Ascend_logo来控制机械臂的移动，效果如下：

![demo](demo.gif) 






