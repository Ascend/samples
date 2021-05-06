中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.0.0及以上版本，支持产品为AscendBot**

** **

# 智能小车介绍
AscendBot是一款面向人工智能及机器人爱好者的开源智能机器人小车，同时也是一个开放的人工智能及机器人开发平台，它具备如下特性：
- 高性能：基于华为Atlas 200 DK，提供8TOPS@FP16的算力
- 易学习：从AI算法到应用均提供完整开发教程及示例代码
- 易上手：开放的硬件清单和搭建教程，开发者可自行动手组装扩展

## 样例功能

智能小车被手机APK遥控，实现物体跟随、车轨道循线、防跌落功能。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../environment)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。
- 完整的小车，包含各个小车部件，电池，路由器，显示屏，摄像头等都已经正确安装。

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

        开发环境，非root用户命令行中执行以下命令下载源码仓。

       **cd $HOME**

       **git clone https://github.com/Ascend/samples.git**

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

        2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

        3. 开发环境中，执行以下命令，解压zip包。
```
            cd $HOME
            unzip ascend-samples-master.zip
```
2. 获取此应用中所需要的原始网络模型。

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下目录：$HOME/samples/cplusplus/contrib/Ascbot。

    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  collision_avoidance_model| 检测小车前方是否有跌落危险 |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/ascbot/ATC_CollisionAntiDrop_caffe_AE/](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/ascbot/ATC_CollisionAntiDrop_caffe_AE/)目录中README.md下载原始模型章节下载模型和权重文件。 |
    |  road_following_model  |  检测车道线，实现循道行驶  |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/ascbot/ATC_LaneDetection_caffe_AE/](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/ascbot/ATC_LaneDetection_caffe_AE/)目录中README.md下载原始模型章节下载模型和权重文件。  |
    |  road_object_detection_deploy|  检测小车前方物体  |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/ascbot/ATC_Object_detection_caffe_AE/](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/ascbot/ATC_Object_detection_caffe_AE/)目录中README.md下载原始模型章节下载模型和权重文件。  |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

    > modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为Davinci模型。
   
    **注：请确认环境变量已经在[环境准备和依赖安装](../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。
```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export LD_LIBRARY_PATH=\\${install_path}/atc/lib64  
```
    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。
```
        wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/car/collision/insert_op_collision_avoidance.cfg

        atc --model="collision_avoidance_model.prototxt" --weight="collision_avoidance_model.caffemodel" --soc_version=Ascend310 --framework=0 --output="collision_avoidance_model" --insert_op_conf=insert_op_collision_avoidance.cfg
        
        wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/car/following/insert_op_road_following.cfg

        atc --model="road_following_model.prototxt" --weight="road_following_model.caffemodel" --soc_version=Ascend310 --framework=0 --output="road_following_model" --insert_op_conf=insert_op_road_following.cfg

        wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/car/object_detection/insert_op_road_object_detection_deploy.cfg

        atc --model="road_object_detection_deploy.prototxt" --weight="road_object_detection_deploy.caffemodel" --soc_version=Ascend310 --framework=0 --output="road_object_detection_deploy" --insert_op_conf=insert_op_road_object_detection_deploy.cfg
```

### 样例部署

1. 开发环境命令行中设置编译依赖的环境变量。
```
     export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
     export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
```
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 如果是3.0.0版本，此处 **DDK_PATH** 环境变量中的 **arm64-liunx** 应修改为 **arm64-linux_gcc7.3.0**。    
        > - 可以在命令行中执行 **uname -a**，查看开发环境和运行环境的cpu架构。如果回显为x86_64，则为x86架构。如果回显为arm64，则为Arm架构。

2. 切换到ascbot_c75目录，创建目录用于存放编译文件，例如，本文中，创建的目录为 **build/intermediates/host**。
```
    cd $HOME/samples/cplusplus/contrib/Ascbot
    mkdir -p build/intermediates/host
```
3. 切换到 **build/intermediates/host** 目录，执行cmake生成编译文件。
```
    cd build/intermediates/host
    make clean        
    cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**
```
4. 执行make命令，生成的可执行文件main在 **ascbot_c75/out** 目录下。

    **make**

### 样例运行

1. 执行以下命令,将开发环境的 **ascend_bot** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
```
    scp -r $HOME/samples/cplusplus/contrib/Ascbot HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx    
```
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2。


2. 设置环境

    进入/etc/rc.local。
```
    vim /etc/rc.local
```
    添加以下指令
 ```   
    echo 504 >/sys/class/gpio/export
    echo 444 >/sys/class/gpio/export
    chown -R HwHiAiUser /sys/class/gpio/gpio444
    chown -R HwHiAiUser /sys/class/gpio/gpio504
    chown -R HwHiAiUser /sys/class/gpio/gpio444/direction
    chown -R HwHiAiUser /sys/class/gpio/gpio504/direction
    chown -R HwHiAiUser /sys/class/gpio/gpio444/value
    chown -R HwHiAiUser /sys/class/gpio/gpio504/value
    chown -R HwHiAiUser /dev/i2c-1
    chown -R HwHiAiUser /dev/i2c-2
    chown -R HwHiAiUser /dev/ttyAMA0
    usermod -aG HwHiAiUser HwHiAiUser
 ```   
3. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/samples/cplusplus/contrib/Ascbot/out**

    切换目录后，执行以下命令运行样例。

    **./main**

### 查看结果

运行完成后，可下载手机端应用控制小车运行。
[手机端下载地址](https://share.weiyun.com/5lsbfzF)



