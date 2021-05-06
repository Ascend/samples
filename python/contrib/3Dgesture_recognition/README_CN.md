中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配CANN3.0.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导**

## 3D动作识别样例

功能：使用3DCNN模型对数据进行分类推理。

样例输入：使用video2bin脚本将视频文件转为数据文件。

样例输出：10个动作的置信度。


### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](https://github.com/Ascend/samples/tree/master/python/environment)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。

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

            **cd $HOME**

            **unzip ascend-samples-master.zip**

2. 获取此应用中所需要的原始网络模型。

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/3Dgesture_recognition

    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    | 3DCNN | 3D动作识别模型。是基于tensorflow的3DCNN模型。 | 请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/3dcnn/ATC_3DCNN_tensorflow_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/3dcnn/ATC_3DCNN_tensorflow_AE)目录中README.md下载原始模型章节下载模型。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为Davinci模型。
   
    **注：请确认环境变量已经在[环境准备和依赖安装](https://github.com/Ascend/samples/tree/dev/python/environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/3Dgesture_recognition**  

        **atc --model=3d_gesture_recognition.pb  --framework=3 --output=3d_gesture_recognition --soc_version=Ascend310 --input_shape="X:1,16,112,112,3" --input_format=NDHWC**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./3d_gesture_recognition.om $HOME/samples/python/contrib/3Dgesture_recognition/model/**

4. 获取样例需要的测试图片。

    执行以下命令，进入样例的data文件夹中，下载对应的测试图片。

    **cd $HOME/samples/python/contrib/3Dgesture_recognition/data**

    **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/3D_gesture_recognition/testdata//test_float32_actiontype7.bin**


### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的3Dgesture_recognition** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/python/contrib/3Dgesture_recognition  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
      
      **cd $HOME/samples/python/contrib/3Dgesture_recognition/**     

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/3Dgesture_recognition/**      

    切换目录后，执行以下命令运行样例。

    **python3.6 src/3Dgesture_recognition.py ./data/**
### 查看结果

运行完成后，会在outputs目录下生成带推理结果的jpg图片。
