**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio运行图片样例?sort_id=3164874)。**

**本样例为清华大学贡献**

## portrait_picture样例

功能：使用PortraitNet模型对输入图片中人像进行分割，然后与背景图像融合，实现背景替换。

样例输入：带有人像的ipg图片和一张背景图像

样例输出：背景替换后的图像。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](https://gitee.com/ascend/samples/blob/master/python/environment)准备好环境。
- 已完成对应产品的开发环境和运行环境安装。

### 软件准备

#### 1. 获取源码包。

  可以使用以下两种方式下载，请选择其中一种进行源码准备。

   - 命令行方式下载（下载时间较长，但步骤简单）。

     开发环境，非root用户命令行中执行以下命令下载源码仓。
        ```
     cd $HOME
     git clone https://gitee.com/ascend/samples.git
        ```
   - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

     1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

     2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

     3. 开发环境中，执行以下命令，解压zip包。
       
      ```
     cd $HOME
     unzip ascend-samples-master.zipt
      ```
#### 2. 获取此应用中所需要的模型（20.0版本）

   参考下表获取此应用中所用到的模型，并将其存放到开发环境普通用户下的工程目录：

	cd $HOME/samples/python/contrib/portrait_picture/model

| **模型名称** | **模型说明**                   | **模型下载路径**                                             |
| ------------ | ------------------------------ | ------------------------------------------------------------ |
| Portrait     | 基于TensorFlow的人像分割模型。 | 请参考https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/portraitnet/ATC_PortraitNet_tf_AE 中README.md原始模型章节，下载**原始模型**及**对应的cfg文件**。 |

#### 3. 将原始模型转换为Davinci模型（20.0 版本）

   **注：请确认环境变量已经在[环境准备和依赖安装](https://gitee.com/ascend/samples/blob/master/python/environment)中配置完成**

   1. 设置LD_LIBRARY_PATH环境变量。

      由于LD_LIBRARY_PATH环境变量在使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

         ```	
      export LD_LIBRARY_PATH=${install_path}/atc/lib64
         ```
	
   2. 执行以下命令使用atc命令进行模型转换。
         ```
      cd $HOME/samples/python/contrib/portrait_picture/model
      atc --model=./portrait.pb  --insert_op_conf=./insert_op.cfg  --output="./portrait" --output_type=FP32 --input_shape="Inputs/x_input:1,224,224,3" --framework=3 --soc_version=Ascend310
         ```
#### 4. 直接获取Davinci模型（20.1 版本）

​	由于版本问题，此模型在20.1版本不能正确转换。因此20.1版本直接获取om模型。

    cd  $HOME/samples/python/contrib/portrait_picture/model
    wget  https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/portrait.om

#### 5. 获取样例需要的测试图片

执行以下命令，进入样例的data文件夹中，下载对应的测试图片。

    cd $HOME/samples/python/contrib/portrait_picture/model
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/Portrait/background.jpg
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/Portrait/ori.jpg


### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](https://gitee.com/ascend/samples/tree/master/python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture#step_2)即可。**

1. 执行以下命令,将开发环境的**portrait_picture**目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
      ```
   scp -r $HOME/samples/python/contrib/portrait_picture/  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   scp -r $HOME/samples/python/common/atlas_utils/   HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx
      ```

   ![icon-note.gif](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif) **说明：**

   > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。


2. 运行可执行文件。

   - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。
	```
     export LD_LIBRARY_PATH=
     source ~/.bashrc
     cd $HOME/samples/python/contrib/portrait_picture/src
     python3 main.py 
	```
   - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
	```
     cd $HOME/python/portrait_picture/src
	```
     切换目录后，执行以下命令运行样例。
   
	```
     python3.6 main.py 
	```


### 查看结果

运行完成后，会在运行环境的命令行中打印出推理结果。