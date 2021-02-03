中文|[English](README_EN.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 高清图像修复 （python）

功能：超高分辨率图像修复。

样例输入：待修复的jpg图片以及对应的mask图片；

样例输出：修复后的图片。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../../environment)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。

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

2. 获取此应用中所需要的单算子Json文件和om模型文件。  

     **wget -P ~/imageinpainting_hifill https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json**   
     **wget -P ~/imageinpainting_hifill https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.om**
    

3. 将单算子Json文件转换为Davinci模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令,使用atc命令进行模型转换。  

        **cd ~/imageinpainting_hifill** 

        **atc --singleop=./matmul_27648.json --output=./0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648 --soc_version=Ascend310**   

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./hifill.om \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/model/** 
 
        **cp ./0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648/*.om \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/model/**

4. 获取样例需要的测试图片。

    执行以下命令，下载对应的测试图片。

    **cd \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/data**

    **wget https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/data/test.jpg**
    
    **cd \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/mask**

    **wget https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/mask/test.jpg** 



### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **imageinpainting_hifill** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/src**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/imageinpainting_hifill/src**      

    切换目录后，执行以下命令运行样例。

    **python3.6 main.py**
### 查看结果

运行完成后，会在out目录下生成带推理结果的jpg图片。
