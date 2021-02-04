**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 模糊图像变清晰

**注：DeblurGAN模型支持动态shape，本案例使用固定shape：1280*720。**

功能：输入一张模糊图片，使用DeblurGAN将其变清晰。

样例输入：待推理的图片，支持jpg/png图。

样例输出：推理后的图片。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](https://gitee.com/ascend/samples/tree/master/cplusplus/environment)准备好环境。

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

2. 获取此应用中所需要的原始网络模型。


    参考下表获取此应用中所用到的原始网络模型，并将其存放到开发环境普通用户下该样例的model文件夹中，本例为：$HOME/samples/cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp/model/。
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  DeblurGAN| 图像变清晰模型。是基于Tensorflow的DeblurGAN模型。  |  请参考[https://gitee.com/chen68/modelzoo/tree/master/contrib/TensorFlow/Research/cv/Blur2Sharp/ATC_DeblurGAN_tf_AE](https://gitee.com/chen68/modelzoo/tree/master/contrib/TensorFlow/Research/cv/Blur2Sharp/ATC_DeblurGAN_tf_AE)目录中README.md下载原始模型章节下载模型文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中提供了转换好的om模型，可直接下载使用，也可下载原始模型重新进行模型转换。

3. 将原始模型转换为Davinci模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](https://gitee.com/ascend/samples/tree/dev/cplusplus/environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令使用atc命令进行模型转换。
        
        **atc --input_shape="blur:1,720,1280,3" --input_format=NHWC --output="./DeblurrGAN_pad_1280_720" --soc_version=Ascend310 --framework=3 --model="./DeblurrGAN-pad-01051648.pb" --log=info**



4. 获取样例需要的测试图片。

    执行以下命令，进入样例的data文件夹中，下载对应的测试图片。

    **cd $HOME/samples/cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp/data**

    **wget https://c7xcode.obs.myhuaweicloud.com/models/DeblurGAN_GOPRO_Blur2Sharp/data.png**


### 样例部署
 
1. 开发环境命令行中设置编译依赖的环境变量。

   基于开发环境与运行环境CPU架构是否相同，请仔细看下面的步骤：

   - 当开发环境与运行环境CPU架构相同时，执行以下命令导入环境变量。

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

   - 当开发环境与运行环境CPU架构不同时，执行以下命令导入环境变量。例如开发环境为X86架构，运行环境为Arm架构，由于开发环境上同时部署了X86和Arm架构的开发套件，后续编译应用时需要调用Arm架构开发套件的ACLlib库，所以此处需要导入环境变量为Arm架构的ACLlib库路径。

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 如果是20.0版本，此处 **DDK_PATH** 环境变量中的 **arm64-liunx** 应修改为 **arm64-linux_gcc7.3.0**。
        > - 可以在命令行中执行 **uname -a**，查看开发环境和运行环境的cpu架构。如果回显为x86_64，则为x86架构。如果回显为arm64，则为Arm架构。

2. 切换到DeblurGAN_GOPRO_Blur2Sharp目录，创建目录用于存放编译文件，例如，本文中，创建的目录为 **build/intermediates/host**。

    **cd $HOME/samples/cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp**

    **mkdir -p build/intermediates/host**

3. 切换到 **build/intermediates/host** 目录，执行cmake生成编译文件。

    - 当开发环境与运行环境操作系统架构相同时，执行如下命令编译。
      
      **cd build/intermediates/host**   

      **make clean**

      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

    - 当开发环境与运行环境操作系统架构不同时，需要使用交叉编译器编译。例如开发环境为X86架构，运行环境为Arm架构，执行以下命令进行交叉编译。

      **cd build/intermediates/host**

      **make clean**
    
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

4. 执行make命令，生成的可执行文件main在 **DeblurGAN_GOPRO_Blur2Sharp/out** 目录下。

    **make**

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **DeblurGAN_GOPRO_Blur2Sharp** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp/out**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/DeblurGAN_GOPRO_Blur2Sharp/out**

    切换目录后，执行以下命令运行样例。

    **mkdir output**

    **./main ../data**

### 查看结果

运行完成后，会在运行环境的命令行中打印出推理结果,并在$HOME/DeblurGAN_GOPRO_Blur2Sharp/out/output目录下生成推理后的图片。