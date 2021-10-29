中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.0.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 卡通图像生成样例

功能：使用cartoonGAN模型对输入图片进行卡通化处理。

样例输入：原始图片jpg文件。

样例输出：带推理结果的jpg文件。

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

      **git clone https://github.com/Ascend/samples.git**

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

      1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

      2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

      3. 开发环境中，执行以下命令，解压zip包。

         **cd $HOME**

         **unzip ascend-samples-master.zip**

2. 获取此应用中所需要Davinci模型。

   - 3.1.0版本
     由于版本问题，此模型在3.1.0版本不能正确转换。因此3.1.0版本直接获取om模型。

       **cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/model**    
       **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/cartoonGAN_picture/cplus/cartoonization.om** 

   - 3.0.0版本    


     1.参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/cartoonGAN_picture。

     | **模型名称**   | **模型说明**       | **模型下载路径**                                             |
     | -------------- | ------------------ | ------------------------------------------------------------ |
     | cartoonization | 图片生成推理模型。 | 请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/cartoonization/ATC_cartoonization_tf_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/cartoonization/ATC_cartoonization_tf_AE)目录中README.md下载原始模型章节的模型文件。 |

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

     > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型重新进行模型转换。

     2. 设置LD_LIBRARY_PATH环境变量。   
        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。  

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

     3. 执行以下命令使用atc命令进行模型转换。  

        **cd \$HOME/models/cartoonGAN_picture**   

        **atc --output_type=FP32 --input_shape="train_real_A:1,256,256,3" --input_format=NHWC --output="./cartoonization" --soc_version=Ascend310 --framework=3 --save_original_model=false --model="./cartoonization.pb" --precision_mode=allow_fp32_to_fp16**   

     4. 执行以下命令将转换好的模型复制到样例中model文件夹中。   

        **cp cartoonization.om $HOME/samples/cplusplus/contrib/cartoonGAN_picture/model/**


### 样例部署

1. 开发环境命令行中设置编译依赖的环境变量。

   基于开发环境与运行环境CPU架构是否相同，请仔细看下面的步骤：

   - 当开发环境与运行环境CPU架构相同时，执行以下命令导入环境变量。

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

        > - 如果是3.0.0版本，此处 **DDK_PATH** 环境变量中的 **x86_64-linux** 应修改为 **x86_64-linux_gcc7.3.0**。
        > - 可以在命令行中执行 **uname -a**，查看开发环境和运行环境的cpu架构。如果回显为x86_64，则为x86架构。如果回显为arm64，则为Arm架构。

   - 当开发环境与运行环境CPU架构不同时，执行以下命令导入环境变量。例如开发环境为X86架构，运行环境为Arm架构，由于开发环境上同时部署了X86和Arm架构的开发套件，后续编译应用时需要调用Arm架构开发套件的ACLlib库，所以此处需要导入环境变量为Arm架构的ACLlib库路径。

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

        > - 如果是3.0.0版本，此处 **DDK_PATH** 环境变量中的 **arm64-liunx** 应修改为 **arm64-linux_gcc7.3.0**。
        > - 可以在命令行中执行 **uname -a**，查看开发环境和运行环境的cpu架构。如果回显为x86_64，则为x86架构。如果回显为arm64，则为Arm架构。

2. 执行以下命令，执行编译脚本，开始样例编译。

   **cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/scripts**

   **bash sample_build.sh**



### 样例运行

1. <a name="step_2"></a>运行可执行文件。

   **bash sample_run.sh**

### 查看结果

运行完成后，会在运行环境的命令行中打印出推理结果,并在该工程目录的out/output目录下生成推理后的图片。