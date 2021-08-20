**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.0.0及以上版本，支持产品为Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## EDVR视频超分辨率样例


功能：使用EDVR模型对输入视频进行超分辨率推理。

样例输入：待推理的低分辨率视频。

样例输出：推理后的超分辨率视频。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../../environment)准备好环境。
- 在ubuntu下执行如下命令安装依赖包
    apt-get install ffmpeg
    pip install imageio
    pip install opencv_python
    pip install ffmpeg_python

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

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/video_super_resolution。
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  EDVR | 视频进行超分辨率推理模型。是基于Tensorflow的EDVR模型。  |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/video_super_resolution/ATC_EDVR_tf_AE/](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/video_super_resolution/ATC_EDVR_tf_AE/)目录中README.md下载原始模型章节下载模型和权重文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为om模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。    

        **export install_path=$HOME/Ascend/ascend-toolkit/latest**     
        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/video_super_resolution**  


        **atc --input_shape="L_input:1,5,180,320,3" --input_format=ND --output="EDVR_180_320" --soc_version=Ascend310 --framework=3 --model="./EDVR_180_320.pb" --output_type=FP32**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./EDVR_180_320.om $HOME/samples/python/level2_simple_inference/6_other/video_super_resolution/model/**

4. 获取样例需要的测试视频。

    执行以下命令，进入样例的data文件夹中，下载对应的测试视频。

    **cd $HOME/samples/python/level2_simple_inference/6_other/video_super_resolution/data**

    **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/low_resolution.mp4**



### 样例运行

**注：目前该样例仅支持在Atlas300上运行。**       

1. 执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/python/level2_simple_inference/6_other/video_super_resolution/src**
     

    切换目录后，执行以下命令运行样例。

      **python3.6 video_super_resolution.py**
### 查看结果

运行完成后，会在outputs目录下生成超分辨率视频。
