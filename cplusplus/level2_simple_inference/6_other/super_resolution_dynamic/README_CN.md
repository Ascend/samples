
**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.0.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 超分辨率图像处理样例

功能：使用FSRCNN对输入图片进行图像超分辨率处理,不同档位的分辨率（样例中分辨率档位分为256,256;512,512;288,288），在应用中加载该om文件，通过传参设置选择不同档位的分辨率进行推理，并将推理结果保存到文件中。

样例输入：原始图片bmp文件。

样例输出：带推理结果的png文件。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../environment)准备好环境。

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

    1.参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下该样例的model文件夹中，本例为：               $HOME/samples/level2_simple_inference/6_other/super_resolution_dynamic/model。

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

      |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
      |---|---|---|
      |  FSRCNN| 图片生成推理模型。  |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/super_resolution/ATC_FSRCNN_caffe_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/super_resolution/ATC_FSRCNN_caffe_AE)目录中README.md下载原始模型章节的模型文件。    |
  
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型重新进行模型转换。

    2. 设置LD_LIBRARY_PATH环境变量。   
    由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。
    
        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    3. 执行以下命令使用atc命令进行模型转换。  

        **cd $HOME/samples/cplusplus/level2_simple_inference/6_other/super_resolution_dynamic/model** 
    
    - 使用FSRCNN模型：     
        **atc --model=./FSRCNN.prototxt --weight=./FSRCNN.caffemodel --framework=0 --input_format=NCHW --input_shape="data: 1, 1, -1, -1" --dynamic_image_size="256,256;512,512;288,288" --output=./FSRCNN_x3 --soc_version=Ascend310 --output_type=FP32**
     
  

### 样例部署

 执行以下命令，执行编译脚本，开始样例编译。   
```cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/6_other/super_resolution_dynamic/scripts```    
```bash sample_build.sh```


 

### 样例运行
  

1. <a name="step_2"></a>执行运行脚本，开始样例运行。  
       
    ```bash sample_run.sh```

**注:也可以自己选择如下的命令执行，自主选择分辨率，可执行程序的第一个、第二个入参需要分别替换为实际的高、宽，必须为模型转换时通过--dynamic_image_size参数指定的其中一档分辨率**：

     ./main ../data 256 256
   
     ./main ../data 512 512 

     ./main ../data 288 288
 
    

### 查看结果

运行完成后，在$HOME/samples/cplusplus/level2_simple_inference/6_other/super_resolution_dynamic/output目录下生成推理后的图片。