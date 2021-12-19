
**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 超分辨率图像处理样例

功能：使用FSRCNN对输入图片进行图像超分辨率处理,不同档位的分辨率（样例中分辨率档位分为256,256;512,512;288,288），在应用中加载该om文件，通过传参设置选择不同档位的分辨率进行推理，并将推理结果保存到文件中。

样例输入：原始图片bmp文件。

样例输出：带推理结果的png文件。

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | presentagent,ffmpeg+acllite| 请参考[第三方依赖安装指导（C++样例）](../../../environment)完成对应安装 |

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。   
    - 命令行方式下载（下载时间较长，但步骤简单）。
       ```    
       # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```   
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
       ``` 
        # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
        # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip
        ```
2. 模型转换。

      |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
      |---|---|---|
      |  FSRCNN| 图片生成推理模型。  |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/super_resolution/ATC_FSRCNN_caffe_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/super_resolution/ATC_FSRCNN_caffe_AE)目录中README.md下载原始模型章节的模型文件。    |
  
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    
    cd ${HOME}/samples/cplusplus/level2_simple_inference/6_other/super_resolution_dynamic/model   
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/super_resolution/FSRCNN/FSRCNN.caffemodel
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/super_resolution/FSRCNN/FSRCNN.prototxt
    atc --model=./FSRCNN.prototxt --weight=./FSRCNN.caffemodel --framework=0 --input_format=NCHW --input_shape="data: 1, 1, -1, -1" --dynamic_image_size="256,256;512,512;288,288" --output=./FSRCNN_x3 --soc_version=Ascend310 --output_type=FP32
    ```
### 样例部署

执行以下命令，执行编译脚本，开始样例编译。   
```
cd ${HOME}/samples/cplusplus/level2_simple_inference/6_other/super_resolution_dynamic/scripts    
bash sample_build.sh
```

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **super_resolution_dynamic** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。    
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r ${HOME}/samples/cplusplus/level2_simple_inference/6_other/super_resolution_dynamic HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser    
    ssh HwHiAiUser@xxx.xxx.xxx.xxx     
    cd ${HOME}/super_resolution_dynamic/scripts
    ```    

2. <a name="step_2"></a>执行运行脚本，开始样例运行。         
    ```
    bash sample_run.sh
    ```      

    **注:也可以自己选择如下的命令执行，自主选择分辨率，可执行程序的第一个、第二个入参需要分别替换为实际的高、宽，必须为模型转换时通过--dynamic_image_size参数指定的其中一档分辨率**：

     ./main ../data 256 256
   
     ./main ../data 512 512 

     ./main ../data 288 288
 
    

### 查看结果

运行完成后，会在样例工程的out/output目录下生成推理后的图片。