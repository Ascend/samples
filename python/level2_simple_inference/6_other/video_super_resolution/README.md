**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## EDVR视频超分辨率样例

功能：使用EDVR模型对输入视频进行超分辨率推理。

样例输入：待推理的低分辨率视频。

样例输出：推理后的超分辨率视频。   

### 适配要求

- 除基本环境搭建外，需要执行如下命令安装额外的第三方依赖
    
    ```
    apt-get install ffmpeg
    pip install imageio
    pip install opencv_python
    pip install ffmpeg_python
    ```
本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | opencv_python, ffmpeg_python, numpy, imageio| 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

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
        # 2. 将zip包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip    

2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  EDVR | 视频进行超分辨率推理模型。是基于Tensorflow的EDVR模型。  |  [https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/EDVR_180_320.pb](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/EDVR_180_320.pb)
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行 
    cd ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution/model     
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/EDVR_180_320.pb        
    atc --input_shape="L_input:1,5,180,320,3" --input_format=ND --output="EDVR_180_320" --soc_version=Ascend310 --framework=3 --model="./EDVR_180_320.pb" --output_type=FP32
    ```

3. 获取样例需要的测试视频。
    ```
    mkdir -p ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution/data
    cd ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution/data
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/low_resolution.mp4  
    ``` 

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **video_super_resolution** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/video_super_resolution/src
    ```

2. <a name="step_2"></a>运行工程。
    ```
    python3.6 video_super_resolution.py
    ```
### 查看结果

运行完成后，会在outputs目录下生成超分辨率视频。
