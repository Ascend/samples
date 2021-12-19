**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**

## 检测视频中物体的样例

功能：使用人脸检测模型同时对两路视频流进行人脸检测。

样例输入：网络摄像头RTSP视频流或者mp4视频文件。

样例输出：presenter界面展现检测结果。

注意：本样例支持海康网络摄像头输出的RTSP视频流。对推流工具推送的视频流的支持有待完善


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
    |  face_detection| 图片分类推理模型。是基于Caffe的resnet ssd模型。 |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    
    cd ${HOME}/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp/model  
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/insert_op.cfg
    atc --model=./face_detection.prototxt --weight=./face_detection_fp32.caffemodel --framework=0 --output=./face_detection --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg
    ```
### 样例部署

执行以下命令，执行编译脚本，开始样例编译。   
```
cd ${HOME}/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp/scripts    
bash sample_build.sh
```

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **multi_channels_rtsp** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。    
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r ${HOME}/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser    
    ssh HwHiAiUser@xxx.xxx.xxx.xxx     
    cd ${HOME}/multi_channels_rtsp/scripts
    ```    

2. <a name="step_2"></a>执行运行脚本，开始样例运行。         
    ```
    bash sample_run.sh
    ```      

### 查看结果

1. 打开presentserver网页界面。

   - 使用产品为200DK开发者板。

      打开启动Presenter Server服务时提示的URL即可。
      
2. 等待Presenter Agent传输数据给服务端，单击“Refresh“刷新，当有数据时相应的Channel 的Status变成绿色。

3. 单击右侧对应的View Name链接，查看结果。

