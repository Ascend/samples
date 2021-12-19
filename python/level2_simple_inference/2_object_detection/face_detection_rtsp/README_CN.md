**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**

## RTSP视频流人脸检测样例

功能：使用人脸检测模型对网络摄像头实时视频进行人脸检测。

样例输入：网路摄像头rtsp视频流。

样例输出：presenter界面展现检测结果。

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。

| 适配项     | 适配条件                                                     | 备注                                                         |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 适配版本   | >=5.0.4                                                    | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件   | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | python-acllite                                             | 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

> - 使用本地视频文件测试本样例时，要求视频文件为h264裸流文件，并且为annex-b格式。mp4转h264可以使用命令：
>
>   ffmpeg -i aaa.mp4 -codec copy -bsf: h264_mp4toannexb -f h264 aaa.h264
>
> - 在使用pyav读取视频切帧时对mp4文件有另外约束，可以参考[ffmpeg指令使MP4文件满足pyav和dvpp的约束](https://bbs.huaweicloud.com/forum/thread-131548-1-1.html)中的指令使mp4文件满足其约束

### 工程准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。   
    - 命令行方式下载（下载时间较长，但步骤简单）。
      ```    
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

2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  face_detection| 是基于Caffe的人脸检测模型。|  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp/model/    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel  
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection-python/insert_op.cfg
    atc --input_shape="data:1,3,300,300" --weight="./face_detection_fp32.caffemodel" --input_format=NCHW --output="./face_detection" --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --framework=0 --model="./face_detection.prototxt"
    ```

3. 获取样例需要的测试h264文件。  
    ```
    # 执行以下命令，进入样例的data文件夹中，下载对应的测试h264文件。
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp/data
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection_rtsp/person.h264
    cd ../src  
    ```

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 修改配置文件

   将文件中[videostream]段下添加要检测的视频文件或者rtsp地址. 本样例最多支持6路视频流

2. 执行以下命令,将开发环境的 **face_detection_rtsp** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
   ```
   # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
   scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx
   cd $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp/script
   ```

3. <a name="step_2"></a>运行可执行文件。
   ```
   bash sample_run.sh
   ```

### 查看结果

1. 打开presentserver网页界面。

   - 使用产品为200DK开发者板。

     打开启动Presenter Server服务时提示的URL即可。

   - 使用产品为300加速卡（ai1s云端推理环境）。

     **以300加速卡（ai1s）内网ip为192.168.0.194，公网ip为124.70.8.192举例说明。**

     启动Presenter Server服务时提示为Please visit [http://192.168.0.194:7009](http://192.168.0.194:7009/) for display server。

     只需要将URL中的内网ip：192.168.0.194替换为公网ip：124.70.8.192，则URL为 [http://124.70.8.192:7009。](http://124.70.8.192:7009。/)

     然后在windows下的浏览器中打开URL即可。

2. 等待Presenter Agent传输数据给服务端，单击“Refresh“刷新，当有数据时相应的Channel 的Status变成绿色。

3. 单击右侧对应的View Name链接，查看结果。