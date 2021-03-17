**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.0.0及以上版本，支持产品为Atlas200DK。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**

## 人脸检测样例

**注：案例详细介绍请参见[人脸检测](https://gitee.com/ascend/samples/wikis/%E4%BA%BA%E8%84%B8%E6%A3%80%E6%B5%8B?sort_id=3170483)。**

功能：使用人脸检测模型对树莓摄像头中的即时视频进行人脸检测。

样例输入：摄像头视频。

样例输出：presenter界面展现检测结果。


### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../../environment)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。
### 工程准备

1. 非root用户进入运行环境，在命令行中执行以下命令下载源码仓。

   **cd $HOME**

   **git clone https://gitee.com/ascend/samples.git**

2. 获取此应用中所需要的原始网络模型。

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/face_detection_camera。
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  face_detection| 是基于Caffe的人脸检测模型。  |  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为Davinci模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export install_path=\\$HOME/Ascend/ascend-toolkit/latest**

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/face_detection_camera**  

        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection-python/insert_op.cfg**

        **atc --input_shape="data:1,3,300,300" --weight="./face_detection_fp32.caffemodel" --input_format=NCHW --output="./face_detection" --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --framework=0 --model="./face_detection.prototxt"**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./face_detection.om $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_camera/model/**      

### 样例部署

1. 修改present相关配置文件。

    将样例目录下**scripts/face_detection.conf**中的 presenter_server_ip、presenter_view_ip 修改为开发板的ip地址。

     使用产品为200DK开发者板。     
        1. 在开发环境中将**scripts/face_detection.conf**中的 presenter_server_ip、presenter_view_ip 修改为该ip地址。   
        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 1.开发环境和运行环境分离部署，一般使用配置的虚拟网卡ip，例如192.168.1.223。
        > - 2.开发环境和运行环境合一部署，一般使用200dk固定ip，例如192.168.1.2
### 样例运行

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
> - 以下出现的**xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2。

1. 执行以下命令,将开发环境的 **face_detection_camera** 目录上传到运行环境中，例如 **/home/HwHiAiUser**。   


    **scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_camera HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. 启动presenterserver并登录运行环境。

     使用产品为200DK开发者板。   
        1. 运行环境中执行以下命令启动presentserver。     
           执行以下命令登录运行环境。      
            **ssh HwHiAiUser@xxx.xxx.xxx.xxx**     
            **cd $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_camera**   
            **bash scripts/run_presenter_server.sh**   

3. <a name="step_2"></a>运行可执行文件。

      **cd $HOME/face_detection_camera/src**

    切换目录后，执行以下命令运行样例。

    **python3.6 main.py**

### 查看结果

1. 打开presentserver网页界面。

   使用产品为200DK开发者板。

      打开启动Presenter Server服务时提示的URL即可。

2. 等待Presenter Agent传输数据给服务端，单击“Refresh“刷新，当有数据时相应的Channel 的Status变成绿色。

3. 单击右侧对应的View Name链接，查看结果。