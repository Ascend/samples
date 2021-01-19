中文|[English](README_EN.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.1及以上版本，支持产品为Atlas200DK。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**

## 人脸识别样例

功能：通过摄像头对视频中的人脸信息进行预测，与已注册的人脸进行比对，预测出最可能的用户。

样例输入：摄像头。

样例输出：presenter界面展现推理结果。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../../../environment)准备好环境。

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

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/face_recognition_camera。
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  face_detection| 人脸检测网络模型。是基于Caffe的Resnet10-SSD300模型转换后的网络模型。  |  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |
    |  vanillacnn| 人脸特征点标记网络模型。是基于Caffe的VanillaCNN模型转换后的网络模型。  |  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/vanillacnn/ATC_vanillacnn_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/vanillacnn/ATC_vanillacnn_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |
    |  sphereface| 特征向量获取网络模型。是基于Caffe的SphereFace模型转换后的网络模型。|  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/sphereface/ATC_sphereface_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/sphereface/ATC_sphereface_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |
    
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为Davinci模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/face_recognition_camera**  

        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_recognition_camera/face_detection_insert_op.cfg**

        **atc --input_shape="data:1,3,300,300" --weight="./face_detection_fp32.caffemodel" --input_format=NCHW --output="./face_detection" --soc_version=Ascend310 --insert_op_conf=./face_detection_insert_op.cfg --framework=0 --model="./face_detection.prototxt"**

        按照同样的方式将vanillacnn，sphereface也进行模型转换。

        **atc --input_shape="data:4,3,40,40" --weight="./vanillacnn.caffemodel" --input_format=NCHW --output="./vanillacnn" --soc_version=Ascend310 --framework=0 --model="./vanilla_deploy.prototxt"**

        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_recognition_camera/sphereface_insert_op.cfg**

        **atc --input_shape="data:8,3,112,96" --weight="./sphereface.caffemodel" --input_format=NCHW --output="./sphereface" --soc_version=Ascend310 --insert_op_conf=./sphereface_insert_op.cfg --framework=0 --model="./sphereface.prototxt"**
    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./face_detection.om $HOME/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera/model/**

        **cp ./vanillacnn.om $HOME/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera/model/**

        **cp ./sphereface.om $HOME/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera/model/**


### 样例部署

1. 修改present相关配置文件。

    将样例目录下**script/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为开发环境中可以ping通运行环境的ip地址。   
    1. 开发环境中使用ifconfig查看可用ip。   
    2. 在开发环境中将**script/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为该ip地址。   
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - 1.开发环境和运行环境分离部署，一般使用配置的虚拟网卡ip，例如192.168.1.223。
    > - 2.开发环境和运行环境合一部署，一般使用200dk固定ip，例如192.168.1.2。

2. 开发环境命令行中设置编译依赖的环境变量。

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**  
 
     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**   
     
3. 切换到face_recognition_camera目录，创建目录用于存放编译文件，例如，本文中，创建的目录为 **build/intermediates/host**。

    **cd $HOME/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera**

    **mkdir -p build/intermediates/host**

4. 切换到 **build/intermediates/host** 目录，执行cmake生成编译文件。   
 
      **cd build/intermediates/host**   
      **make clean**   
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

5. 执行make命令，生成的可执行文件main在 **face_recognition_camera/out** 目录下。

    **make**


### 样例运行

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
> - 以下出现的**xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2。

1. 执行以下命令,将开发环境的 **face_recognition_camera** 目录上传到运行环境中，例如 **/home/HwHiAiUser**。   

    **开发环境与运行环境合一部署，请跳过此步骤！**   

    **scp -r $HOME/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera/ HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. 启动presenterserver并登录运行环境。
   
    1. 开发环境中执行以下命令启动presentserver。   
        **cd $HOME/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera/**   
        **bash script/run_presenter_server.sh**   
    2. 执行以下命令登录运行环境。   
        **开发环境与运行环境合一部署，请跳过此步骤！**   
        **ssh HwHiAiUser@xxx.xxx.xxx.xxx**      

   ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**      
   > - 当提示“Please input a absolute path to storage facial recognition data:“时，请输入MindStudio中存储人脸注册数据及解析数据，此路径MindStudio用户需要有读写权限，如果此路径不存在，脚本会自动创建。


3. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。   
      **export LD_LIBRARY_PATH=**   
      **source ~/.bashrc**     
      **cd $HOME/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera/out**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。   
      **cd $HOME/face_recognition_camera/out**

    切换目录后，执行以下命令运行样例。

    **./main**

### 查看结果

1. 打开presentserver网页界面，打开启动Presenter Server服务时提示的URL即可。

2. 等待Presenter Agent传输数据给服务端，单击“Refresh“刷新，当有数据时相应的Channel 的Status变成绿色。

3. 单击右侧对应的View Name链接，查看运行结果。