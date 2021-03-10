中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.1及以上版本，支持产品为Atlas200DK。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**

## 摄像头检测样例

功能：使用人脸检测模型对树莓摄像头中的即时视频进行人脸检测。

样例输入：摄像头视频。

样例输出：presenter server web页面现检测结果。


### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../../environment)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。 开发环境，非root用户命令行中执行以下命令下载源码仓。   
      
       ```
       cd $HOME
       git clone https://gitee.com/ascend/samples.git       
       ```
       
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。   
        
        2. 将ZIP包上传到开发环境中的普通用户home目录中，例如 **$HOME/ascend-samples-master.zip**。   
        
        3. 开发环境中，执行以下命令，解压zip包。   
           
            ```
            cd $HOME
            unzip ascend-samples-master.zip
            
            ```
            

2. 获取此应用中所需要的原始网络模型。

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的工程目录所在的model文件夹下，例如：$HOME/samples/cplusplus/level2_simple_inference/2_object_detection/face_detection_camera/model。

    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  face_detection| 图片分类推理模型。是基于Caffe的resnet ssd模型。 |  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为Davinci模型。
   
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        ```
        export install\_path=$HOME/Ascend/ascend-toolkit/latest
        export LD_LIBRARY_PATH=${install_path}/atc/lib64
        ```

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        ```
        cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/face_detection_camera/model
        
        wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection_camera/insert_op.cfg  
        
        atc --output_type=FP32 --input_shape="data:1,3,300,300" --weight=./face_detection_fp32.caffemodel --input_format=NCHW --output=./face_detection --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --framework=0 --save_original_model=false --model=./face_detection.prototxt
        ```
        
        ​    

### 样例部署

1. 修改样例配置文件scripts/face_detection.conf

    ```
    [baseconf]
    # A socket server address to communicate with presenter agent
    presenter_server_ip=127.0.0.1
    
    # The port of presenter agent and server communicate with
    presenter_server_port=7006
    
    #the ip in presenter server view web url
    presenter_view_ip=127.0.0.1
    
    #view entry label in presenter server view web
    channel_name=person
    
    #the data type that send to presenter server from agent, 0:image, 1:video
    content_type=1
    ```

    在开发环境上使用ifconfig查看ip；如果是在PC服务器上启动presenter server，将配置项presenter_server_ip、presenter_view_ip 修改为开发环境中可以ping通运行环境的ip地址，例如192.168.1.223; 如果是直接在Atlas200DK上启动persenter server，则这两项都使用Atlas200DK的固定IP，例如192.168.1.2；
    

2. 设置编译环境变量

     ```
     export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux 
     export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub   
     ```

     

3. 创建编译目录

    ```
    cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/face_detection_camera
    mkdir -p build/intermediates/host
    ```

    

4. 执行cmake命令

      ```
      cd build/intermediates/host  
      make clean   
      cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
      ```

      

5. 执行make命令

    ```
    make
    ```

    生成的可执行文件main在 face_detection_camera/out 目录下。

    

### 样例运行

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
> - 以下出现的**xxx.xxx.xxx.xxx**为运行环境ip，Atlas200DK在USB连接时一般为192.168.1.2。

1. 启动presenterserver。执行下述命令启动presenter server。注意：如果是在开发板上直接运行presenter server，则下面的命令在开发板上执行；如果是在PC服务器上运行presenter server，则是在PC服务器上执行下面的命令。

    ```
    cd $HOME/samples/common/  
    bash scripts/run_presenter_server.sh ../cplusplus/level2_simple_inference/2_object_detection/face_detection_camera/scripts/face_detection.conf   
    ```

    

2. 如果是在PC服务器上编译样例，则需要将编译完成的 face_detection_camera 目录上传到Atlas200DK开发板，例如 /home/HwHiAiUser目录下 

    ```
     scp -r $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/face_detection_camera HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ```

    直接在Atlas200DK上编译的场景不需要拷贝

    

3. 如果是在PC服务器上编译样例，则ssh登录开发板

    ```
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    ```

    

4. 如果在开发板上直接编译运行的场景，需要清除atc离线模型转换设置的环境变量，执行命令

    ```
    export LD_LIBRARY_PATH=   
    source ~/.bashrc    
    ```

    

5. 执行应用程序

    如果是在开发板上直接编译的场景，执行命令

    ```
    cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/face_detection_camera/out
    ./main
    ```

    如果是在PC服务器编译场景，执行命令

    ```
    cd $HOME/face_detection_camera/out
    ./main
    ```

    

### 查看结果

1. 打开presenter server网页界面,打开启动Presenter Server服务时提示的URL即可。

2. 单击网页“Refresh“刷新，页面的“person”链接变为绿色，表示页面已经和应用程序链接成功。

3. 单击“person”链接，查看结果。

