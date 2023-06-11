## 目录

  - [样例介绍](#样例介绍)
  - [获取源码包](#获取源码包) 
  - [第三方依赖安装](#第三方依赖安装)
  - [样例运行](#样例运行)
  - [其他资源](#其他资源)
  - [更新说明](#更新说明)
  - [已知issue](#已知issue)
    
## 样例介绍

以YOLOV7网络模型为例，使能Acllite对图片进行预处理，并通过模型转换使能静态AIPP功能，使能AIPP功能后，YUV420SP_U8格式图片转化为RGB，然后减均值和归一化操作，并将该信息固化到转换后的离线模型中，对YOLOV7网络执行推理，对图片进行物体检测和分类，并给出标定框和类别置信度。
  
样例输入：图片。    
样例输出：图片物体检测，并且在图片上给出物体标注框，类别以及置信度。

## 获取源码包
    
 可以使用以下两种方式下载，请选择其中一种进行源码准备。

 - 命令行方式下载（下载时间较长，但步骤简单）。

   ```    
   # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
   cd ${HOME}     
   git clone https://github.com/Ascend/samples.git
   ```
   **注：如果需要切换到其它tag版本，以v0.5.0为例，可执行以下命令。**
   ```
   git checkout v0.5.0
   ```   
 - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
   **注：如果需要下载其它版本代码，请先请根据前置条件说明进行samples仓分支切换。**   
   ``` 
   # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
   # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
   # 3. 开发环境中，执行以下命令，解压zip包。     
   cd ${HOME}    
   unzip ascend-samples-master.zip
   ```

## 第三方依赖安装


设置环境变量，配置程序编译依赖的头文件，库文件路径。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

   ```
    export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
    export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
    export THIRDPART_PATH=${DDK_PATH}/thirdpart
    export LD_LIBRARY_PATH=${THIRDPART_PATH}/lib:$LD_LIBRARY_PATH
   ```
   创建THIRDPART_PATH路径

   ```
    mkdir -p ${THIRDPART_PATH}
   ```
- acllite

    注：源码安装ffmpeg主要是为了acllite库的安装
    执行以下命令安装x264

    ```
    # 下载x264
    cd ${HOME}
    git clone https://code.videolan.org/videolan/x264.git
    cd x264
    # 安装x264
    ./configure --enable-shared --disable-asm
    make
    sudo make install
    sudo cp /usr/local/lib/libx264.so.164 /lib
    ```   
    执行以下命令安装ffmpeg

    ```
    # 下载ffmpeg
    cd ${HOME}
    wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
    tar -zxvf ffmpeg-4.1.3.tar.gz
    cd ffmpeg-4.1.3
    # 安装ffmpeg
    ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
    make -j8
    make install
    ```   
   执行以下命令安装acllite

    ```
    cd ${HOME}/samples/inference/acllite/aclliteCPP
    make
    make install
    ```   
    </details> 

- opencv

  执行以下命令安装opencv(注:确保是3.x版本)
  ```
  sudo apt-get install libopencv-dev
  ```   

## 样例运行

  - 数据准备

    请从以下链接获取该样例的输入图片，放在data目录下。
        
    ```    
    cd $HOME/samples/inference/modelInference/sampleYOLOV7/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg
    ```

  - ATC模型转换

    将YOLOV7原始模型转换为适配昇腾310处理器的离线模型（\*.om文件），放在model路径下。

    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd $HOME/samples/inference/modelInference/sampleYOLOV7/model
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/yolov7/yolov7x.onnx
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/yolov7/aipp.cfg
    atc --model=yolov7x.onnx --framework=5 --output=yolov7x --input_shape="images:1,3,640,640"  --soc_version=Ascend310  --insert_op_conf=aipp.cfg
    ```

  - 样例编译

    执行以下命令，执行编译脚本，开始样例编译。
    ```
    cd $HOME/samples/inference/modelInference/sampleYOLOV7/scripts
    bash sample_build.sh
    ```
  - 样例运行

    执行运行脚本，开始样例运行。
    ```
    bash sample_run.sh
    ```
  - 样例结果展示
    
   运行完成后，会在样例工程的out目录下生成推理后的图片，显示对比结果如下所示。
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/yolov7/out_dog.jpg "image-20211028101534905.png")

## 其他资源

以下资源提供了对ONNX项目和YOLOV7模型的更深入理解：

**ONNX**
- [GitHub: ONNX](https://github.com/onnx/onnx)

**Models**
- [YOLOV7 - object detect](https://github.com/Ascend/modelzoo-GPL/tree/master/built-in/ACL_Pytorch/Yolov7_for_Pytorch)

**Documentation**
- [AscendCL Samples介绍](../README_CN.md)
- [使用AscendCLC API库开发深度神经网络应用](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha006/infacldevg/aclcppdevg/aclcppdevg_000000.html)
- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/03/07 | 新增sampleYOLOV7/README.md |
  

## 已知issue

  暂无
