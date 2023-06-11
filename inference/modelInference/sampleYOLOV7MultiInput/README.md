## 目录

  - [样例介绍](#样例介绍)
  - [样例流程图](#样例流程图)
  - [目录结构](#目录结构)
  - [获取源码包](#获取源码包) 
  - [第三方依赖安装](#第三方依赖安装)
  - [样例运行](#样例运行)
  - [其他资源](#其他资源)
  - [更新说明](#更新说明)
  - [已知issue](#已知issue)
    
## 样例介绍
功能：使用yolov7模型对输入数据进行预测推理，推理检测出图片/视频中所有可检测物体，并将推理结果打印到输出上，是一个是基于多路、多线程方案实现的高性能案例，通过多卡并行处理多路数的数据并输出，支持多种输入输出。    
样例输入：原始图片jpg文件/视频mp4文件/视频h26X文件/rtsp视频流。   
样例输出：带推理结果的图片/带推理结果的视频文件/rtsp视频流展示/cv::imshow窗口展示/打屏显示。 

## 样例流程图
通用目标检测与识别一站式方案是一个是基于多路、多线程方案实现的高性能案例，通过多卡并行处理多路数的数据并输出。
其中整体流程图如下所示：
![流程图](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/sampleYolov7MultiInput/%E6%B5%81%E7%A8%8B%E5%9B%BE.png)

- 管理线程：将线程和队列打包在一起，并完成进程创建、消息队列创建、消息发送和消息接收守护。
- 数据输入线程：对输入图片或视频进行解码。
- 数据预处理线程：对数据输入线程传过来的YUV图片进行处理（resize等操作）。
- 推理线程：使用YOLOV7模型进行推理。
- 数据后处理线程：分析推理结果，输出框点及标签信息。
- 数据输出线程：将框点及标签信息标识到输出数据上。


## 目录结构

```
├── model                      //模型文件夹，存放样例运行需要的模型文件
│   └── xxxx.onnx                 
├── data                       //数据文件夹
│   └── xxxx                   //测试数据,输入视频 
├── pic                        //数据文件夹
│   └── xxxx                   //测试数据,输入图片
├── inc                        //头文件文件夹
│   ├── Params.h               //声明样例使用的数据结构的头文件 
│   └── label.h                //声明样例模型使用的类别标签的头文件 
├── out                        //编译输出文件夹，存放编译生成的可执行文件
│   ├── xxxx                   //可执行文件 
│   └── xxxx                   //样例运行的输出结果图片/视频
├── scripts                    //配置文件+脚本文件夹
│   ├── test.json              //样例运行使用的参数配置文件 
│   ├── sample_build.sh        //快速编译脚本
│   ├── sample_run.sh          //快速运行脚本
│   ├── writingMethodForEachInputOutput.json        //不同输入输出数据类型参数配置参考文件
│   └── multiInputWithMultiModelPerDevice.json      //多device多推理线程多输入数据配置参考文件
├── src 
│   ├── acl.json               //系统初始化的配置文件 
│   ├── CMakeLists.txt         //Cmake编译文件
│   ├── dataInput              //数据输入解码处理线程文件夹，存放该业务线程的头文件及源码
│   ├── dataOutput             //数据输出处理线程文件夹，存放该业务线程的头文件及源码
│   ├── detectInference        //检测模型推理线程文件夹，存放该业务线程的头文件及源码
│   ├── detectPreprocess       //检测模型预处理线程文件夹，存放该业务线程的头文件及源码
│   ├── detectPostprocess      //检测模型后处理线程文件夹，存放该业务线程的头文件及源码
│   ├── pushrtsp               //rtsp展示线程文件夹，存放该业务线程的头文件及源码
│   └── main.cpp               //主函数，yolo检测功能的实现文件  
└── CMakeLists.txt             //编译脚本入口，调用src目录下的CMakeLists文件
```

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
- x264

    执行以下命令安装x264
   ```
   cd ${HOME}
   git clone https://code.videolan.org/videolan/x264.git
   cd x264
   # 安装x264
   ./configure --enable-shared --disable-asm
   make
   sudo make install
   sudo cp /usr/local/lib/libx264.so.164 /lib
   ```   

- ffmpeg

  执行以下命令安装ffmpeg
   ```
   cd ${HOME}
   wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
   tar -zxvf ffmpeg-4.1.3.tar.gz
   cd ffmpeg-4.1.3
   # 安装ffmpeg
   ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
   make -j8
   make install
   ```   
   
   </details> 

- opencv

  执行以下命令安装opencv(注:确保是3.x版本)
  ```
  sudo apt-get install libopencv-dev
  ```   

- jsoncpp

  命令行apt安装jsoncpp：

   ```
   # 安装完成后静态库在系统：/usr/include；动态库在：/usr/lib/x84_64-linux-gnu
   sudo apt-get install libjsoncpp-dev 
   sudo ln -s /usr/include/jsoncpp/json/ /usr/include/json
   ```

## 样例运行
   > 注：这里以一路MP4视频为输入，输出保存为离线mp4视频为例演示验证样例运行，关于样例运行所使用的更多相关参数配置说明请参考[样例参数配置说明](./configDemo.md)。

  - 数据准备

    请从以下链接获取该样例的输入视频，放在data目录下。
        
    ```    
    cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
    ```

  - ATC模型转换

    将yolov7原始模型转换为适配昇腾310处理器的离线模型（\*.om文件），放在model路径下。
   
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/model
    # 下载yolov7的原始模型文件及AIPP配置文件
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/yolov7/yolov7x.onnx --no-check-certificate
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/yolov7/aipp.cfg --no-check-certificate
    # 请使用与芯片名相对应的<soc_version>取值进行模型转换，然后再进行推理
    atc --model=yolov7x.onnx --framework=5 --output=yolov7x --input_shape="images:1,3,640,640"  --soc_version=Ascend310  --insert_op_conf=aipp.cfg
    ```

  - 样例编译

    执行以下命令，执行编译脚本，开始样例编译。
    ```
    cd $HOME/samples/inference/modelInference/sampleYOLOV7MultiInput/scripts
    bash sample_build.sh
    ```

  - 样例运行

    执行运行脚本，开始样例运行。
    ```
    bash sample_run.sh
    ```

  - 样例结果展示
    
    会根据scripts/test.json文件配置的结果输出方式输出应用的推理结果，包含物体的检测框信息。
    输出数据类型配置为video，输出数据存储在out文件夹下，为名称类似于：**XXXX.mp4** 的视频。

    执行成功后，样例将根据配置的输出数据类型，输出不同文件，结果框坐标信息等可能会根据版本、环境有所不同，请以实际情况为准。

    - 若输出数据类型配置为pic
      输出数据存储在out/文件夹下，为名称类似于**channel_X_out_pic_Y.jpg** 的图片，其中X代表第x路，Y代表第y张图片。

    - 若输出数据类型配置为video
      输出数据存储在out/output文件夹下，为名称类似于：**XXXX.mp4** 的视频，其中X代表第x路。

## 其他资源

以下资源提供了对通用目标识别样例的更多了解，包括如何进行定制开发和性能提升：

**Documentation**
- [通用目标识别样例](https://github.com/Ascend/samples/wikis/%E9%80%9A%E7%94%A8%E7%9B%AE%E6%A0%87%E8%AF%86%E5%88%AB%E6%A0%B7%E4%BE%8B/%E5%89%8D%E8%A8%80/%E6%A6%82%E8%BF%B0)
- [AscendCL Samples介绍](../README_CN.md)
- [使用AscendCLC API库开发深度神经网络应用](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha006/infacldevg/aclcppdevg/aclcppdevg_000000.html)
- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/05/16| 案例新增功能点：模型多batch、单路输入支持1-4路后处理，imshow输出展示 |
| 2023/05/16| 修改sampleYOLOV7MultiInput/README.md，新增样例配置文件说明configDemo.md |
| 2023/03/28| 新增sampleYOLOV7MultiInput/README.md |
  

## 已知issue

  暂无