# 前言

### 概述

本文档详细描述了通用目标识别的代码流程，编译运行方法，以及如何基于现有样例代码进行功能的定制、扩展，开发自己的AI应用程序。

### 读者对象

本文档适用于基于CANN进行AI应用开发的人员，通过本文档您可以达成：

- 掌握基于CANN进行目标识别相关应用开发的流程。
- 能够基于通用样例进行应用的定制与扩展。
- 能够进行模型的替换，若有CANN不支持的算子，能够进行自定义算子的开发。

掌握以下经验和技能可以更好地理解本文档：

- 具备C++/C语言程序开发能力
- 对机器学习、深度学习有一定的了解
- 若您需要进行自定义算子的开发，还需要对TVM及开源TensorFlow/Caffe/PyTorch框架有一定的了解；对数学表达式有一定的了解。



# 入门学习

### 昇腾产品形态说明

以昇腾 AI 处理器的PCIe的工作模式进行区分，如果PCIe工作在主模式，可以扩展外设，则称为RC模式；如果PCIe工作在从模式，则称为EP模式。  

- 昇腾 AI 处理器的工作模式如下：
  − 昇腾310 AI处理器有EP和RC两种模式。
  − 昇腾310P AI处理器只有EP模式。
  − 昇腾910 AI处理器只有EP模式。

- 支持RC模式的产品有：Atlas 200 AI加速模块、Atlas 200 DK 开发者套件。
  产品的CPU直接运行用户指定的AI业务软件，接入网络摄像头、I2C传感器、SPI显示器等其他外挂设备作为从设备接入产品。
- 支持EP模式的产品
  昇腾310 AI处理器：Atlas 200 AI加速模块、Atlas 300I 推理卡、Atlas 500 智能小站、Atlas 500 Pro 智能边缘服务器、Atlas 800 推理服务器。
  昇腾310P AI处理器：Atlas 300I Pro 推理卡、Atlas 300V Pro 视频解析卡。
  昇腾910 AI处理器：Atlas 800 训练服务器、Atlas 300T 训练卡。

EP模式通常由Host侧作为主端，Device侧作为从端。客户的AI业务程序运行在Host系统中，产品作为Device系统以PCIe从设备接入Host系统，Host系统通过PCIe通道与Device系统交互，将AI任务加载到Device侧的昇腾 AI 处理器中运行。

两种模式的产品及架构如图1所示。

Host和Device的概念说明如下：

- Host：是指与昇腾AI处理器所在硬件设备相连接的X86服务器、ARM服务器，利用昇腾AI处理器提供的NN（Neural-Network）计算能力完成业务。
- Device：是指安装了昇腾AI处理器的硬件设备，利用PCIe接口与服务器连接，为服务器提供NN计算能力。

图1RC和EP模式

![输入图片说明](https://images.gitee.com/uploads/images/2022/0217/154812_7f97b90d_5400693.png "屏幕截图.png")

### CANN逻辑架构

图2逻辑架构

![输入图片说明](https://images.gitee.com/uploads/images/2022/0217/154918_d5412a59_5400693.png "屏幕截图.png")

1. 昇腾计算语言接口
   昇腾计算语言（Ascend Computing Language，AscendCL）接口是昇腾计算开放编程框架，是对低层昇腾计算服务接口的封装。它提供Device（设备）管理、Context（上下文）管理、Stream（流）管理、内存管理、模型加载与执行、算子加载与执行、媒体数据处理、Graph（图）管理等API库，供用户开发人工智能应用调用。
2. 昇腾计算服务层
   本层主要提供昇腾计算库，例如神经网络（Neural Network，NN）库、线性代数计算库（Basic Linear Algebra Subprograms，BLAS）等；昇腾计算调优引擎库，例如算子调优、子图调优、梯度调优、模型压缩以及AI框架适配器。
3. 昇腾计算编译引擎
   本层主要提供图编译器（Graph Compiler）和TBE算子开发支持。前者将用户输入中间表达（Intermediate Representation，IR）的计算图编译成NPU运行的模型。后者提供用户开发自定义算子所需的工具。
4. 昇腾计算执行引擎
   本层负责模型和算子的执行，提供如运行时（Runtime）库（执行内存分配、模型管理、数据收发等）、图执行器（Graph Executor）、数字视觉预处理（Digital Vision Pre-Processing，DVPP）、人工智能预处理（Artificial Intelligence Pre-Processing，AIPP）、华为集合通信库（Huawei Collective Communication Library，HCCL）等功能单元。
5. 昇腾计算基础层
   本层主要为其上各层提供基础服务，如共享虚拟内存（Shared Virtual Memory，SVM）、设备虚拟化（Virtual Machine，VM）、主机-设备通信（Host Device Communication，HDC）等。



# 样例介绍

### 简介

目标识别是计算机视觉领域中的一项关键技术，随着深度学习技术的发展，目标识别的应用场景也越来越广泛。当前, 目标识别主要有以下几个应用场景:

- 安全领域：指纹识别、物体识别等。
- 交通领域：车牌号识别、无人驾驶、交通标志识别等。
- 医疗领域：心电图、B超、健康管理、营养学等。
- 生活领域：智能家居、智能购物、智能测肤等。

为了提升开发者基于CANN进行目标识别相关应用程序的开发效率，降低开发门槛，本样例提供了通用目标识别应用代码，支持对图片，离线视频以及RTSP视频流进行识别，开发者可以方便的基于此样例进行扩展，定制自己的AI应用。

### 应用流程

本样例基于CANN，实现了在昇腾AI处理器上对输入图片或者视频进行目标识别，通用业务流程如下所示：

![输入图片说明](https://images.gitee.com/uploads/images/2022/0217/155208_63329a39_5400693.png "屏幕截图.png")

- AscendCL初始化
  调用aclInit接口实现初始化AscendCL。
- 运行管理资源申请
  依次申请运行管理资源Device、Context与Stream，确保可以使用这些资源执行运算、管理任务。在申请管理资源时，样例进行了当前昇腾AI处理器运行模式的判断，做到不同形态的昇腾AI处理器的代码实现归一。
- 加载模型文件
  加载模型文件，并构建模型的输入与输出。
- 数据预处理。
  针对不同类型的输入，分别进行不同的处理，使其满足模型对输入数据的要求。
- 模型推理
  执行模型推理，并获取输出结果。
- 推理结果后处理
  根据用户选择的后处理方式，对输出结果进行不同的处理，例如对推理结果进行标注，输出为离线视频文件，或通过网页在线展示等。


### 获取样例

单击Gitee或Github，进入Samples开源仓，按照Samples仓根目录下的README中的"版本说明"，获取配套的Samples版本.

### 目录结构

```
├── model                      //模型文件夹，存放样例运行需要的模型文件
│   └── xxxx.pb                 
├── data                       //数据文件夹
│   └── xxxx                   //测试数据,输入图片/视频 
├── inc                        //头文件文件夹
│   └── CarParams.h            //声明样例使用的数据结构的头文件 
├── out                        //编译输出文件夹，存放编译生成的可执行文件
│   ├── xxxx                   //可执行文件 
│   └── output                 //结果输出文件夹（如果不存在需要自行创建）
│       └── xxxx               //样例运行的输出结果
├── display                    //网页展示功能实现代码文件夹
│   ├── presenterserver        //presenterserver文件夹
│   └── run_presenter_server.sh//presenterserver启动脚本
├── scripts                    //配置文件+脚本文件夹
│   ├── params.conf            //样例运行配置文件 
│   ├── present_start.conf     //presentserver启动配置文件 
│   ├── sample_build.sh        //快速编译脚本
│   └── sample_run.sh          //快速运行脚本
├── src 
│   ├── acl.json               //系统初始化的配置文件 
│   ├── CMakeLists.txt         //Cmake编译文件
│   ├── classifyPostprocess    //分类模型后处理线程文件夹，存放该业务线程的头文件及源码
│   ├── classifyPreprocess     //分类模型预处理线程文件夹，存放该业务线程的头文件及源码
│   ├── detectPostprocess      //检测模型后处理线程文件夹，存放该业务线程的头文件及源码
│   ├── detectPreprocess       //检测模型预处理线程文件夹，存放该业务线程的头文件及源码
│   ├── inference              //预处理线程文件夹，存放该业务线程的头文件及源码
│   ├── presentagentDisplay    //网页展示线程文件夹，存放该业务线程的头文件及源码
│   └── main.cpp               //主函数，图片分类功能的实现文件  
└── CMakeLists.txt             //编译脚本入口，调用src目录下的CMakeLists文件
```

# 环境准备

## 硬件和操作系统要求

本样例已在满足如下条件的硬件环境中进行测试，若环境不符合如下要求，样例可能运行失败

| 产品型号                             | 支持的操作系统             |
| ------------------------------------ | -------------------------- |
| Atlas 200 DK 开发者套件（型号 3000） | Ubuntu 18.04               |
| Atlas 300I Pro 推理卡                | Ubuntu 18.04 /  CentOS 7.6 |

## CANN软件要求

本样例支持的CANN版本为：5.0.4.alpha001及以上版本，CANN软件的获取请参见[昇腾社区软件下载](https://www.hiascend.com/software/cann/community)，CANN软件的安装请参见[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-develope)的“CANN软件安装”。
请注意，安装CANN软件前，需要已完成驱动和固件的安装。


 **说明:** 
准备好基础CANN环境后，您需要参见如下内容完成环境变量配置、公共库文件准备、文件目录创建、依赖安装等，进行这些准备前，您需要了解两个基本概念：开发环境与运行环境，以便更好的理解后续的操作：

- 开发环境指编译开发代码的环境，运行环境指运行推理程序的环境，运行环境必须带昇腾AI处理器。
- 开发环境与运行环境合设场景指带昇腾AI处理器的机器既作为开发环境又作为运行环境，此种场景下，代码开发与代码运行在同一台机器上。
- 开发环境与运行环境分设场景指开发者使用其他独立机器进行代码开发与编译，而不使用带有昇腾AI处理器的机器。

## 环境变量配置及其他准备

### 开发环境与运行环境合设场景

  1. 配置环境变量。

     - 以安装用户在任意目录下执行以下命令，打开.bashrc文件。

       `vi ~/.bashrc`  

     - 在文件最后一行后面添加如下内容。

       ```
       export CPU_ARCH=`arch`  # 使用arch命令自动获取当前操作系统架构
       export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  # 存储第三方库文件的路径，例如依赖安装中的OpenCV、FFmpeg等
       export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH  # 运行时链接库文件
       export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest # CANN软件安装后文件存储路径，若是root用户安装，$HOME请替换为/usr/local
       ```

     - 执行命令保存文件并退出。

       `:wq!`  

     - 执行命令使其立即生效。

       `source ~/.bashrc`

  2. 创建第三方依赖文件夹。

     `mkdir -p ${THIRDPART_PATH}`

  3. 下载samples仓源码。

     ```
     cd ${HOME}      # 此处以将samples源码仓下载到用户家目录为例，开发者可自定义下载路径
     sudo apt-get install git
     git clone https://github.com/Ascend/samples.git
     ```

  4. 将samples源码仓中的公共库拷贝到前面创建的第三方依赖文件夹中。

     `cp -r ${HOME}/samples/common ${THIRDPART_PATH}`

### 开发环境与运行环境分设场景

 需要分别在开发环境与运行环境上进行如下准备工作。

  - 开发环境

    1. 配置环境变量。

       - 以安装用户在任意目录下执行以下命令，打开.bashrc文件。

         `vi ~/.bashrc`  

       - 在文件后一行后面添加如下内容。

         ```
         # 配置为运行环境的操作系统架构，取值为aarch64或者x86_64
         export CPU_ARCH=[aarch64/x86_64]
         # 存储第三方库文件的路径，例如依赖安装中的OpenCV、FFmpeg等
         export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}
         # CANN软件安装后文件存储路径，最后一级请根据运行环境的操作系统架构设置，运行环境架构为AArch64，这里填arm64-linux；运行环境为X86，则这里填x86_64-linux，此处以运行环境架构为AArch64为例
         export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest/arm64-linux
         ```

       - 执行命令保存文件并退出。

         `:wq!`  

       - 执行命令使其立即生效。

         `source ~/.bashrc`

    2. 创建第三方依赖文件夹。

       `mkdir -p ${THIRDPART_PATH}`

    3. 下载samples仓源码。

       ```
       cd ${HOME}      # 此处以将samples源码仓下载到用户家目录为例，开发者可自定义下载路径
       sudo apt-get install git
       git clone https://github.com/Ascend/samples.git
       ```

    4. 将samples源码仓中的公共库拷贝到前面创建的第三方依赖文件夹中。

       `cp -r ${HOME}/samples/common ${THIRDPART_PATH}`

  - 运行环境

    1. 配置环境变量。

       - 以安装用户在任意目录下执行以下命令，打开.bashrc文件。

         `vi ~/.bashrc`  

       - 在文件后一行后面添加如下内容。

         ```
         export CPU_ARCH=`arch`  # 使用arch命令自动获取当前操作系统架构
         export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  # 存储第三方库文件的路径，例如依赖安装中的OpenCV、FFmpeg等
         export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH  # 运行时链接库文件
         export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest # CANN软件安装后文件存储路径
         ```

        - 执行命令保存文件并退出。

          `:wq!`  

        - 执行命令使其立即生效。 

          `source ~/.bashrc`

    2. 创建第三方依赖文件夹，作为后续依赖的安装路径。

       `mkdir -p ${THIRDPART_PATH}`

## 依赖安装

### OpenCV

本样例使用OpenCV接口的作用是进行输入视频文件的读取，以及所有类型输出数据的后处理，数据后处理包括目标标注、不同类型数据的输出，为此必选依赖。

- 开发环境与运行环境合设场景

  在环境上执行如下命令安装OpenCV：

  `sudo apt-get install libopencv-dev`

- 开发环境与运行环境分设场景

  - 若开发环境与运行环境操作系统架构相同，请分别在开发环境与运行环境上执行如下命令安装OpenCV:

    `sudo apt-get install libopencv-dev`

  - 若开发环境与运行环境操作系统架构不同，例如开发环境架构为X86，运行环境架构为AArch64，此种场景下，需要在开发环境上通过源码的方式对OpenCV进行交叉编译、安装，但此种方式操作复杂，所以此处采用直接在运行环境上安装opencv，然后将安装后的文件拷贝到开发环境的方式，详细步骤如下：

    1. 在运行环境上执行如下命令安装OpenCV，需要确保运行环境已接入互联网。

       `sudo apt-get install libopencv-dev`

    2. 在开发环境上执行如下命令，拷贝运行环境上对应的库文件。

       ```
       # 将运行环境下AArch64形态的opencv相关文件拷贝到开发环境（X86架构）的aarch64-linux-gnu目录，不会对本地开发环境本身使用产生任何影响。以下命令在开发环境中执行
       cd /usr/lib/aarch64-linux-gnu
       # 拷贝OpenCV相关库文件，其中X.X.X.X为运行环境IP地址，HwHiAiUser为运行环境的运行用户，此处仅为示例。
       sudo scp -r HwHiAiUser@X.X.X.X:/lib/aarch64-linux-gnu/* ./
       sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/aarch64-linux-gnu/* ./
       sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/*.so.* ./
       # 拷贝opencv相关头文件。
       sudo scp -r HwHiAiUser@X.X.X.X:/usr/include/opencv* /usr/include
       ```

### FFmpeg

本样例中，FFmpeg的作用是在输入数据类型为RTSP视频流或者离线视频的情况下，进行数据切帧的操作，如果您的业务不包含输入时RTSP视频流或者离线视频的场景，该第三方库实际上并不会被调用，可以不安装此依赖。

- 开发环境与运行环境合设场景

  在环境上参考如下命令使用源码编译的方式安装FFmpeg：

   ```
   # 下载并解压缩FFmpeg安装包，此处以将FFmpeg安装包存储在用户家目录下为例，开发者也可以自定义FFmpeg安装包存储路径。
   cd ${HOME}
   wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
   tar -zxvf ffmpeg-4.1.3.tar.gz
   cd ffmpeg-4.1.3
   # 安装ffmpeg
   ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --prefix=${THIRDPART_PATH}
   make -j8
   make install
   ```

- 开发环境与运行环境分设场景

  此种场景下，需要在开发环境上以源码的方式安装FFmpeg，详细步骤如下所示：

  1. 下载FFmpeg。

     ```
     # 下载并解压缩FFmpeg安装包，此处以将FFmpeg安装包存储在用户家目录下为例，开发者也可以自定义FFmpeg安装包存储路径。
     cd ${HOME}
     wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
     tar -zxvf ffmpeg-4.1.3.tar.gz
     cd ffmpeg-4.1.3
     ```

  2. 安装FFmpeg。

     - 若运行环境的操作系统架构为X86，在开发环境上执行如下命令安装FFmpeg。

       ```
        ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --prefix=${THIRDPART_PATH}
        make -j8
        make install
       ```

     - 若运行环境的操作系统架构为AArch64，在开发环境上执行如下命令安装FFmpeg。

       ```
       ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --cross-prefix=aarch64-linux-gnu- --enable-cross-compile --    arch=aarch64 --target-os=linux --prefix=${THIRDPART_PATH}
       make -j8
       make install
       ```

### PresentAgent

PresentAgent是为了将带有推理结果的图片数据发送到网页进行显示，如果您的业务场景不存在需要在网页观察推理结果的情况，可以不安装PresentAgent。由于PresentAgent依赖Protobuf，所以若需要使用PresentAgent，也要要同步安装Protobuf。

- 开发环境与运行环境合设场景

  在环境上参考如下命令使用源码方式安装Protobuf以及PresentAgent：

   ```
    # 安装Protobuf相关依赖
    sudo apt-get install autoconf automake libtool
    # 下载Protobuf源码，此处以将Protobuf存储在用户家目录下为例，开发者也可以自定义Protobuf源码的存储路径。
    cd ${HOME}
    git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
    # 编译安装Protobuf
    cd protobuf
    ./autogen.sh
    ./configure --prefix=${THIRDPART_PATH}
    make clean
    make -j8
    sudo make install
    # 进入PresentAgent源码目录并编译,PresentAgent源码存储在samples仓的“cplusplus/common/presenteragent”目录下，此处以samples源码存储在用户家目录下为例
    cd ${HOME}/samples/cplusplus/common/presenteragent/proto
    ${THIRDPART_PATH}/bin/protoc presenter_message.proto --cpp_out=./
    # 编译安装Presentagnet
    cd ..
    make -j8
    make install
   ```

- 开发环境与运行环境分设场景

  此种场景下，需要在开发环境上以源码的方式安装Protobuf以及PresentAgent，详细步骤如下所示：

  1. 安装Protobuf相关依赖。

     ```
     # 安装protobuf相关依赖
     sudo apt-get install autoconf automake libtool 
     # 安装pip3
     sudo apt-get install python3-pip 
     # 安装presentserver启动所需要的python库。若安装失败，请自行更换python源。
     python3.6 -m pip install --upgrade pip --user
     python3.6 -m pip install tornado==5.1.0 protobuf Cython numpy --user
     python3.7 -m pip install tornado==5.1.0 protobuf Cython numpy --user
     ```

  2. 安装Protobuf。

     - 若运行环境的操作系统架构为X86，在开发环境上执行如下命令安装Protobuf。

       ```
       # 下载protobuf源码
       cd ${HOME}
       git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
       # 编译安装protobuf
       cd protobuf
       ./autogen.sh
       ./configure --prefix=${THIRDPART_PATH}
       make -j8
       sudo make install
       ```

     - 若运行环境的操作系统架构为AArch64，在开发环境上执行如下命令安装Protobuf。

       ```
       # 下载protobuf源码
       cd ${HOME}
       git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
       cp -r protobuf protobuf_arm
       # 首次编译安装protobuf，生成x86架构的protoc文件
       cd protobuf
       ./autogen.sh
       ./configure
       make -j8
       sudo make install
       cd $HOME/protobuf_arm
       ./autogen.sh
       ./configure --build=x86_64-linux-gnu --host=aarch64-linux-gnu --with-protoc=protoc --prefix=${THIRDPART_PATH}
       make -j8
       make install
       ```

  3. 生成PresentAgent的proto文件，并安装PresentAgent。

     ```
     # 进入PresentAgent的源码目录，并生成对应的proto文件
     cd $HOME/samples/cplusplus/common/presenteragent/proto
     sudo ldconfig
     protoc presenter_message.proto --cpp_out=./
     # 安装presenteragent
     cd ..
     make -j8
     make install
     # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
     sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
     ```

### AclLite

AclLite库是对AscendCL DVPP图像和视频处理相关接口，AscendCL设备管理、资源管理、模型推理等接口进行了封装，旨在为用户提供一组更简易的公共接口。本样例是基于AclLite接口进行的开发，所以需要下载并编译安装AclLite库。

- 开发环境与运行环境合设场景

  1. 获取samples仓源码包

     此处已将samples仓下载到$HOME路径下为例，若之前步骤已经下载过此源码包，则此处无需重复下载，直接进行第2个步骤即可。
     可以使用以下两种方式下载，请选择其中一种即可   

      - 命令行下载

        ```       
        cd ${HOME}     
        git clone https://github.com/Ascend/samples.git
        ```

      - 压缩包下载   

        ``` 
         # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
         # 2. 将ZIP包上传到普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
         # 3. 执行以下命令，解压zip包。     
         cd ${HOME}    
         unzip ascend-samples-master.zip
        ```

  2.进入acllite目录

  ```
  cd ${HOME}/samples/cplusplus/common/acllite
  ```

  3.执行编译安装命令。

  ```
  make 
  make install
  ```

  安装完成后，编译生成的libacllite.so会被拷贝到`${THIRDPART_PATH}/lib`路径下；头文件会被拷贝到`${THIRDPART_PATH}/include/acllite`路径。  

   **须知：若开发者定制了AclLite库中代码，则需要重新编译。** 


- 开发环境与运行环境分设场景

  1. 获取samples仓源码包   

     此处已将samples仓下载到$HOME路径下为例，若之前步骤已经下载过此源码包，则此处无需重复下载，直接进行第2个步骤即可。
     可以使用以下两种方式下载，请选择其中一种即可   

      - 命令行下载

        ```    
        # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
        cd ${HOME}     
        git clone https://github.com/Ascend/samples.git
        ```

      - 压缩包下载   

        ``` 
         # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
         # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
         # 3. 开发环境中，执行以下命令，解压zip包。     
         cd ${HOME}    
         unzip ascend-samples-master.zip
        ```

  2. 进入acllite目录

    ```
    cd ${HOME}/samples/cplusplus/common/acllite
    ```

  3. 执行编译安装命令。

    ```
    make 
    make install
    ```

    安装完成后，编译生成的libacllite.so会被拷贝到开发环境的`${THIRDPART_PATH}/lib`路径下；头文件会拷贝到开发环境的`${THIRDPART_PATH}/include/acllite`路径。

  4. 运行环境库文件部署。

     a. 将开发环境中的libacllite.so拷贝到运行环境的`${THIRDPART_PATH}/lib`路径。

     b. 在运行环境下切换到 root用户，打开`/etc/ld.so.conf.d/mind_so.conf` ，将`${THIRDPART_PATH}/lib`追加到文件末尾，保存后退出，执行命令ldconfig。 
     **须知：若开发者定制了AclLite库中代码，则需要重新编译，并重新将库文件拷贝到运行环境。** 


# 模型及数据准备

样例运行前，请参见本章节准备样例依赖的模型文件及测试数据文件。

### 准备模型

#### <a name="model-list">模型列表</a>

| **模型名称** | **模型说明**                                      | **模型详细描述**                                             |
| ------------ | ------------------------------------------------- | ------------------------------------------------------------ |
| yolov3       | 图片检测推理模型。是基于onnx的Yolov3模型。        | 模型详细描述请参见[https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov/ATC_yolov3_onnx_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov/ATC_yolov3_onnx_AE)。您可以参见readme中的“原始模型”章节下载原始模型网络文件和配置文件，也可以直接参见下方的[模型转换](#model_convert)章节使用wget命令下载。 |
| carcolor        | 车辆颜色分类推理模型。是基于tensorflow的CNN模型。 | 模型详细描述请参见[https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/ATC_CarColor_tensorflow_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/ATC_CarColor_tensorflow_AE)。您可以参见readme中的“原始模型”章节下载原始模型网络文件和配置文件，也可以直接参见下方的[模型转换](#model_convert)章节使用wget命令下载。 |

#### <a name="model_convert">模型转换</a>

需要将下载的原始模型转换为适配昇腾AI处理器的离线om模型，并放置到样例代码中的“model”目录下。

为方便操作，此处直接给出了原始模型的下载命令以及模型转换命令，可直接拷贝执行。当然，您也可以参见[模型列表](#model-list)中的下载地址中对应的README进行手工操作，并了解更多细节。

```
# 进入目标识别样例工程根目录
cd $HOME/samples/cplusplus/level3_application/1_cv/detect_and_classify
# 创建并进入model目录
mkdir model
cd model
# 下载yolov3的原始模型文件及AIPP配置文件
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/yolov3_t.onnx
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/aipp_onnx.cfg
# 执行模型转换命令，生成yolov3的适配昇腾AI处理器的离线模型文件
atc --model=./yolov3_t.onnx --framework=5 --output=yolov3 --input_shape="images:1,3,416,416;img_info:1,4" --soc_version=Ascend310 --input_fp16_nodes="img_info" --insert_op_conf=aipp_onnx.cfg
# 下载color模型的原始模型文件及AIPP配置文件
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/color.pb
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/aipp.cfg
# 执行模型转换命令，生成color的适配昇腾AI处理器的离线模型文件
atc --input_shape="input_1:-1,224,224,3" --output=./color_dynamic_batch --soc_version=Ascend310 --framework=3 --model=./color.pb --insert_op_conf=./aipp.cfg --dynamic_batch_size="1,2,4,8"
```

### 准备数据

样例编译时会自动下载测试数据，无需手工下载。

若您想自行下载测试数据，可参见如下命令：

```
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car0.mp4 --no-check-certificate
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car1.mp4 --no-check-certificate
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3_carColor_sample/data/car1.jpg --no-check-certificate
```

样例数据下载完后请存储在样例工程的data目录下。



# <a name="compile-run">样例编译运行</a>

环境及模型、数据准备完成后，您可参考本章节进行样例的编译运行。

1. 在通用目标识别样例工程的根目录下执行以下命令，进行样例编译。

   ```
   cd scripts 
   bash sample_build.sh
   ```

   编译完成后，会在out文件夹下生成可执行文件main。

2. 修改scripts目录下的params.conf文件，配置样例的输入数据类型及结果展示类型。

   ```
   [base_options]
   device_num=1    # Device数量    
   RtspNumPerDevice=1      # 每个Device上的输入路数
   
   [options_param_0]    # 第1路的配置参数
   inputType_0=pic       # 第1路的输入数据类型
   outputType_0=pic    # 第1路的输出数据类型
   inputDataPath_0=../data/pic   # 第1路的输入数据路径
   
   #outputFrameWidth_0=1280  # outputType_0为video时，需要配置此参数，代表输出视频的宽
   #outputFrameHeight_0=720  # outputType_0为video时，需要配置此参数，代表输出视频的高
   
   #[options_param_1]    # 第2路的配置参数
   #inputType_1=video
   #outputType_1=presentagent
   #inputDataPath_1=../data/car2.mp4
   #outputFrameWidth_1=2368
   #outputFrameHeight_1=1080
   
   .......
   ```

   参数说明：

   - device_num，表示运行此样例的Device数量，device_X_options表示每一个Device上的配置，其中X为Device ID。需要注意，device_num的优先级高于device_X_options的个数，例如，若device_num配置为1，但配置了两个Device的详细信息，即device_0_options与device_1_options，那么实际生效的只有device_0_options，若device_num配置为2，则device_0_options与device_1_options都会生效。

   - RtspNumPerDevice，表示每个Device上开启的路数，默认值为1。当有多个输入流的时候（多个离线视频/多个RTSP输入流），可通过此参数开启多路特性，提升推理性能。

   - inputType_X，表示第X+1路的输入数据类型，其中X需要从“0”开始递增，此参数当前支持的配置项有：

     - pic：表示输入数据为图片，当前此样例支持的图片格式为JPEG压缩图片
     - video：表示输入数据为MP4视频文件
     - rtsp：表示输入数据为rtsp流

   - outputType_X，表示第X+1路的输出数据类型，其中X需要从“0”开始递增，此参数当前支持的配置项有：

     - pic：表示输出结果为图片
     - video：表示输出结果为MP4视频文件
     - stdout：表示将推理结果打屏输出。
     - presentagent：表示用PresentAgent展示推理结果

      **注意：当前presentagent仅支持单路展示。若开启了多路视频特性，不支持使用presentagent在线展示；可配置为video或stdout，若路数较多，建议使用stdout打屏显示，否则性能可能会较低。**

   - inputDataPath_X：表示第X+1路的输入数据路径，其中X需要从“0”开始递增，此参数的配置规则如下：

     - 若输入数据类型是图片,则填写图片所在文件夹的相对路径，只支持填入一个路径
     - 若输入数据类型是mp4视频文件，则填写视频文件的相对路径，只支持填入一个路径
     - 若输入数据类型是rtsp流，则填写rtsp流地址，只支持填入一个地址

   其中options_param_X设置的路数会自动根据base_options分配到各个Device上，如果options_param_X设置的路数与base_options不匹配，则会报错。
   Device数量为2，每个Device开启两路输入的配置示例如下所示，请注意每个input与output配置的序号都是从0开始递增的：         

    ```
    [base_options]
    device_num=2   # Device数量    
    RtspNumPerDevice=2      # 每个Device上的输入路数

    [options_param_0]    # 第1路的配置参数
    inputType_0=video       # 第1路的输入数据类型
    outputType_0=video     # 第1路的输出数据类型
    inputDataPath_0=../data/video0.mp4   # 第1路的输入数据路径
    outputFrameWidth_0=1280  # outputType_0为video时，需要配置此参数，代表输出视频的宽
    outputFrameHeight_0=720  # outputType_0为video时，需要配置此参数，代表输出视频的高
 
    [options_param_1]    # 第2路的配置参数
    inputType_1 = video  # 第2路的输入数据类型
    outputType_1 = video   # 第2路的输出数据类型
    inputDataPath_1 =../data/video2.mp4
    outputFrameWidth_1=1280
    outputFrameHeight_1=720


    [options_param_2]    
    inputType_2=video       # 第3路的输入数据类型
    outputType_2=video      # 第3路的输出数据类型
    inputDataPath_2=../data/video3.mp4    # 第3路的输入数据路径
    outputFrameWidth_2=1280   # outputType_0为video时，需要配置此参数，代表输出视频的宽
    outputFrameHeight_2=720  # outputType_0为video时，需要配置此参数，代表输出视频的高
 
    [options_param_3]
    inputType_3 = video  # 第4路的输入数据类型
    outputType_3 = video   # 第4路的输出数据类型
    inputDataPath_3 =../data/video4.mp4
    outputFrameWidth_3=1280
    outputFrameHeight_3=720
    ```

3. 若输出类型配置的为“presentagent”，运行可执行文件前您需要参考此步骤启动PresentServer，若配置的其他输出类型，则此步骤可跳过。

   1. 配置PresentServer配置文件“present_start.conf”,配置文件参数如下：

      在通用目标识别样例根目录下执行如下命令打开配置文件：

   ```
      cd scripts
      vim present_start.conf
   ```

      配置文件如下所示：
        ```
        [present_serer_options]
        # A socket server address to communicate with presenter agent
        presenter_server_ip=192.168.1.2
        

        # The port of presenter agent and server communicate with
        presenter_server_port=7006
        
        #the ip in presenter server view web url 
        presenter_view_ip=192.168.1.2
        
        #view entry label in presenter server view web
        channel_name=multi_videos
        
        #the data type that send to presenter server from agent, 0:image, 1:video 
        content_type=1
        
        [display]
        display_channel=0
        ```

      - 其中presenter_server_ip为数据发送IP，presenter_view_ip为网页展示IP，两者的IP需要保持一致，配置参考如下：
        - 对于Atlas 200 DK开发者板，请填写Atlas 200 DK的与windows主机通信的IP地址即可，例如“192.168.1.2”
        - 对于Atlas 300加速卡（例如，ai1s云端推理环境），请填写ai1s的内网IP地址。
      - presenter_server_port：PresenterServer的访问端口，请配置为PresentAgent的默认端口号7006即可。

   2. 启动PresentServer服务。

      在通用目标识别样例根目录下执行如下命令启动PresentServer：

      ```
      cd ../display
      bash run_presenter_server.sh ../scripts/present_start.conf
      ```

      其中run_presenter_server.sh为PresentServer的启动脚本，present_start.conf为上一步骤中修改的PresentServer的配置文件。
      其中PresentServer后，界面会提示PresentServer服务所使用的IP地址及端口号。

   3. <a name="start_presentserver">访问PresentServer展示界面。</a>

      1. 在windows系统中通过浏览器访问PresentServer网页界面。

         - 对于Atlas 200 DK开发者板，请使用启动PresenterServer服务时提示的URL访问即可。
         - 对于Atlas 300加速卡（例如，ai1s云端推理环境）：
           以内网的IP地址为“192.168.0.194”，公网的IP地址为“124.70.8.192”进行举例说明。
           启动PresentServer服务时提示“Please visit http://192.168.0.194:7009 for display server”，用户需要将提示URL中的内网IP地址“192.168.0.194”替换为公网IP地址“124.70.8.192”进行访问，即需要访问的URL为“http://124.70.8.192:7009”。

      2. 等待PresentAgent传输数据给服务端，单击“Refresh“刷新，当有数据时相应Channel的Status会变成绿色。

      3. 然后单击右侧对应的View Name链接，查看结果。

         **说明** ：PresentServer当前仅支持显示前四路，如果业务有修改展示路数的需要，除代码开发适配外，还需要对网页UI代码进行修改：

         - 修改文件：

           display/presenterserver/display/ui/templates/view.html

         - 核心代码：

           ![输入图片说明](https://images.gitee.com/uploads/images/2022/0218/173836_91cde736_8083019.png "微信图片_20220218173816.png")

4. 运行样例。

   ```
   cd ../out
   ./main
   ```

5. 查看运行结果。

   样例将根据配置的输出数据类型，输出不同文件：

   - 若输出数据类型配置为pic
     输出数据存储在out/output文件夹下，为名称类似于**device_X_out_pic_Y.jpg** 的图片，其中X代表第x路，Y代表第y张图片。

   - 若输出数据类型配置为video
     输出数据存储在out/output文件夹下，为名称类似于：**out_testX.mp4** 的视频，其中X代表第x路。

   - 若输出数据类型配置为presentagent
     请参见[访问PresentServer展示界面](#start_presentserver)查看推理结果。



# 定制开发

## 模型替换

若现有样例中的模型无法满足用户的诉求，可按照本节中的步骤进行模型替换。

### 生成需要替换的离线模型

1. 将需要替换的原始框架模型保存到样例目录的model文件夹下。

2. 使用ATC工具，将原始框架模型转换为离线om模型。

   ATC工具的详细描述及使用约束可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“推理应用开发”的“ATC模型转换”。

### 样例解析

当前样例为多线程两模型串接样例，在模型替换的场景下，样例代码的如下部分需要进行修改：

- 模型推理相关代码需要进行修改。
- 若模型的输入数据要求与样例中模型对输入数据的要求不一致，则需要进行预处理部分代码的修改。
- 若模型的输出数据与样例中模型的输出数据不同或对数据的处理方式不同，则需要进行后处理部分代码的修改。

针对预处理及后处理的相关代码定制，本节不进行介绍，详细定制方法请参见[数据预处理](#data-preprocess)与[数据后处理](#data-postprocess)，本节主要介绍模型替换场景下如何对模型推理部分的代码进行修改，以及替换模型后，如何打通预处理-推理-后处理的代码流程。

下面我们从线程层面，分析当前样例的业务逻辑，如下所示：  

| **线程**              | **主要代码文件**                                | **线程功能介绍**                                             |
| --------------------- | ----------------------------------------------- | ------------------------------------------------------------ |
| 主线程                | src/main.cpp                                    | 主线程，0号线程，负责拉起所有线程，并等待收到结束信号后退出。 |
| 检测模型预处理线程    | src/detectPreprocess/detectPreprocess.cpp       | 检测模型预处理线程，线程的个数等于设置的路数，负责初始化消息数据，将解码后的图片数据处理为模型可以接受的数据并保存为消息数据，然后将消息数据标记为MSG_DETECT_PREPROC_DATA，并发送给推理线程，推理线程会根据数据是否为最后一帧，进行区分处理。 |
| 检测模型后处理线程    | src/detectPostprocess/detectPostprocess.cpp     | 检测模型后处理线程，线程的个数等于设置的路数，负责接受来自推理线程的被标记为MSG_DETECT_INFER_OUTPUT的消息数据，并对其进行检测模型的后处理，将消息数据标记为MSG_DETECT_POSTPROC_DATA后发送给分类模型预处理线程。 |
| 分类模型预处理线程    | src/classifyPreprocess/classifyPreprocess.cpp   | 分类模型预处理线程，线程的个数等于设置的路数，负责接受来自检测模型后处理线程的被标记为MSG_DETECT_POSTPROC_DATA的消息数据，并对其进行分类模型的预处理，将消息数据标记为MSG_CLASSIFY_PREPROC_DATA后发送给推理线程。 |
| 分类模型后处理线程    | src/classifyPostprocess/classifyPostprocess.cpp | 分类模型后处理线程，线程的个数等于设置的路数，负责接受来自推理线程的被标记为MSG_CLASSIFY_INFER_OUTPUT的消息数据，并对其进行分类模型的后处理，如果采用网页展示的形式输出推理结果，则会将消息数据标记为MSG_PRESENT_AGENT_DISPLAY继续发送给presentserver展示线程；其他场景下则单纯输出推理结果文件，并在接受到最后一帧数据时给主线程发送终止信号。 |
| 推理线程              | src/inference/inference.h                       | 推理线程，线程的个数等于Device的个数，负责接受被标记为MSG_DETECT_PREPROC_DATA和MSG_CLASSIFY_PREPROC_DATA的消息数据，并送给模型做推理，推理完成后再将数据发送给对应的后处理线程。 |
| presentserver展示线程 | src/presentagentDisplay/presentagentDisplay.cpp | 非必须，当且仅当样例采用presentserver展示的方式输出推理结果时被拉起。接受来自分类模型后处理线程的被标记为MSG_PRESENT_AGENT_DISPLAY的数据，并发送到网页，并在接受到最后一帧数据时给主线程发送终止信号。 |

通过以上表格可知，线程间主要以消息数据的形式进行交互，因此在替换模型的场景下，既要以替换后的模型初始化AclLiteModel类对象，也要保证替换模型后的推理线程可以与预处理/后处理线程进行消息数据的正确交互。

### 关键代码修改

下面详细介绍模型替换场景下推理部分的代码修改点。

1. 配置代表模型文件路径的变量，并用该变量初始化推理线程数据成员中的AclLiteModel类对象。

   将原推理线程类中定义的AclLiteModel类的实例：```detectModel_```与```classifyModel_```，修改为替换后的模型对象。

   代码文件：src/inference/inference.h

   参考代码：

   ```
    ...
    // AclLiteModel detectModel_; --> AclLiteModel targetModel_;
    AclLiteModel detectModel_;
    AclLiteModel classifyModel_;
    ...
   ```

   修改推理线程类对象的构造函数，将构造函数中初始化数据成员AclLiteModel类实例的文件路径，修改为替换后的模型文件路径。

   代码文件：src/inference/inference.cpp

   参考代码：

   ```
   ...
   namespace{
   ...
   //global variable, model file path
   //const char* kDetectModelPath = "../model/yolov3.om"; --> const char* kTargetModelPath = "../model/target.om";
   const char* kDetectModelPath = "../model/yolov3.om";
   const char* kClassifyModelPath = "../model/color_dvpp_10batch.om";
   ...
   }
   
   //inferenceThread constructed function
   //detectModel_(kDetectModelPath), --> targetModel_(kTargetModelPath)
   InferenceThread::InferenceThread(aclrtRunMode& runMode) :
   detectModel_(kDetectModelPath),
   classifyModel_(kClassifyModelPath),
   batchSize_(kBatch),
   runMode_(runMode) {
       imageInfoSize_ = 0;
       imageInfoBuf_ = nullptr;
       classifyInputSize_ = 0;
       classifyInputBuf_ = nullptr;
   }
   
   ...
   
   // init 
   AclLiteError InferenceThread::Init() {
   
       // AclLiteError ret = detectModel_.Init(); --> AclLiteError ret = targetModel_.Init();
       AclLiteError ret = detectModel_.Init();
       if (ret != ACLLITE_OK) {
           ACLLITE_LOG_ERROR("detect Model init failed, error:%d", ret);
           return ret;
       }
   
       ...
    
       return ACLLITE_OK;
   }
   ```

2. 根据替换后模型的实际需要，对初始化模型输入接口进行替换。

   以下代码样例以初始化检测模型和颜色识别模型为例，此处的输入数据为图片数据。用户需要根据实际需要对如下代码进行改造。

   代码文件：
   src/inference/inference.cpp

   参考代码：

   ```
   AclLiteError InferenceThread::InitModelInput() {   
       //prepare classify model input2 data & size
       classifyInputSize_ = YUV420SP_SIZE(kClassifyModelWidth, kClassifyModelHeight) * batchSize_;
       void* buf = nullptr;
       aclError aclRet = aclrtMalloc(&buf, classifyInputSize_, 
                                     ACL_MEM_MALLOC_HUGE_FIRST);
       if ((buf == nullptr) || (aclRet != ACL_ERROR_NONE)) {
           ACLLITE_LOG_ERROR("Malloc classify inference input buffer failed, "
                           "error %d", aclRet);
           return ACLLITE_ERROR;
       }
       classifyInputBuf_ = (uint8_t *)buf;
       return ACLLITE_OK;
   }
   ```

   ```
   AclLiteError InferenceThread::DetectModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
   
       ......
   
       //prepare detectmodel input2 data & size
       aclFloat16 new_shapeHeight = aclFloatToFloat16((float)kDetectModelHeight);
       aclFloat16 new_shapeWidth = aclFloatToFloat16((float)kDetectModelWidth);
       aclFloat16 orig_shapeHeight = aclFloatToFloat16((float)carDetectDataMsg->imageFrame.height);
       aclFloat16 orig_shapeWidth = aclFloatToFloat16((float)carDetectDataMsg->imageFrame.width);
       const aclFloat16 imageInfo[4] = {new_shapeHeight, new_shapeWidth,
                                    orig_shapeHeight, orig_shapeWidth};
       imageInfoSize_ = aclDataTypeSize(ACL_FLOAT16) * 4;
       imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                    runMode_, MEMORY_DEVICE);
       if (imageInfoBuf_ == nullptr)
       {
           ACLLITE_LOG_ERROR("Copy image info to device failed");
           return ACLLITE_ERROR;
       }
   
       ......
   }
   ```

3. 修改模型的执行接口。

   由于线程间是通过消息交互通信，因此模型推理所需数据及推理过程中产生的中间结果都需要存放在消息数据中。所以判断当前样例的模型执行接口是否需要修改，以及替换模型后推理结果如何保存在消息数据中，都要求用户对消息数据的成员变量有一定的了解，才能进行分析设计。

   本步骤中仅展示当前样例中消息数据的接口以及检测模型执行推理接口的逻辑，以供用户参考：

   消息数据结构： 

   | **结构体**       | **代码文件**    | **注释**                                                     |
   | ---------------- | --------------- | ------------------------------------------------------------ |
   | CarDetectDataMsg | inc/CarParams.h | inferThreadId：推理线程ID，标识消息将发送的线程。<br>detectPostThreadId：检测模型后处理线程ID，标识消息将发送的线程。<br>classifyPreThreadId：分类模型预处理线程ID，标识消息将发送的线程。<br>classifyPostThreadId：分类模型后处理线程ID，标识消息将发送的线程。<br>presentAgentDisplayThreadId：页面展示线程ID，标识消息将发送的线程。<br>deviceId：设备ID，区分设备配置参数。<br>isLastFrame：是否为最后一帧数据，0为不是，1为是。<br>frameNum：数据帧数，表示为该路第x帧数据。<br>imageFrame：自定义结构体数据，存放将原始图片解码为YUV420SP_U8格式后的数据，数据存放在dvpp内存上。<br>resizedFrame：自定义结构体数据，存放缩放至检测模型所需分辨率大小的图片数据，数据存放在dvpp内存上。<br>frame：OpenCV的Mat类数据，存放BGR格式的原始图片数据，分辨率与原图一致。用于后续将两模型推理结果画在其中。<br>detectInferData：检测模型推理结果。<br>carInfo：自定义结构体数据，用于存放检测模型的推理结果及分类模型需要使用的图片数据。<br>flag：判断是否有车辆被检测到，0为无，1为有。<br>classifyInferData：分类模型推理结果。 |
   | CarInfo          | inc/CarParams.h | cropedImgs：自定义结构体数据，存放从imageFrame抠出的有车辆区域。<br>resizedImgs：自定义结构体数据，将cropedImgs缩放至车辆颜色分类模型所需分辨率大小的图片数据。<br>rectangle：检测框，存放检测区域左上右下两点的坐标。<br>detect_result：检测模型推理结果。<br>carColor_result：车辆颜色分类模型推理结果。 |

   代码文件：src/inference/inference.cpp

   参考代码：

   ```
   AclLiteError InferenceThread::DetectModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
       // if last frame 
       if (carDetectDataMsg->isLastFrame == 1) {
           ACLLITE_LOG_INFO("it is lastframe in Detect Inference");
           return ACLLITE_OK;
       }
   
       ......
   
       // create input
       AclLiteError ret = detectModel_.CreateInput(carDetectDataMsg->resizedFrame.data.get(), 
                                                   carDetectDataMsg->resizedFrame.size, 
                                                   imageInfoBuf_, imageInfoSize_);
       if (ret != ACLLITE_OK) {
           ACLLITE_LOG_ERROR("Create detect model input dataset failed");
           return ACLLITE_ERROR;
       }
       // save infer result to detectInferData
       ret = detectModel_.Execute(carDetectDataMsg->detectInferData);
       if (ret != ACLLITE_OK) {
           ACLLITE_LOG_ERROR("Execute detect model inference failed, error: %d", ret);
           return ACLLITE_ERROR;
       }
       // destroy input
       detectModel_.DestroyInput();
   
       return ACLLITE_OK;
   }
   
   ```

4. 修改推理完成后的消息发送接口。

   本样例通过定义一系列消息数据标识码，来区分不同线程不同状态的消息数据。替换模型的场景下，可以直接复用如下代码，您也可以重新设计一套标识码。

   代码文件：inc/CarParams.h

   参考代码：

   ```
   ···
   
   //const int MSG_DETECT_PREPROC_DATA = 3; --> const int MSG_TARGET_PREPROC_DATA = X;
   //const int MSG_DETECT_INFER_OUTPUT = 4; --> const int MSG_TARGET_INFER_OUTPUT = Y;
   //const int MSG_DETECT_POSTPROC_DATA = 5; --> const int MSG_TARGET_POSTPROC_DATA = Z;
   
   const int MSG_APP_START = 1;
   const int MSG_READ_FRAME = 2;
   const int MSG_DETECT_PREPROC_DATA = 3;
   const int MSG_DETECT_INFER_OUTPUT = 4;
   const int MSG_DETECT_POSTPROC_DATA = 5;
   const int MSG_CLASSIFY_PREPROC_DATA = 6;
   const int MSG_CLASSIFY_INFER_OUTPUT = 7;
   const int MSG_ENCODE_FINISH = 8;
   const int MSG_PRESENT_AGENT_DISPLAY = 9;
   const int MSG_APP_EXIT = 10;
   ···
   ```

   若消息数据标识码有更新，您需要使用更新后的标识码来标识存放模型推理结果的消息数据，并发送给业务下游的目标线程。

   代码文件：src/inference/inference.cpp

   参考代码：

   ```
   // function name: DetectMsgSend --> TargetMsgSend
   AclLiteError InferenceThread::DetectMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
       while(1)
       {
           // MSG_DETECT_INFER_OUTPUT --> MSG_TARGET_INFER_OUTPUT, send msg to taeget model postprocess thread
           AclLiteError ret = SendMessage(carDetectDataMsg->detectPostThreadId, MSG_DETECT_INFER_OUTPUT, carDetectDataMsg);
           if(ret == ACLLITE_ERROR_ENQUEUE)
           {
               usleep(500);
               continue;
           }
           else if(ret == ACLLITE_OK)
           {
               break;
           }
           else
           {
               ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
               return ret;
           }
       }
   
       return ACLLITE_OK;
   }
   ```

5. 修改推理线程消息数据处理接口，串接模型的推理与消息发送业务流程。

   代码文件：src/inference/inference.cpp

   参考代码：

   ```
   AclLiteError InferenceThread::Process(int msgId, shared_ptr<void> data) {
       switch(msgId) {
           // MSG_DETECT_PREPROC_DATA、MSG_CLASSIFY_PREPROC_DATA is detect/classify preprocess thread result msg
        
           //case MSG_TARGET_PREPROC_DATA:
           //    TargetModelExecute(static_pointer_cast<CarDetectDataMsg>(data));
           //    TargetMsgSend(static_pointer_cast<CarDetectDataMsg>(data));
           //    break;        
   
           case MSG_DETECT_PREPROC_DATA:
               DetectModelExecute(static_pointer_cast<CarDetectDataMsg>(data));
               DetectMsgSend(static_pointer_cast<CarDetectDataMsg>(data));
               break;
        
           case MSG_CLASSIFY_PREPROC_DATA:
               ClassifyModelExecute(static_pointer_cast<CarDetectDataMsg>(data));
               ClassifyMsgSend(static_pointer_cast<CarDetectDataMsg>(data));
               break;
           default:
               ACLLITE_LOG_INFO("Inference thread ignore msg %d", msgId);
               break;
       }
   
       return ACLLITE_OK;
   }
   ```

6. 修改串接预处理->推理->后处理线程的代码。


   对于预处理线程，若消息数据标识码有更新，则需要将消息数据标识为更新后的标识码（例如：MSG_TARGET_PREPROC_DATA），然后发送给业务下游的目标线程。

   代码文件：src/detectPreprocess/detectPreprocess.cpp

   参考代码：

   ```
   AclLiteError DetectPreprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
       AclLiteError ret;
       if(carDetectDataMsg->isLastFrame == 0){
           while(1){
               // if not last frame, send msg to infer thread
               // sign msg by MSG_DETECT_PREPROC_DATA --> MSG_TARGET_PREPROC_DATA
               ret = SendMessage(carDetectDataMsg->inferThreadId, MSG_DETECT_PREPROC_DATA, carDetectDataMsg);
               if(ret == ACLLITE_ERROR_ENQUEUE)
               {
                   usleep(500);
                   continue;
               }
               else if(ret == ACLLITE_OK)
               {
                   break;
               }
               else
               {
                   ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                   return ret;
               }
           }

           ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
           if (ret != ACLLITE_OK) {
               ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
               return ret;
           } 
       }
       // if last frame, need set msg value
       else{
               shared_ptr<CarDetectDataMsg> carDetectDataMsgEnd = make_shared<CarDetectDataMsg>();
               carDetectDataMsgEnd->inferThreadId = inferThreadId_;
               carDetectDataMsgEnd->detectPostThreadId = detectPostThreadId_;
               carDetectDataMsgEnd->classifyPreThreadId = classifyPreThreadId_;
               carDetectDataMsgEnd->classifyPostThreadId = classifyPostThreadId_;
               carDetectDataMsgEnd->presentAgentDisplayThreadId = presentAgentDisplayThreadId_;
               carDetectDataMsgEnd->deviceId = carDetectDataMsg->deviceId;
               carDetectDataMsgEnd->frameNum = carDetectDataMsg->frameNum;
               carDetectDataMsgEnd->isLastFrame = carDetectDataMsg->isLastFrame;
               while(1)
               {
                   // sign msg by MSG_DETECT_PREPROC_DATA --> MSG_TARGET_PREPROC_DATA
                   ret = SendMessage(carDetectDataMsgEnd->inferThreadId, MSG_DETECT_PREPROC_DATA, carDetectDataMsgEnd);
                   if(ret == ACLLITE_ERROR_ENQUEUE)
                   {
                       usleep(500);
                       continue;
                   }
                   else if(ret == ACLLITE_OK)
                   {
                       break;
                   }
                   else
                   {
                       ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                       return ret;
                   }
               }
       }

       return ACLLITE_OK;
   }
   ```

   对于结尾的后处理线程，若消息数据标识码有更新，则需要将代码修改为接收新的消息数据标识码（例如：MSG_TARGET_INFER_OUTPUT）标识的数据，并进行后续处理。

   代码文件：src/classifyPostprocess/classifyPostprocess.cpp

   参考代码：

   ```
   AclLiteError ClassifyPostprocessThread::Process(int msgId, 
                                shared_ptr<void> data) {
       AclLiteError ret = ACLLITE_OK;
       switch(msgId) {
           // reveive msg, signed by MSG_CLASSIFY_INFER_OUTPUT, from infer thread
           // MSG_CLASSIFY_INFER_OUTPUT --> MSG_TARGET_INFER_OUTPUT
           case MSG_CLASSIFY_INFER_OUTPUT:
               InferOutputProcess(static_pointer_cast<CarDetectDataMsg>(data));
               break;
           // reveive msg, signed by MSG_ENCODE_FINISH, when is last frame, from classify postprocess thread
           case MSG_ENCODE_FINISH:
               // send exit signal to main thread
               SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
               break;
           default:
               ACLLITE_LOG_INFO("Classify Postprocess thread ignore msg %d", msgId);
               break;
       }

       return ret;
   }
   ```

   在复杂的业务场景中，对于业务中间的预处理/后处理线程，只要保证消息数据被正确标识以便于区分，以及发送的线程id正确即可，无其他特殊处理逻辑，参考代码如下所示。

   代码文件：src/classifyPreprocess/classifyPreprocess.cpp、src/detectPostprocess/detectPostprocess.cpp

   参考代码：

   ```
   AclLiteError DetectPostprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
       while(1) 
       {
           AclLiteError ret = SendMessage(carDetectDataMsg->classifyPreThreadId, MSG_DETECT_POSTPROC_DATA, carDetectDataMsg);
           if(ret == ACLLITE_ERROR_ENQUEUE)
           {
               usleep(500);
               continue;
           }
           else if(ret == ACLLITE_OK)
           {
               break;
           }
           else
           {
               ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
               return ret;
           }
       }

       return ACLLITE_OK;
   }

   AclLiteError ClassifyPreprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
       while(1) 
       {
           AclLiteError ret = SendMessage(carDetectDataMsg->inferThreadId, MSG_CLASSIFY_PREPROC_DATA, carDetectDataMsg);
           if(ret == ACLLITE_ERROR_ENQUEUE)
           {
               usleep(500);
               continue;
           }
           else if(ret == ACLLITE_OK)
           {
               break;
           }
           else
           {
               ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
               return ret;
           }
       }

       return ACLLITE_OK;
   }

   ```

### 模型增删

在模型增加或删除的场景中，可以参考以上板块内容，主要参考以下步骤进行开发：

 - 增加模型时，开发对应的预处理线程和后处理线程文件，并在推理线程中添加对应的模型推理接口；删除模型时，去除该模型对应的预处理线程后处理线程文件，并在推理线程中删去对应的模型推理接口；

 - 完成以上开发后，根据自身业务，在```CarParams.h```中增加/删除，新增/删去的线程文件的线程号；并通过这些线程号，使得消息数据的能够在线程之间被识别和收发，使得更新后的各业务线程被串联，从而保证了线程嵌入/移出样例的原业务流程。



## <a name="data-preprocess">数据预处理</a>

### 概述

基于CANN开发的推理应用程序支持多种格式的媒体文件的输入，包含图片及视频，针对不同的输入，推理前处理的方式不同，本章节详细介绍不同输入媒体的预处理方式。

### 准备动作

- 设置输入数据类型

  当前通用识别样例支持图片、视频、RTSP流三种类型的多媒体数据；主要通过配置文件中的参数 **inputType_X** 设置第X路输入数据类型。该参数的值仅可在pic video rtsp三者中任选其一，详细设置方法及说明请见[样例编译运行](#compile-run)。

- 数据准备

  在data文件夹下，存放待测试数据，若文件夹不存在则需手动创建。
  测试数据下载完成后，将数据文件路径填写至配置文件中，细设置方法及说明请见[样例编译运行](#compile-run)。



### <a name="picture-process">图片</a>

#### 简介

本样例使用的模型支持的输入图片约束如下：

| **模型**                                        | **输入图片编码格式** | **输入图片分辨率** |
| ----------------------------------------------- | -------------------- | ------------------ |
| 图片检测推理模型。基于onnx的yolov3模型          | BGR                  | 宽：416 高：416    |
| 车辆颜色分类推理模型。基于tensorflow的CNN模型。 | RGB                  | 宽：224 高：224    |

若您想直接使用样例中的模型，但输入图片不符合模型要求；或者是想使用自己的模型，而模型对输入图片的要求也与样例模型不一致。在这两者任一场景下，都可以参考本章节，对输入图片的预处理流程进行改造，使其符合实际业务要求。

**注：** 样例使用的离线模型，由于另外配置了AIPP文件进行了格式转换，所以在代码层面，图片数据是直接以YUV420SP_U8的格式送给模型做推理，相关内容请见本篇的[格式转换](#format_trans) 。

#### 解码

输入模型的图片数据要求为非压缩的、指定编码格式的图片数据，若您的原始图片是经过压缩的（例如后缀为.jpg、.jpeg的图片），此种场景下，则需要将压缩后的图片数据解码为指定的某种编码格式的数据，然后再进行后续的操作。

##### 样例解析

本样例中，输入数据是后缀为.jpg的压缩图片，所以首先使用DVPP的JPEGD功能将图片解码为YUV420SP_U8格式，然后再对图片进行缩放、抠图等操作。解码相关操作的代码示例如下：

1. 打开图片所在文件夹，并将图片文件名存储到vector中。

   代码文件：src/detectPreprocess/detectPreprocess.cpp

   代码示例：
    <a name="OpenPicsDir"></a>

    ```
    AclLiteError DetectPreprocessThread::OpenPicsDir() {
        # inputDataPath_ : pics directory path
        string inputImageDir = inputDataPath_;
        # fileVec_ : vector which save all pics name
        GetAllFiles(inputImageDir, fileVec_);
        if (fileVec_.empty()) {
            ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
            return ACLLITE_ERROR;
        }
    
        return ACLLITE_OK;
    }
    ```

2. 根据vector中的图片文件名，将图片读入。

   代码文件：src/detectPreprocess/detectPreprocess.cpp

   代码示例：

    ```
    AclLiteError DetectPreprocessThread::ReadPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
        // set msg data value 
        carDetectDataMsg->inferThreadId = inferThreadId_;
        carDetectDataMsg->detectPostThreadId = detectPostThreadId_;
        carDetectDataMsg->classifyPreThreadId = classifyPreThreadId_;
        carDetectDataMsg->classifyPostThreadId = classifyPostThreadId_;
        carDetectDataMsg->presentAgentDisplayThreadId = presentAgentDisplayThreadId_;
        carDetectDataMsg->deviceId = deviceId_;
        carDetectDataMsg->frameNum = frameCnt_;
        carDetectDataMsg->isLastFrame = 0;
    
        // if last pic
        if (frameCnt_ == fileVec_.size()) { 
            carDetectDataMsg->isLastFrame = 1;
            return ACLLITE_OK;
        }
        
        string picFile = fileVec_[frameCnt_];
        // read jpg pic to ImageData, needed to dvpp process and model execute
        AclLiteError ret = ReadJpeg(carDetectDataMsg->imageFrame, picFile);
        // read jpg pic to Mat, needed to record model inference result
        carDetectDataMsg->frame = cv::imread(picFile);
    
        frameCnt_++;
        
        return ACLLITE_OK;
    }
    ```

3. 使用DVPP的JPEGD功能将读入的jpg图片数据，解压缩为YUV420SP_U8格式的数据。

   其中```DetectPreprocessThread::ProcessPic```接口是对JPEGD功能的调用接口，```JpegDHelper::Process```接口是对AscendCL的```acldvppJpegDecodeAsync```接口的封装，JPEGD的功能描述可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“推理应用开发”的“高级功能 > 媒体数据预处理V1 > JPEGD图片解码”。

   - 代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例：

     ```
     AclLiteError DetectPreprocessThread::ProcessPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         // if last pic
         if(carDetectDataMsg->isLastFrame == 1)
             return ACLLITE_OK;
        
         ImageData imageDevice, yuvImage;
         // copy data to dvpp memory
         AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Copy image to device failed");
             return ACLLITE_ERROR;
         }
         // jpegd
         ret = dvpp_.JpegD(carDetectDataMsg->imageFrame, imageDevice);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Pic decode failed");
             return ACLLITE_ERROR;
         }
         ...
     
         return ACLLITE_OK;
     }
     ```

   - 代码文件：acllite/src/JpegDHelper.cpp

     代码示例：

     ```
     AclLiteError JpegDHelper::Process(ImageData& dest, ImageData& src) {
         // init ouput pic data desc
         int ret = InitDecodeOutputDesc(src);
         if (ret != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("InitDecodeOutputDesc failed");
             return ret;
         }
         // jpegd
         aclError aclRet = acldvppJpegDecodeAsync(dvppChannelDesc_, 
                                                  reinterpret_cast<void *>(src.data.get()),
                                                  src.size, decodeOutputDesc_, stream_);
         if (aclRet != ACL_SUCCESS) {
             ACLLITE_LOG_ERROR("acldvppJpegDecodeAsync failed, error: %d", aclRet);
             return ACLLITE_ERROR_JPEGD_ASYNC;
         }
     
         aclRet = aclrtSynchronizeStream(stream_);
         if (aclRet != ACL_SUCCESS) {
             ACLLITE_LOG_ERROR("Sync stream failed, error: %d", aclRet);
             return ACLLITE_ERROR_SYNC_STREAM;
         }
         // set ImageData value
         dest.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
         dest.width = src.width;
         dest.height = src.height;
         dest.alignWidth = ALIGN_UP128(src.width);
         dest.alignHeight = ALIGN_UP16(src.height);
         dest.size = YUV420SP_SIZE(dest.alignWidth, dest.alignHeight);
         dest.data = SHARED_PTR_DVPP_BUF(decodeOutBufferDev_);
        
         return ACLLITE_OK;
     }
     ```

   关键功能接口列表：

   | **功能场景**                       | **代码文件**                                                 | **接口/结构体**                                 |
   | ---------------------------------- | ------------------------------------------------------------ | ----------------------------------------------- |
   | 识别配置文件中文件夹的所有图片文件 | src/detectPreprocess/detectPreprocess.cpp                    | OpenPicsDir                                     |
   | 车辆检测模型读取Jpg图片            | acllite/src/AclLiteUtils.cpp                                 | ReadJpeg                                        |
   | 车辆检测模型读取Jpg图片接口调用处  | src/detectPreprocess/detectPreprocess.cpp                    | ReadPic                                         |
   | JPEGD功能接口                      | acllite/src/JpegDHelper.cpp<br>acllite/src/AclLiteImageProc.cpp | JpegDHelper::Process<br>AclLiteImageProc::JpegD |
   | JPEGD功能调用                      | src/detectPreprocess/detectPreprocess.cpp                    | DetectPreprocessThread::ProcessPic              |


##### 定制开发

下面介绍几种常见场景下，如何基于本样例进行解码功能的定制开发。

- 输入图片数据为YUV420SP_U8格式的文件，但后续需要使用DVPP的VPC功能对图片的大小进行调整。

  此种场景下，输入数据已经为YUV420SP_U8编码格式的数据，无需进行解码操作。但由于后续需要使用DVPP的VPC功能对图片的进行二次处理，则需要将读取后的数据封装为和消息数据```carDetectDataMsg```中的数据成员```imageFrame```同类型的数据（ImageData类型）。

  基于样例进行定制的流程如下：

  1. 打开图片文件夹并读取文件夹下的图片。

     打开图片文件夹的接口可直接复用样例代码，无需更改；但需要修改读取图片的接口```DetectPreprocessThread::ReadPic```，将其中的```ReadJpeg```替换为```ReadBinFile```，代码示例如下：

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例：

     ```
        AclLiteError DetectPreprocessThread::ReadPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
            // set msg data value 
            carDetectDataMsg->inferThreadId = inferThreadId_;
            carDetectDataMsg->detectPostThreadId = detectPostThreadId_;
            carDetectDataMsg->classifyPreThreadId = classifyPreThreadId_;
            carDetectDataMsg->classifyPostThreadId = classifyPostThreadId_;
            carDetectDataMsg->presentAgentDisplayThreadId = presentAgentDisplayThreadId_;
            carDetectDataMsg->deviceId = deviceId_;
            carDetectDataMsg->frameNum = frameCnt_;
            carDetectDataMsg->isLastFrame = 0;
        
            // if last pic
            if (frameCnt_ == fileVec_.size()) { 
                carDetectDataMsg->isLastFrame = 1;
                return ACLLITE_OK;
            }
            
            string picFile = fileVec_[frameCnt_];
      
            // read pic data to memory
            void* data = nullptr;
            uint32_t size = 0;
            AclLiteError ret =  **ReadBinFile(picFile, data, size)** ;
            ...
            need api，turning data and size to imageFrame in carDetectDataMsg
            ...
            // read pic to Mat, needed to record model inference result
            carDetectDataMsg->frame = cv::imread(picFile);
        
            frameCnt_++;
            
            return ACLLITE_OK;
        }
     ```

  2. 将读取后的数据封装为ImageData类型的数据，代码示例如下：

     ImageData定义如下，存储在“acllite/inc/AclLiteType.h”文件中：

     ```
     struct ImageData {
         acldvppPixelFormat format;
         uint32_t width = 0;
         uint32_t height = 0;
         uint32_t alignWidth = 0;
         uint32_t alignHeight = 0;
         uint32_t size = 0;
         std::shared_ptr<uint8_t> data = nullptr;
      ;
     ```

     将上一步读取的数据封装为ImageData类型的代码参考如下：

     ```
     ...
     // dest：dst ImageData
     dest.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
     /*
         width : origin pic width
         height: origin pic height
         alignWidth：align up width，default 0
         alignHeight ：align up height，default 0
         size : data size
         buf : pointer, pic data
         data : smart pointer
     */
     dest.width = width;
     dest.height = height;
     dest.alignWidth = 0;
     dest.alignHeight = 0;
     dest.size = size;
     image.data.reset((uint8_t *)buf, [](uint8_t* p) 
                 { delete[](p); }
                 );
     ...
     ```

  3. 由于后续需要使用dvpp功能，使用DVPP功能则数据必须在DVPP内存上，因此需要将数据拷贝至DVPP内存后，再调用相关功能接口进行操作，此处介绍数据拷贝的代码示例。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例：

     ```
     AclLiteError DetectPreprocessThread::ProcessPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         ...
         // copy data to dvpp memory
         AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Copy image to device failed");
             return ACLLITE_ERROR;
         }
         ...
      }
     ```

  4. 本样例使用多线程+消息队列实现线程通信，若您不使用样例中的ImageData数据结构存储数据，则需要对消息数据中的数据成员进行替换，而且也需要对可能涉及到的接口的参数类型进行修改。

     消息数据定义代码文件：acllite/inc/CarParams.h

     代码示例：

     ```
     ...
     struct CarDetectDataMsg {
         int inferThreadId;
         int detectPostThreadId;
         int classifyPreThreadId;
         int classifyPostThreadId;
         int presentAgentDisplayThreadId;
         uint32_t deviceId;
         int isLastFrame;
         int frameNum;
        
         // yuv ImageData 
         ImageData imageFrame;
         // resized ImageData 
         ImageData resizedFrame;
         // record model infer result
         cv::Mat frame;
        
         vector<InferenceOutput> detectInferData;
         vector<CarInfo> carInfo;
         int flag;
         vector<InferenceOutput> classifyInferData;
     };
     ...
     ```

    关键功能接口列表 ：   

  | **功能场景**                       | **代码文件**                              | **接口/结构体**                                       |
  | ---------------------------------- | ----------------------------------------- | ----------------------------------------------------- |
  | 识别配置文件中文件夹的所有图片文件 | src/detectPreprocess/detectPreprocess.cpp | OpenPicsDir                                           |
  | 读取bin文件                        | acllite/src/AclLiteUtils.cpp              | ReadBinFile                                           |
  | 车辆检测模型读取bin文件接口替换处  | src/detectPreprocess/detectPreprocess.cpp | ReadPic                                               |
  | 结构体ImageData                    | acllite/inc/AclLiteType.h                 | ImageData：封装图片数据及图片相关参数的结构体         |
  | 结构体CarDetectDataMsg             | acllite/inc/params.h                      | CarDetectDataMsg：基础的消息的结构体                  |
  | 结构体CarInfo                      | inc/params.h                              | CarInfo：车辆检测模型推理的中间结果及抠出的车辆区域等 |


- 若您的图片数据既不是jpg文件，也不是YUV420SP_U8格式文件，但后续仍然需要使用DVPP的VPC功能对图片的进行缩放或者抠图等处理。

  此种场景下，您可以使用OpenCV等第三方图像处理接口进行读取和解码操作，并使图片数据满足DVPP对图片进行缩放抠图等操作的约束，详细约束可参见[昇腾文档中心](https://www.hiascend.com/document?tag=community-developer)相关文档。

  如下为使用OpenCV进行图片预处理的示例：

  1. 打开图片所在文件夹，并将图片文件名存储到vector中。

     该部分代码可直接参考 [OpenPicsDir](#OpenPicsDir) 进行复用。

  2. 根据vector中的图片文件名，用OpenCV将图片读入并转换图片编码格式，并将Mat转换为ImageData类型数据。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例如下：

     ```
     AclLiteError DetectPreprocessThread::ReadPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         ...     
         string picFile = fileVec_[frameCnt_];
         // read pic by opencv
         cv::Mat tmpMat, yuvMat;
         tmpMat = cv::imread(picFile);
         // convert bgr to yuv
         cv::cvtColor(tmpMat, yuvMat, BGR_NV122CV_YUV);
         // turn Mat into ImageData
         carDetectDataMsg->imageFrame.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
         carDetectDataMsg->imageFrame.width = yuvMat.cols;
         carDetectDataMsg->imageFrame.height = yuvMat.rows;
         carDetectDataMsg->imageFrame.alignWidth = 0;
         carDetectDataMsg->imageFrame.alignHeight = 0;
         carDetectDataMsg->imageFrame.size = yuvMat.cols * yuvMat.rows * yuvMat.elemSize();
         carDetectDataMsg->imageFrame.data.reset((uint8_t *)yuvMat.data, [](uint8_t* p) 
                                                 { delete[](p); }
                                                 );
     
         // read jpg pic to Mat, which is used to record model inference result
         carDetectDataMsg->frame = cv::imread(picFile);
         ...
     }
     ```

  3. 由于此场景后续需要使用DVPP功能，则数据必须在DVPP内存上，因此需要将数据拷贝至DVPP内存后，再调用相关功能接口进行操作。此处仅介绍将数据拷贝至DVPP内存的代码示例。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例如下：

     ```
     AclLiteError DetectPreprocessThread::ProcessPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         ...
         // copy data to dvpp memory
         AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Copy image to device failed");
             return ACLLITE_ERROR;
         }
         ...
      }
     ```

   接口开发完毕后，可以参见如下关键功能接口列表确认需要更改的地方：

  | **业务场景**                                                 | **代码文件**                              | **接口/结构体**                                       |
  | ------------------------------------------------------------ | ----------------------------------------- | ----------------------------------------------------- |
  | 接口开发完成后替换处，该接口旨在实现从文件读取数据，并设置消息数据参数 | src/detectPreprocess/detectPreprocess.cpp | ReadPic                                               |
  | 结构体ImageData                                              | acllite/inc/AclLiteType.h                 | ImageData：封装图片数据及图片相关参数的结构体         |
  | 结构体CarDetectDataMsg                                       | inc/params.h                              | CarDetectDataMsg：基础的消息的结构体                  |
  | 结构体CarInfo                                                | inc/params.h                              | CarInfo：车辆检测模型推理的中间结果及抠出的车辆区域等 |

- 若您的图片数据既不是jpg文件，也不是bin文件，且后续不需要使用DVPP的VPC功能对图片做处理或者打算使用其他第三方库对图片做处理。

  需要您自行分析上述表格中涉及的接口、结构体及被调用的位置，考虑接口开发及流程的设计规划。

#### <a name="vpc-process">计算机视觉预处理（VPC）</a>

当解码后的图片数据需要进行二次处理以满足业务或模型需要时，例如当图片数据分辨率不满足模型需要，或者是需要抠取图片某块区域，此时可使用DVPP的VPC功能对图片数据进行处理。

本节仅对VPC（Vision Preprocessing Core）功能中的抠图与缩放功能进行介绍，VPC的使用约束及其他VPC功能的描述可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“推理应用开发”的“AscendCL API参考 > 媒体数据预处理V1 > VPC功能”。

##### 样例解析

本样例涉及两个模型，故有两套预处理流程。其中图片检测模型预处理流程为：缩放（resize）；车辆颜色分类模型预处理流程为：抠图（crop）-> 缩放（resize）。由于抠图和缩放两功能属并列关系，因此按功能划分，分别介绍两个功能的实现流程。

- 缩放功能实现流程参考如下：

  1. 创建一个AclLiteImageProc类对象并将其初始化，该类内部主要负责管理DVPP图片处理相关功能的调用。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例：
            

     ```
     AclLiteError DetectPreprocessThread::Init() {
                
         ...
         // dvpp_ ：data member in class DetectPreprocessThread
         aclRet = dvpp_.Init();
         if (aclRet) {
         ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
         return ACLLITE_ERROR;
         }
            
         ...
            
         return ACLLITE_OK;
      }        
     ```

  2. 通过AclLiteImageProc类对象，调用缩放功能相关功能接口，修改图片大小使其满足模型需要。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例：
            

     ```
     AclLiteError DetectPreprocessThread::ProcessPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         // if last frame
         if(carDetectDataMsg->isLastFrame == 1)
             return ACLLITE_OK;
         ImageData imageDevice, yuvImage;
         // copy data from host to dvpp memory
         AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Copy image to device failed");
             return ACLLITE_ERROR;
         }
         // decode
         ret = dvpp_.JpegD(carDetectDataMsg->imageFrame, imageDevice);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Pic decode failed");
             return ACLLITE_ERROR;
         }
         // ProportionPasteCenter，modelWidth_：target width，modelHeight_：target height
         ret = dvpp_.ProportionPasteCenter(carDetectDataMsg->resizedFrame, carDetectDataMsg->imageFrame, 0, 0, modelWidth_, modelHeight_);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Pic decode failed");
             return ACLLITE_ERROR;
         }
            
         return ACLLITE_OK;
     }     
     ```

   关键功能接口列表：

  | **业务场景**                               | **代码文件**                                                 | **接口/结构体**                                              |
  | ------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | 创建并初始化一个AclLiteImageProc类对象     | acllite/src/AclLiteImageProc.cpp<br> src/detectPreprocess/detectPreprocess.cpp | AclLiteImageProc::Init<br>DetectPreprocessThread::Init       |
  | 通过AclLiteImageProc类对象并使用缩放功能   | acllite/src/AclLiteImageProc.cpp<br> src/detectPreprocess/detectPreprocess.cpp | AclLiteImageProc::Resize<br>DetectPreprocessThread::ProcessPic |
  | 缩放功能的封装                             | acllite/src/ResizeHelper.cpp                                 | ResizeHelper::Process                                        |
  | AclLiteImageProc类对DVPP图片处理功能的管理 | acllite/src/AclLiteImageProc.cpp                             | AclLiteImageProc::Resize                                     |

- 抠图功能实现流程参考如下：

  1. 创建一个AclLiteImageProc类对象并将其初始化，该类内部主要负责管理DVPP图片处理相关功能的调用。

     代码文件：src/classifyPreprocess/classifyPreprocess.cpp

     代码示例：
            

     ```
     AclLiteError ClassifyPreprocessThread::Init() {
            
         AclLiteError aclRet = dvpp_.Init();
         if (aclRet) {
             ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
             return ACLLITE_ERROR;
         }
         return ACLLITE_OK;
     }  
     ```

  2. 通过AclLiteImageProc类对象，调用抠图功能相关功能接口，从目标图片数据中抠出期望得到的图片。

     代码文件：src/classifyPreprocess/classifyPreprocess.cpp

     代码示例：

     ```
     AclLiteError ClassifyPreprocessThread::MsgProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
         // if last frame
         if(carDetectDataMsg->isLastFrame == 1)
             return ACLLITE_OK;
         // if car detected
         carDetectDataMsg->flag = 0;
         //No car detected
         if (carDetectDataMsg->carInfo.size() == 0) {
             carDetectDataMsg->flag = 1;
             return ACLLITE_OK;
         }
         // copy data from host to dvpp
         ImageData imageDevice;
         AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
         if (ret == ACLLITE_ERROR) {
             ACLLITE_LOG_ERROR("Copy image to device failed");
             return ACLLITE_ERROR;
         }
            
         // crop car area from pic
         ret = Crop(carDetectDataMsg->carInfo, imageDevice);
         if (ret) {
             ACLLITE_LOG_ERROR("Crop all the data failed, all the data failed");
             return ACLLITE_ERROR;
         }
                
         ...
     
         return ACLLITE_OK;
     } 
     ```

     ```
     AclLiteError ClassifyPreprocessThread::Crop(vector<CarInfo> &carImgs, ImageData &orgImg) {
         static int cnt = 0;
         AclLiteError ret = ACLLITE_OK;
         for (int i = 0; i < carImgs.size(); i++) {
             // crop area from pic，coordinate is detec model infer result
             ret = dvpp_.Crop(carImgs[i].cropedImgs, orgImg,
                              carImgs[i].rectangle.lt.x, carImgs[i].rectangle.lt.y,
                              carImgs[i].rectangle.rb.x, carImgs[i].rectangle.rb.y);                                    
             if (ret) {
                 ACLLITE_LOG_ERROR("Crop image failed, error: %d, image width %d, "
                                   "height %d, size %d, crop area (%d, %d) (%d, %d)", 
                                   ret, carImgs[i].cropedImgs.width, carImgs[i].cropedImgs.height,                            
                                   carImgs[i].cropedImgs.size, carImgs[i].rectangle.lt.x, 
                                   carImgs[i].rectangle.lt.y, carImgs[i].rectangle.rb.x, 
                                   carImgs[i].rectangle.rb.y);
                 return ACLLITE_ERROR;
             }
         }
         return ret;
     }
     ```

   关键功能接口列表：

  | **业务场景**                               | **代码文件**                                                 | **接口/结构体**                                              |
  | ------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | 创建并初始化一个AclLiteImageProc类对象     | acllite/src/AclLiteImageProc.cpp<br> src/detectPreprocess/detectPreprocess.cpp | AclLiteImageProc::Init()<br>DetectPreprocessThread::Init()   |
  | 通过AclLiteImageProc类对象并使用抠图功能   | acllite/src/AclLiteImageProc.cpp<br>src/classifyPreprocess/classifyPreprocess.cpp | AclLiteError AclLiteImageProc::Crop<br>ClassifyPreprocessThread::Crop |
  | 抠图功能的封装                             | acllite/src/CropAndPasteHelper.cpp                           | CropAndPasteHelper::Process                                  |
  | AclLiteImageProc类对DVPP图片处理功能的管理 | acllite/src/AclLiteImageProc.cpp                             | AclLiteImageProc::Crop                                       |

##### 定制开发

下面介绍几种常见场景下，如何基于本样例进行图片大小调整的定制开发：

- 若您直接使用样例中模型，或者模型要求的图片分辨率大小与样例模型一致，则可直接可复用样例代码，无需任何定制。

- 若您的模型对输入图片的大小要求与样例代码中的模型要求不一致，您可以直接复用样例代码，仅对相关接口的参数进行调整，详细操作如下。

  - 修改代码ProportionPasteCenter接口，将坐标点参数修改为期望缩放的区域坐标。

    请注意，对于ProportionPasteCenter接口，可参考下表，修改参数rbHorz, rbVert至替换后模型所需宽高即可。

    | 说明项 | 具体描述                                                     |
    | ------ | ------------------------------------------------------------ |
    | 函数   | AclLiteError ProportionPasteCenter(ImageData& dest, ImageData& src, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert) |
    | 功能   | 将图片等比例缩放到指定大小，且位于输出图片中央               |
    | 参数   | dest: 缩放后的图片<br>src: 待缩放图片<br>rbHorz: 缩放目标大小的宽度<br>rbVert: 缩放目标大小的高度 |

    | 备注   | ProportionPasteCenter()在内部封装了对齐操作，会对传入图片的宽高及坐标偏移值做自动化处理  

  - 修改代码resize接口，将width与height调整为模型要求的宽与高即可。

    请注意，对于resize接口，可参考下表，修改参数width, height至替换后模型所需宽高即可。

    | 说明项 | 具体描述                                                     |
    | ------ | ------------------------------------------------------------ |
    | 函数   | AclLiteError Resize(ImageData& dest,ImageData& src, uint32_t width, uint32_t height) |
    | 功能   | 将图片缩放到指定大小                                         |
    | 参数   | dest: 缩放后的图片<br>src: 待缩放图片<br>width: 缩放目标大小的宽度<br>height: 缩放目标大小的高度 |
    | 备注   | resize()在内部封装了对齐操作，使用的对齐参数为16x2           |

  - 修改代码文件中的crop接口，将坐标点参数修改为期望抠取的区域坐标。

    请注意，对于crop接口，可参考下表，修改参数ltHorz, ltVert, rbHorz, rbVert，重新选定待抠图区域即可。

    | 说明项 | 具体描述                                                     |
    | ------ | ------------------------------------------------------------ |
    | 函数   | AclLiteError Crop(ImageData& dest, ImageData& src, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert) |
    | 功能   | 抠图贴图，从原图抠出(ltHorz, ltVert)、(rbHorz, rbVert)两点确定的矩形区域,并贴至贴图区域(0, 0)(rbHorz-ltHorz, ltVert-rbVert) |
    | 参数   | dest：抠图贴图后图片数据<br>src：待处理图片数据<br>ltHorz：左上点的X坐标<br>ltVert：左上点的Y坐标<br>rbHorz：右下点的X坐标<br>rbVert：右下点的Y坐标 |
    | 备注   | Crop()在内部封装了对齐操作，会对传入图片的宽高及坐标偏移值做自动化处理 |

- 若您的模型对图片的要求与样例不一致，且不使用DVPP提供的图片预处理功能，则需要利用第三方库文件自行开发与如下接口功能相似的接口并进行替换。

  | **业务场景**               | **代码文件**                                                 | **接口/结构体**                                              |
  | -------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | 开发缩放功能接口的替换接口 | acllite/src/CropAndPasteHelper.cpp                           | CropAndPasteHelper::Process                                  |
  | 缩放接口的替换点           | src/detectPreprocess/detectPreprocess.cpp<br>src/classifyPreprocess/classifyPreprocess.cpp | DetectPreprocessThread::ProcessPic<br>ClassifyPreprocessThread::Resize |
  | 开发抠图功能接口的替换接口 | acllite/src/CropAndPasteHelper.cpp                           | CropAndPasteHelper::ProportionProcess                        |
  | 抠图接口的替换点           | src/classifyPreprocess/classifyPreprocess.cpp                | ClassifyPreprocessThread::Crop                               |


#### <a name="format_trans">格式转换</a>

CANN提供了DVPP进行图片预处理，但基于处理性能的考虑，DVPP对数据的输入、输出有一定的限制，且其输出格式通常为YUV420SP等格式。若DVPP的输出数据格式与模型要求的格式不一致，您可以使用CANN提供的AIPP功能对输入到模型的数据进行格式转换，AIPP作为对模型预处理的补充，可以满足更广泛场景的需要。

AIPP的功能需要在将开源模型转换为om离线模型时开启，如下所示：

```
atc --input_shape="input_1:10,224,224,3" --insert_op_conf=./aipp.cfg --output=./color_dvpp_10batch --soc_version=Ascend310 --framework=3 --model=./color.pb
```

其中insert_op_conf参数的输入就是AIPP的配置文件，AIPP的提供的所有功能都通过此配置文件承载。本节主要介绍如何通过AIPP实现数据格式的转换（又称色域转换），AIPP其他的功能请参考参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“ATC模型转换”的“高级功能 > AIPP使能”。

##### <a name="sample-analysis">样例解析</a>

- 本样例使用了AIPP功能将DVPP输出的YUV420SP_U8格式转换为RGB格式，AIPP配置文件的关键参数样例如下所示：

      ```
      aipp_op {
          aipp_mode: static
          input_format : YUV420SP_U8
          csc_switch : true
          rbuv_swap_switch : false
          matrix_r0c0 : 256
          matrix_r0c1 : 454
          matrix_r0c2 : 0
          matrix_r1c0 : 256
          matrix_r1c1 : -88
          matrix_r1c2 : -183
          matrix_r2c0 : 256
          matrix_r2c1 : 0
          matrix_r2c2 : 359
          input_bias_0 : 0
          input_bias_1 : 128
          input_bias_2 : 128
      }
      ```

  每一种格式转换都有官方模板供直接使用，详细可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“ATC模型转换”的“高级功能 > AIPP使能 > 配置文件模板”。

- 使用ATC工具进行离线模型转换时加载AIPP配置文件，即可对输入到模型中的数据进行格式转换。

  例如图片检测推理模型的模型转换：

  atc --model=./yolov3_t.onnx --framework=5 --output=yolov3 --input_shape="images:1,3,416,416;img_info:1,4" --soc_version=Ascend310 --input_fp16_nodes="img_info" --**insert_op_conf=aipp_onnx.cfg** 

##### 定制开发

1. 若您经过DVPP预处理后（或其他预处理），输入到模型的数据编码格式与模型要求不一致，您可以直接在模型转换时通过AIPP功能实现格式的转换，无需进行代码层面的修改。

   1. 首先获取使用AIPP功能前及AIPP功能需要输出的图片排布格式，然后参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“ATC模型转换”的“高级功能 > AIPP使能 > 配置文件模板”，选择相应的色域转换模板填入自行创建的AIPP配置文件，文件名为xxx.cfg。

   2. 使用ATC工具转换模型，并导入上一步准备好的AIPP配置文件。
      ATC转换命令的参考格式如下，其中insert_op_conf参数中配置的即为上一步骤创建的AIPP配置文件：

      ```
      atc --model=<model_file_path> --weight=<weight_file_path> --framework=<framework_ID> --insert_op_conf=<AIPP_file_path> --output=<om_file_path> --soc_version=<soc_version>
      ```

      例如，[样例解析](#sample-analysis)中的模型，若不导入AIPP配置文件，则模型只能接受RGB格式的图片数据，导入AIPP配置文件后，模型可接受YUV420SP_U8格式的数据。

2. 若您经过DVPP预处理后（或其他预处理），输入到模型的数据编码格式与模型要求不一致，若不使用AIPP功能，则您需要自行基于第三方库开发色域转换的功能接口，并添加到原预处理业务逻辑中。
   关键修改点列表：

   | **业务场景**                 | **代码文件**                                  | **接口/结构体**                      |
   | ---------------------------- | --------------------------------------------- | ------------------------------------ |
   | 图片检测推理模型的添加点     | src/detectPreprocess/detectPreprocess.cpp     | DetectPreprocessThread::ProcessPic   |
   | 车辆颜色分类推理模型的添加点 | src/classifyPreprocess/classifyPreprocess.cpp | ClassifyPreprocessThread::MsgProcess |


#### 图片预处理涉及代码文件汇总

本样例为两模型串接样例，且消息数据结构体相对复杂。在此以表格形式，分别梳理两模型预处理模块业务逻辑及样例消息数据结构。

- 消息数据结构： 

  | **结构体**       | **代码文件**    | **注释**                                                     |
  | ---------------- | --------------- | ------------------------------------------------------------ |
  | CarDetectDataMsg | inc/CarParams.h | inferThreadId：推理线程ID，标识消息将发送的线程。<br>detectPostThreadId：检测模型后处理线程ID，标识消息将发送的线程。<br>classifyPreThreadId：分类模型预处理线程ID，标识消息将发送的线程。<br>classifyPostThreadId：分类模型后处理线程ID，标识消息将发送的线程。<br>presentAgentDisplayThreadId：页面展示线程ID，标识消息将发送的线程。<br>deviceId：设备ID，区分设备配置参数。<br>isLastFrame：是否为最后一帧数据，0为不是，1为是。<br>frameNum：数据帧数，表示为该路第x帧数据。<br>imageFrame：自定义结构体数据，存放将原始图片解码为YUV420SP_U8格式后的数据，数据存放在dvpp内存上。<br>resizedFrame：自定义结构体数据，存放缩放至检测模型所需分辨率大小的图片数据，数据存放在dvpp内存上。<br>frame：OpenCV的Mat类数据，存放BGR格式的原始图片数据，分辨率与原图一致。用于后续将两模型推理结果画在其中。<br>detectInferData：检测模型推理结果。<br>carInfo：自定义结构体数据，用于存放检测模型的推理结果及分类模型需要使用的图片数据。<br>flag：判断是否有车辆被检测到，0为无，1为有。<br>classifyInferData：分类模型推理结果。 |
  | CarInfo          | inc/CarParams.h | cropedImgs：自定义结构体数据，存放从imageFrame抠出的有车辆区域。<br>resizedImgs：自定义结构体数据，将cropedImgs缩放至车辆颜色分类模型所需分辨率大小的图片数据。<br>rectangle：检测框，存放检测区域左上右下两点的坐标。<br>detect_result：检测模型推理结果。<br>carColor_result：车辆颜色分类模型推理结果。 |


- 检测模型预处理流程主要接口介绍：

  | **接口**                           | **代码文件**                              | **接口任务**                                                 |
  | ---------------------------------- | ----------------------------------------- | ------------------------------------------------------------ |
  | DetectPreprocessThread::Init       | src/detectPreprocess/detectPreprocess.cpp | 线程初始化接口，在线程被拉起时调用，除设置数据成员初值外，会读配置文件中输入类型数据参数到数据成员inputType_中，根据inputType_值在后续调用对应输入类型数据的接口。 |
  | DetectPreprocessThread::Process    | src/detectPreprocess/detectPreprocess.cpp | 线程主业务处理接口，根据接受的消息数据id，选择不同的处理流程或报错。 |
  | DetectPreprocessThread::MsgRead    | src/detectPreprocess/detectPreprocess.cpp | 读消息接口，根据inputType_值选择不同的读数据接口，图片场景下调用ReadPic接口。 |
  | DetectPreprocessThread::ReadPic    | src/detectPreprocess/detectPreprocess.cpp | 读图片接口，除设置消息数据将要发送的线程id初值外，也将原始图片分别用OpenCV接口和ReadJpeg接口读入到消息中。 |
  | DetectPreprocessThread::MsgProcess | src/detectPreprocess/detectPreprocess.cpp | 消息处理接口，根据inputType_值选择不同的消息数据处理接口，图片场景下调用ProcessPic接口。 |
  | DetectPreprocessThread::ProcessPic | src/detectPreprocess/detectPreprocess.cpp | 图片处理接口，将消息数据中的原始图片数据解码成YUV420SP_U8格式的数据，并将该数据缩放到车辆检测模型需要的分辨率大小。 |
  | DetectPreprocessThread::MsgSend    | src/detectPreprocess/detectPreprocess.cpp | 消息发送接口，将消息数据发送给推理线程，并送给车辆检测模型做推理；该接口会对是否为最后一张图片数据做区分。 |


- 车辆颜色分类模型预处理流程主要接口介绍： 

  | **接口**                             | **代码文件**                                  | **接口任务**                                                 |
  | ------------------------------------ | --------------------------------------------- | ------------------------------------------------------------ |
  | ClassifyPreprocessThread::Init       | src/classifyPreprocess/classifyPreprocess.cpp | 线程初始化接口，在线程被拉起时调用。                         |
  | ClassifyPreprocessThread::Process    | src/classifyPreprocess/classifyPreprocess.cpp | 线程主业务处理接口，根据接受的消息数据id，选择不同的处理流程或报错。 |
  | ClassifyPreprocessThread::MsgProcess | src/classifyPreprocess/classifyPreprocess.cpp | 消息处理接口，内部包含抠图及缩放操作。                       |
  | ClassifyPreprocessThread::Crop       | src/classifyPreprocess/classifyPreprocess.cpp | 抠图接口，根据车辆检测模型的推理结果，从原图中抠出车辆区域，并存储至消息数据。 |
  | ClassifyPreprocessThread::Resize     | src/classifyPreprocess/classifyPreprocess.cpp | 缩放接口，将抠出的车辆区域图片缩放至车辆颜色分类模型需要的分辨率大小，并存储至消息数据。 |
  | ClassifyPreprocessThread::MsgSend    | src/classifyPreprocess/classifyPreprocess.cpp | 消息发送接口，将消息数据发送给推理线程，并送给车辆颜色分类模型做推理。 |



### 离线视频

#### 简介

输入是视频文件的场景下，最终传入模型进行处理的仍是一帧帧的图片。本样例使用的模型支持的输入数据约束如下：

| **模型**                                      | **输入图片编码格式** | **输入图片分辨率** |
| --------------------------------------------- | -------------------- | ------------------ |
| 图片检测推理模型。基于onnx的yolov3模型        | BGR                  | 宽：416 高：416    |
| 车辆颜色分类推理模型。基于tensorflow的CNN模型 | RGB                  | 宽：224 高：224    |

若您想直接使用样例中的模型，但使用的离线视频文件不符合模型要求；或者是想使用自己的模型，而模型对输入数据的要求与样例的模型不一致。在这两者任一场景下，都可以参考本章节，对输入视频的预处理流程进行改造，使其符合实际业务要求。

**注：** 样例使用的离线模型，由于另外配置了AIPP文件进行了格式转换，所以在代码层面，图片数据是直接以YUV420SP_U8的格式送给模型做推理，相关内容请见本篇的[格式转换](#format_trans) 。

#### 样例解析

对于视频文件，本样例采用FFmpeg切帧+DVPP VEDC解码的设计，首先将一个视频文件转换为一帧帧的图片进行数据读取，然后再通过DVPP的VPC功能将图片数据处理为满足模型约束的数据。

- DVPP的VDEC功能描述及约束可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“推理应用开发”的“AscendCL API参考> 媒体数据预处理V1 > VDEC功能”。
- DVPP的VPC功能描述及约束可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“推理应用开发”的“AscendCL API参考 > 媒体数据预处理V1 > VPC功能”。

样例实现流程如下所示：

1. 设置配置文件，指定输入文件类型及文件路径。

   通过对配置文件参数的设置，指定第x路（即某个Device）输入数据类型为video，并填写视频文件路径，设置方法可参考[样例编译运行](#compile-run)。 

2. 使用FFmpeg对视频文件进行切帧，并使用DVPP的VDEC功能对视频进行解码。

   - 创建一个AclLiteVideoProc类对象并用视频文件路径将其初始化。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     打开视频文件所在路径并读取视频文件的代码示例：

     ```
     AclLiteError DetectPreprocessThread::Init() {
         // get input data path
         AclLiteError aclRet = GetInputDataPath(inputDataPath_, deviceId_);
         if (aclRet != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("GetInputDataPath failed, error %d", aclRet);
             return ACLLITE_ERROR;
         }
         // get input data type
         aclRet = GetInputDataType(inputType_, deviceId_);
         if (aclRet != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("GetInputDataType failed, error %d", aclRet);
             return ACLLITE_ERROR;
         }
        
         if (inputType_ == "pic") {
             aclRet = OpenPicsDir();
         }
         // init AclLiteVideoProc
         else{ 
             aclRet = OpenVideoCapture();
         }  
         if (aclRet != ACLLITE_OK) {
             return ACLLITE_ERROR;
         }
         ...
        
     }
     ```

     读取视频文件函数OpenVideoCapture的实现示例：

     ```
     AclLiteError DetectPreprocessThread::OpenVideoCapture() {
         // create instance 
         if (IsRtspAddr(inputDataPath_)) {
             cap_ = new AclLiteVideoProc(inputDataPath_);
         } else if (IsVideoFile(inputDataPath_)) {          // offline video file
             if (!IsPathExist(inputDataPath_)) {
                 ACLLITE_LOG_ERROR("The %s is inaccessible", inputDataPath_.c_str());
                 return ACLLITE_ERROR;
             }
             cap_ = new AclLiteVideoProc(inputDataPath_);
         } else {
             ACLLITE_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                             " video file or camera id");
             return ACLLITE_ERROR;
         }
         // init
         if(!cap_->IsOpened()) {
             delete cap_;
             ACLLITE_LOG_ERROR("Failed to open video");
             return ACLLITE_ERROR;
         }
        
         return ACLLITE_OK;
     }
     ```

   - 通过AclLiteVideoProc类对象，调用"读"接口，从视频文件切下一帧数据并对其进行解码，从而获得一帧YUV420SP数据。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例：

     ```
     AclLiteError DetectPreprocessThread::ReadStream(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         ···
         // read frame from video to msg
         AclLiteError ret = cap_->Read(carDetectDataMsg->imageFrame);
         if (ret != ACLLITE_OK) {
             if(ret == ACLLITE_ERROR_DECODE_FINISH) {
             //not error，the last frame
             carDetectDataMsg->isLastFrame = 1;
             return ACLLITE_OK;
             } 
             else {
             // error
             carDetectDataMsg->isLastFrame = 1;
             return ret;
             }
         }
         // calculate frame number
         frameCnt_++;
         return ACLLITE_OK;
     }
     ```

3. 对解码后的视频帧数据进行预处理

   上一步将视频数据解码成了一帧帧图片数据，若解码后的图片数据仍与模型需要不符，则需要继续对图片数据进行预处理，详细方法可参见[图片预处理](#picture-process)，对数据进行抠图、缩放等处理。

#### 定制点分析

此处仅分析视频文件的读取及解码的定制点，视频文件解码成一帧帧图片后的后续定制，请参见[图片预处理](#picture-process)。

1. 若输入的视频文件满足VDEC功能约束。

   只需要修改配置文件，更换输入数据即可，视频文件的读取及解码部分代码无需做其他更改。

2. 若输入的视频文件不满足VDEC功能约束，则需要您自行使用第三方库，开发对应功能的接口并进行替换，涉及代码参考如下。

   视频预处理涉及代码文件汇总

   | **接口**                                   | **代码文件**                              | **接口任务**                                    |
   | ------------------------------------------ | ----------------------------------------- | ----------------------------------------------- |
   | DetectPreprocessThread::OpenVideoCapture() | src/detectPreprocess/detectPreprocess.cpp | 创建并初始化一个AclLiteVideoProc类对象          |
   | DetectPreprocessThread::ReadStream         | src/detectPreprocess/detectPreprocess.cpp | 从输入视频读取一帧帧yuv数据，并设置消息数据初值 |
   | VideoCapture::Read                         | acllite/srcsrc/VideoCapture.cpp           | 从解码队列中读取解码完成数据                    |
   | VideoCapture::FrameDecodeThreadFunction    | acllite/srcsrc/VideoCapture.cpp           | 调用ffmpeg切帧+vdec解码功能接口                 |
   | FFmpegDecoder::Decode                      | acllite/srcsrc/VideoCapture.cpp           | FFmpeg切帧功能封装                              |
   | VdecHelper::Process                        | acllite/srcsrc/VdecHelper.cpp             | VDEC功能的封装                                  |



### RTSP视频流

#### 简介

本样例使用的模型支持的输入数据约束如下：

| **模型**                                      | **输入图片编码格式** | **输入图片分辨率** |
| --------------------------------------------- | -------------------- | ------------------ |
| 图片检测推理模型。基于onnx的yolov3模型        | BGR                  | 宽：416 高：416    |
| 车辆颜色分类推理模型。基于tensorflow的CNN模型 | RGB                  | 宽：224 高：224    |

若您想直接使用样例中的模型，但使用的rtsp流不符合模型要求；或者是想使用自己的模型，而模型对输入数据的要求与样例的模型不一致。在这两者任一场景下，都可以参考本章节，对输入视频的预处理流程进行改造，使其符合实际业务要求。

**注：** 样例使用的离线模型，由于另外配置了AIPP文件进行了格式转换，所以在代码层面，图片数据是直接以YUV420SP_U8的格式送给模型做推理，相关内容请见本篇的[格式转换](#format_trans) 。

#### 样例解析

对于视频文件，本样例采用FFmpeg进行切帧，使用DVPP的VEDC进行解码。

首先将RTSP视频流转换为一帧帧的图片进行数据读取，然后再通过DVPP的VPC功能将图片数据处理为满足模型约束的数据。

- DVPP的VDEC功能描述及约束可参见[Ascend文档中心](https://www.hiascend.com/document?data=community-developer)推理应用开发”的“AscendCL API参考> 媒体数据预处理V1 > VDEC功能”。
- DVPP的VPC功能描述及约束可参见[Ascend文档中心](https://www.hiascend.com/document?data=community-developer)的“推理应用开发”的“AscendCL API参考 > 媒体数据预处理V1 > VPC功能”。

样例实现流程如下所示：

1. 设置配置文件，指定输入文件类型及文件路径。

   通过对配置文件参数的设置，指定第x路（即某个Device）输入数据类型为rtsp，并填写rtsp流地址，设置方法可参考[样例编译运行](#compile-run)。

2. FFmpeg切帧+VDEC解码，由于AclLiteVideoProc类对处理视频文件和rtsp流的实现接口做了统一，因此在接口调用层面，读视频和读rtsp流共用一套接口。

   - 创建一个AclLiteVideoProc类对象并用rtsp流地址路径将其初始化。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     读取rtsp视频流的代码示例：

     ```
     AclLiteError DetectPreprocessThread::Init() {
         // get input data path
         AclLiteError aclRet = GetInputDataPath(inputDataPath_, deviceId_);
         if (aclRet != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("GetInputDataPath failed, error %d", aclRet);
             return ACLLITE_ERROR;
         }
         // get input data type
         aclRet = GetInputDataType(inputType_, deviceId_);
         if (aclRet != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("GetInputDataType failed, error %d", aclRet);
             return ACLLITE_ERROR;
         }
        
         ...
         // init AclLiteVideoProc
         else{
             aclRet = OpenVideoCapture();
         }
         if (aclRet != ACLLITE_OK) {
             return ACLLITE_ERROR;
         }
         ...
        
     }
     ```

     读取视频流函数OpenVideoCapture的实现示例：

     ```
     AclLiteError DetectPreprocessThread::OpenVideoCapture() {
         // create instance 
         if (IsRtspAddr(inputDataPath_)) {
             cap_ = new AclLiteVideoProc(inputDataPath_);
         } else if (IsVideoFile(inputDataPath_)) {
             if (!IsPathExist(inputDataPath_)) {
                 ACLLITE_LOG_ERROR("The %s is inaccessible", inputDataPath_.c_str());
                 return ACLLITE_ERROR;
             }
             cap_ = new AclLiteVideoProc(inputDataPath_);
         } else {
             ACLLITE_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                             " video file or camera id");
             return ACLLITE_ERROR;
         }
         // init
         if(!cap_->IsOpened()) {
             delete cap_;
             ACLLITE_LOG_ERROR("Failed to open video");
             return ACLLITE_ERROR;
         }
        
         return ACLLITE_OK;
     }
     ```

   - 通过AclLiteVideoProc类对象，调用"读"接口，从rtsp流切下一帧数据并解码，从而获得一帧YUV420SP数据。

     代码文件：src/detectPreprocess/detectPreprocess.cpp

     代码示例：

     ```
     AclLiteError DetectPreprocessThread::ReadStream(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         ···
         // read frame from rtsp to msg
         AclLiteError ret = cap_->Read(carDetectDataMsg->imageFrame);
         if (ret != ACLLITE_OK) {
             if(ret == ACLLITE_ERROR_DECODE_FINISH) {
             // not error，the last frame
             carDetectDataMsg->isLastFrame = 1;
             return ACLLITE_OK;
             } 
             else {
             // error
             carDetectDataMsg->isLastFrame = 1;
             return ret;
             }
         }
         // calculate frame number
         frameCnt_++;
         return ACLLITE_OK;
     }
     ```

3. 对解码后的视频帧数据进行预处理。

   上一步将视频数据解码成了一帧帧图片数据，若解码后的图片数据仍与模型需要不符，则需要继续对图片数据进行预处理，详细方法可参见[图片预处理](#picture-process)，对数据进行抠图、缩放等处理。

#### 定制点分析

此处仅分析RTSP视频流的读取及解码的定制点，视频流解码成一帧帧图片后的后续定制，请参见[图片预处理](#picture-process)。

1. 若输入的rtsp视频流满足VDEC功能约束。

   只需要修改配置文件，更换输入数据即可，视频流的读取及解码部分代码无需做其他更改。

2. 若输入的rtsp视频流不满足VDEC功能约束，则需要您自行使用第三方库，开发对应功能的接口并进行替换，涉及代码参考如下。

   rtsp流预处理涉及代码文件汇总

   | **接口**                                   | **代码文件**                              | **接口任务**                                    |
   | ------------------------------------------ | ----------------------------------------- | ----------------------------------------------- |
   | DetectPreprocessThread::OpenVideoCapture() | src/detectPreprocess/detectPreprocess.cpp | 创建并初始化一个AclLiteVideoProc类对象          |
   | DetectPreprocessThread::ReadStream         | src/detectPreprocess/detectPreprocess.cpp | 从输入视频读取一帧帧yuv数据，并设置消息数据初值 |
   | VideoCapture::Read                         | acllite/srcsrc/VideoCapture.cpp           | 从解码队列中读取解码完成数据                    |
   | VideoCapture::FrameDecodeThreadFunction    | acllite/srcsrc/VideoCapture.cpp           | 调用ffmpeg切帧+vdec解码功能接口                 |
   | FFmpegDecoder::Decode                      | acllite/srcsrc/VideoCapture.cpp           | FFmpeg切帧功能封装                              |
   | VdecHelper::Process                        | acllite/srcsrc/VdecHelper.cpp             | VDEC功能的封装                                  |


## <a name="data-postprocess">数据后处理</a>

### 简介

数据后处理指获得模型推理结果后，如何对数据进行进一步处理，例如将推理结果画框标注、以及将带有推理结果的图片数据以不同形式保存等。

CANN未提供封装的数据后处理相关接口，需要用户根据模型推理结果和业务需要自行实现。

### 样例解析

本样例中，用户可以通过配置输出数据类型，选择不同的数据输出格式，例如如视频、图片等。下面我们详细介绍样例代码是如何实现通过读参数，使样例自动地选择后处理方式的，并对不同后处理方式的实现进行简单介绍。

1. 配置输出类型相关参数。

   样例目录下的“scripts/params.conf”配置文件，提供了outputType_x参数，用于配置第X+1路的输出数据类型，此参数当前支持的配置项有：pic，video，presentagent。

   - pic：表示输出结果为图片。
   - video：表示输出结果为MP4视频文件。
   - presentagent：表示用PresentAgent展示推理结果，当前仅支持一路展示。

   如果outputType_X为video，还需要另外配置```outputFrameWidth_X```、```outputFrameHeight_0```。

   如果outputType_X为presentagent，则运行样例前需要先启动present server。

   配置示例如下所示：

    ```
    [base_options]
    device_num=1
    RtspNumPerDevice=1 

    [options_param_0]
    inputType_0=video  #pic ; video ; rtsp
    outputType_0=video  #pic ; video ; presentagent 
    inputDataPath_0=../data/mp4/car1.mp4
    outputFrameWidth_0=1280
    outputFrameHeight_0=720
    
    #[options_param_1]
    #inputType_1 = video
    #outputType_1 = presentagent
    #inputDataPath_1=../data/car2.mp4
    #outputFrameWidth_1=2368
    #outputFrameHeight_1=1080
    ```

   此配置文件中的详细参数说明可参见[样例编译运行](#compile-run) 。 

2. 模型后处理模块识别输出类型参数。

   设计接口，根据outputType_X变量名，识别到某一路的后处理数据类型。

   - 存储后处理数据类型的变量到类的私有数据成员，以供后续访问。
   - 如果后处理数据类型为video，还需要另外调用接口，存储输出视频的分辨率到到类的私有数据成员，以供后续访问。

   代码文件：src/classifyPostprocess/classifyPostprocess.cpp
   代码示例：

    ```
    AclLiteError ClassifyPostprocessThread::GetOutputDataType(std::string& outputType, uint32_t deviceId) {
        std::string outputTypeKey = "outputType_" + to_string(deviceId);
        std::map<std::string, std::string> config;
        if(!ReadConfig(config, configFile_)) {
            return ACLLITE_ERROR;
        }
        // get output type value
        std::map<std::string, std::string>::const_iterator mIter = config.begin();
        for (; mIter != config.end(); ++mIter) {
            if (mIter->first == outputTypeKey) {
                outputType.assign(mIter->second.c_str());
                ACLLITE_LOG_INFO("device %d output type is : %s", 
                                 deviceId, outputType.c_str());
            }
        }
        // verify output type
        if (outputType.empty() || (outputType != "video" &&
            outputType != "pic" && outputType != "presentagent")) {
            ACLLITE_LOG_ERROR("device %d output type is invalid", deviceId);
            return ACLLITE_ERROR;     
        }
    
        return ACLLITE_OK;
    }
    ```

    输出数据类型为video时，获取并存储输出视频的分辨率。

    ```
    AclLiteError ClassifyPostprocessThread::GetOutputFrameResolution(int& frameWidth, int& frameHeight, uint32_t deviceId) {
        std::string outputFrameWidthKey = "outputFrameWidth_" + to_string(deviceId);
        std::string outputFrameHeightKey = "outputFrameHeight_" + to_string(deviceId);
    
        std::map<std::string, std::string> config;
        if(!ReadConfig(config, configFile_)) {
            return ACLLITE_ERROR;
        }
        
        std::map<std::string, std::string>::const_iterator mIter = config.begin();
        // set output video resolution
        for (; mIter != config.end(); ++mIter) {
            if (mIter->first == outputFrameWidthKey) {
                frameWidth = atoi(mIter->second.c_str());
                ACLLITE_LOG_INFO("video %d width %d", 
                                 deviceId, frameWidth);
            } else if (mIter->first == outputFrameHeightKey) {
                frameHeight = atoi(mIter->second.c_str());
                ACLLITE_LOG_INFO("video %d height %d", 
                                 deviceId, frameHeight);
            }
        }
    
        return ACLLITE_OK;
    }
    ```

3. 提取并解析模型推理结果：

   不同的模型需要分别编写并调用推理结果解析接口。

   - 车辆检测模型

     代码文件：src/detectPostprocess/detectPostprocess.cpp

     代码示例： 

     ```
     AclLiteError DetectPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
         if (carDetectDataMsg->isLastFrame) 
             return ACLLITE_OK;
      
         float* detectData = (float *)carDetectDataMsg->detectInferData[kBBoxDataBufId].data.get();
         if(detectData == nullptr){
             ACLLITE_LOG_ERROR("detect inferoutput is null\n");
             return ACLLITE_ERROR;
         }
         uint32_t* boxNum = (uint32_t *)carDetectDataMsg->detectInferData[kBoxNumDataBufId].data.get();
         uint32_t totalBox = boxNum[0];
     
         for (uint32_t i = 0; i < totalBox; i++) {
             uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
             if (score < 60) {
                 continue;
             }
             uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
             if (objIndex == 2){
                 CarInfo carInfo;
                 carInfo.rectangle.lt.x = detectData[totalBox * TOPLEFTX + i];
                 carInfo.rectangle.lt.y = detectData[totalBox * TOPLEFTY + i];
                 carInfo.rectangle.rb.x = detectData[totalBox * BOTTOMRIGHTX + i];
                 carInfo.rectangle.rb.y = detectData[totalBox * BOTTOMRIGHTY + i];
                 uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
                 carInfo.detect_result = yolov3Label[objIndex] + std::to_string(score) + "\%";
                 carDetectDataMsg->carInfo.emplace_back(carInfo);
             }
         }
         return ACLLITE_OK;
     }
     ```

   - 颜色分类模型
     代码文件：src/classifyPostprocess/classifyPostprocess.cpp
     代码示例：

     ```
     AclLiteError ClassifyPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
         ...
         void* data = (void *)carDetectDataMsg->classifyInferData[0].data.get();
         if (data == nullptr) {
             return ACLLITE_ERROR;
         }
         float* outData = NULL;
         outData = reinterpret_cast<float*>(data);
     
         for(int i = 0; i < carDetectDataMsg->carInfo.size(); i++){
             int maxConfidentIndex = i * kEachResultTensorNum;
             for(int j = 0; j < kEachResultTensorNum; j++){
                 int index = i * kEachResultTensorNum + j;
                 // Select the highest confidence level
                 if(outData[index] > outData[maxConfidentIndex]){
                     maxConfidentIndex = index;
                 }
             }
             // get index
             int colorIndex = maxConfidentIndex - i * kEachResultTensorNum;
             // get color
             carDetectDataMsg->carInfo[i].carColor_result = kCarColorClass[colorIndex];
         }
      
     ...
     }
     ```

4. 将模型推理结果贴至原图，并根据输出类型参数，将数据按不同格式保存至本地：

   代码文件：src/classifyPostprocess/classifyPostprocess.cpp

   代码示例：

    ```
    AclLiteError ClassifyPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
        ...
        AclLiteError ret;
        if (outputType_ == "video") {
            ret = DrawResultOnVideo(carDetectDataMsg);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Draw classify result on video failed, error %d", ret);
                return ACLLITE_ERROR;
            }        
        }
        else if (outputType_ == "presentagent") {
            ret = SendImage(carDetectDataMsg);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Send image to presentAgent failed, error %d", ret);
                return ACLLITE_ERROR;
            }
        }
        else {
            ret = DrawResultOnPic(carDetectDataMsg);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Send image to presentAgent failed, error %d", ret);
                return ACLLITE_ERROR;
            }        
        }
    
        return ACLLITE_OK;
    }
    ```


### 定制点开发

下面介绍**更换模型**、**修改输出数据类型**两种定制场景下您需要完成的工作，由于这两种场景为并列关系，所以分别介绍其所需要的修改，如果您的业务需要考虑这两种场景同时存在的情况，也可相结合参考，完成后处理功能模块的定制开发。

- 模型替换场景：当您需要更换模型，且模型推理结果解析逻辑与样例模型不同时，可参考如下思路进行定制。

  1. 根据替换后模型的后处理逻辑，开发对应的推理结果解析接口。
  2. 在后处理模块的消息处理函数中，替换调用新开发的后处理解析接口。

  代码文件：src/classifyPostprocess/classifyPostprocess.cpp

  代码示例：

  ```
  AclLiteError DetectPostprocessThread::Process(int msgId, shared_ptr<void> data) {
      AclLiteError ret = ACLLITE_OK;
      switch(msgId) {
          case MSG_DETECT_INFER_OUTPUT:
              // analyse inference result
              InferOutputProcess(static_pointer_cast<CarDetectDataMsg>(data));
              MsgSend(static_pointer_cast<CarDetectDataMsg>(data));
              break;
          default:
              ACLLITE_LOG_INFO("Detect PostprocessThread thread ignore msg %d", msgId);
              break;
      }
    
      return ret;
  }
  ```

- 增加/修改输出数据类型的场景。

  1. 修改获取输出类型的接口，增加/修改对有效参数的识别：

     代码文件：src/classifyPostprocess/classifyPostprocess.cpp

     代码示例：

     ```
     AclLiteError ClassifyPostprocessThread::GetOutputDataType(std::string& outputType, uint32_t deviceId) {
         std::string outputTypeKey = "outputType_" + to_string(deviceId);
         std::map<std::string, std::string> config;
         if(!ReadConfig(config, configFile_)) {
             return ACLLITE_ERROR;
         }
      
         std::map<std::string, std::string>::const_iterator mIter = config.begin();
         for (; mIter != config.end(); ++mIter) {
             if (mIter->first == outputTypeKey) {
                 outputType.assign(mIter->second.c_str());
                 ACLLITE_LOG_INFO("device %d output type is : %s", 
                                  deviceId, outputType.c_str());
             }
         }
         // check value
         if (outputType.empty() || (outputType != "video" &&
             outputType != "pic" && outputType != "presentagent")) {
             ACLLITE_LOG_ERROR("device %d output type is invalid", deviceId);
             return ACLLITE_ERROR;     
         }
      
         return ACLLITE_OK;
     }
     ```

  2. 根据输出类型参数，自行判断是否需要开发另外的初始化参数设置接口（非必需）：

     代码文件：src/classifyPostprocess/classifyPostprocess.cpp

     代码示例：

     ```
     AclLiteError ClassifyPostprocessThread::SetOutputVideo() {
         stringstream sstream;
         sstream.str("");
         sstream << "../out/output/out_test" << deviceId_<<".mp4";
         outputVideo_.open(sstream.str(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 25.0, cv::Size(outputFrameWidth_,outputFrameHeight_));
         return ACLLITE_OK;
     }
     ```

     ```
     AclLiteError ClassifyPostprocessThread::GetOutputFrameResolution(int& frameWidth, int& frameHeight, uint32_t deviceId) {
         std::string outputFrameWidthKey = "outputFrameWidth_" + to_string(deviceId);
         std::string outputFrameHeightKey = "outputFrameHeight_" + to_string(deviceId);
      
         std::map<std::string, std::string> config;
         if(!ReadConfig(config, configFile_)) {
             return ACLLITE_ERROR;
         }
        
         std::map<std::string, std::string>::const_iterator mIter = config.begin();
         for (; mIter != config.end(); ++mIter) {
             if (mIter->first == outputFrameWidthKey) {
                 frameWidth = atoi(mIter->second.c_str());
                 ACLLITE_LOG_INFO("video %d width %d", 
                                  deviceId, frameWidth);
             } else if (mIter->first == outputFrameHeightKey) {
                 frameHeight = atoi(mIter->second.c_str());
                 ACLLITE_LOG_INFO("video %d height %d", 
                                  deviceId, frameHeight);
             }
         }
      
         return ACLLITE_OK;
     }
     ```

  3. 开发新的数据类型输出接口，并在解析完模型推理结果后，根据输出数据类型调用接口进行结果输出。

     代码文件：src/classifyPostprocess/classifyPostprocess.cpp

     代码示例：

     ```
     AclLiteError ClassifyPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
         ...
         AclLiteError ret;
         if (outputType_ == "video") {
             ret = DrawResultOnVideo(carDetectDataMsg);
             if (ret != ACLLITE_OK) {
                 ACLLITE_LOG_ERROR("Draw classify result on video failed, error %d", ret);
                 return ACLLITE_ERROR;
             }        
         }
         else if (outputType_ == "presentagent") {
             ret = SendImage(carDetectDataMsg);
             if (ret != ACLLITE_OK) {
                 ACLLITE_LOG_ERROR("Send image to presentAgent failed, error %d", ret);
                 return ACLLITE_ERROR;
             }
         }
         else {
             ret = DrawResultOnPic(carDetectDataMsg);
             if (ret != ACLLITE_OK) {
                 ACLLITE_LOG_ERROR("Send image to presentAgent failed, error %d", ret);
                 return ACLLITE_ERROR;
             }        
         }
      
         return ACLLITE_OK;
     }
     ```

     输出图片类型数据：

     ```
     AclLiteError ClassifyPostprocessThread::DrawResultOnPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         ···
         stringstream sstream;
         sstream.str("");
         sstream << "../out/output/device_" << carDetectDataMsg->deviceId << "_out_pic_" << carDetectDataMsg->frameNum << ".jpg";
         cv::imwrite(sstream.str(), carDetectDataMsg->frame);
         return ACLLITE_OK;
     }
     ```

     输出视频类型数据：

     ```
     AclLiteError ClassifyPostprocessThread::DrawResultOnVideo(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {  
         ...
         resize(carDetectDataMsg->frame, carDetectDataMsg->frame, 
                 cv::Size(outputFrameWidth_,outputFrameHeight_),
                 0, 0, cv::INTER_LINEAR);
         outputVideo_ << carDetectDataMsg->frame;
         return ACLLITE_OK;
     }
     ```

     将带有推理结果的图片数据发送给presentagent展示线程：

     ```
     AclLiteError ClassifyPostprocessThread::SendImage(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
         ...
         AclLiteError ret = DisplayMsgSend(carDetectDataMsg);
         if (ret != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("Send display msg failed");
             return ACLLITE_ERROR;
         }
      
         return ACLLITE_OK;
     }
     ```

### 后处理业务流程代码及接口说明

- 车辆检测模型：

  | **接口**           | **代码文件**                                | **接口功能**                                                 |
  | ------------------ | ------------------------------------------- | ------------------------------------------------------------ |
  | Process            | src/detectPostprocess/detectPostprocess.cpp | 消息数据处理接口，线程主干接口，用来处理检测模型线程发送的数据 |
  | InferOutputProcess | src/detectPostprocess/detectPostprocess.cpp | 检测解析模型的推理结果                                       |
  | MsgSend            | src/detectPostprocess/detectPostprocess.cpp | 将存放解析完推理结果数据的消息发送给分类模型预处理线程       |

- 颜色分类模型：

  | **接口**                 | **代码文件**                                    | **接口功能**                                             |
  | ------------------------ | ----------------------------------------------- | -------------------------------------------------------- |
  | GetOutputDataType        | src/classifyPostprocess/classifyPostprocess.cpp | 识别输出数据类型的配置参数                               |
  | GetOutputFrameResolution | src/classifyPostprocess/classifyPostprocess.cpp | 输出数据类型为video时，识别输出视频的分辨率参数          |
  | SetOutputVideo           | src/classifyPostprocess/classifyPostprocess.cpp | 设置输出视频的文件名                                     |
  | InferOutputProcess       | src/classifyPostprocess/classifyPostprocess.cpp | 解析模型推理结果并根据输出数据类型调用不同的数据输出接口 |

  |DrawResultOnPic|src/classifyPostprocess/classifyPostprocess.cpp|将模型推理结果绘制在原图上并保存为图片|+
  |DrawResultOnVideo|src/classifyPostprocess/classifyPostprocess.cpp|将模型推理结果绘制在原图上并保存为视频|
  |SendImage|src/classifyPostprocess/classifyPostprocess.cpp|将模型推理结果绘制在原图上并将数据发送给presentagent展示线程|



## 动态Batch

### 简介

Batch即为每次模型推理处理的图片数，动态Batch代表执行推理时，模型每次处理的图片数量是不固定的。
例如，检测出目标后再执行目标识别网络的场景，由于目标个数不固定导致目标识别网络每次处理的图片个数不固定。如果每次推理都按照最大的BatchSize进行计算，会造成计算资源浪费。因此，存在推理需要支持动态Batch的场景。

当前通用目标识别样例未覆盖动态Batch的场景，若您需要支持动态Batch，需要自行对样例代码进行改造，本节介绍动态Batch的实现流程及关键样例代码，有助于您针对样例代码进行动态Batch场景的改造。

**须知：**  动态Batch与动态分辨率场景不支持同时存在。

### 流程说明

动态Batch场景下，首先需要将模型转换为支持该特性的离线模型文件，并设置不同档位的batch数；

然后在执行模型推理前，需要使用AscendCL的aclmdlSetDynamicBatchSize接口设置本次模型推理时的batch大小，并根据batch数创建对应大小的InputDataSet；

最后需对模型的后处理函数进行改造，使其能识别每次推理的batch数并进行结果解析。

### 实现步骤

下面仅介绍动态Batch场景需要关注的步骤的实现，其他未体现步骤同非动态Batch场景。

1. 将原始框架模型转换为支持动态Batch特性的离线om模型。

   ATC工具提供了转换参数```--dynamic_batch_size```，用于使能转换后的离线om模型支持动态Batch的特性。
   ```--dynamic_batch_size```参数需要与```--input_shape```参数配合使用，其中```--input_shape```设置模型支持的shape信息，动态Batch场景下，输入shape中的“Batch”设置为“-1”，代表Batch数不固定；而```--dynamic_batch_size```参数则用于设置动态Batch场景下“Batch”支持的档位数。
   例如：

   ```
   atc --model=<model_path> --weight=<weight_path> --framework=<origin_framework_ID> --output=<offlinemodel_path> --soc_version=<soc_version> --input_shape="data:-1,3,224,224"  --dynamic_batch_size="1,2,4,8"  
   ```

   其中，“--input_shape”中的“-1”表示第一维(Batch)的大小不固定，则需要设置支持的BatchSize，```-dynamic_batch_size="1,2,4,8"```表示输入shape的Batch支持取值分别为1、2、4、8四个档位，则在模型编译时，支持的输入组合档数分别为：

   - 第0档：data(1,3,224,224)
   - 第1档：data(2,3,224,224)
   - 第2档：data(4,3,224,224)
   - 第3档：data(8,3,224,224)

   ATC工具的动态Batch功能详细描述及使用约束可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“推理应用开发”的“ATC模型转换 > 参数说明 > 基础功能参数 > 输入选项 > --dynamic_batch_size”，如果在模型转换时出现问题，您可以到[modelzoo仓创建issue](https://github.com/Ascend/modelzoo/issues)。

2. 应用程序开发时，在模型执行前，需要设置模型推理时的实际batch大小，并创建对应batch大小的输入数据。

   用户需要根据本次推理的实际batch数，生成本次推理的模型输入。

   以下代码样例主要展示业务逻辑，出于代码简单易读性考虑，此处假设图片数据格式默认符合模型所需格式，且每张图片的大小一致，为batch=1时模型所需的图片大小。在实际实现和调用接口时，需结合业务场景进行设计开发：

   ```
   // read pic
   void ReadPicture(const string &picturePath)
   {
       ifstream binFile(picturePath, ifstream::binary);
       binFile.seekg(0, binFile.end);
       pictureDataSize = binFile.tellg();
       binFile.seekg(0, binFile.beg);
       // pictureData : pic data ; pictureDataSize : pic data size
       if (runMode == ACL_HOST) {
           aclError ret = aclrtMallocHost(&pictureData, pictureDataSize);
       }
       else{
           aclError ret = aclrtMalloc(&pictureData, pictureDataSize, ACL_MEM_MALLOC_HUGE_FIRST);
       }
       binFile.read(static_cast<char *>(pictureData), pictureDataSize);
       binFile.close();
   }
   
   // copy all batch pic 
   void CopyData(int batchSize, uint32_t pos)
   {
       for(int i = 0; i < batchSize; i++)
       {
           string fileName = "xxx" + (to_string(i)) ;
           ReadPicture(fileName);
   
           aclError ret;
           if (pictureDeviceData == nullptr)
           {
               ret = aclrtMalloc(&pictureDeviceData, pictureDataSize * batchSize, ACL_MEM_MALLOC_HUGE_FIRST);
           }
           if (runMode == ACL_HOST) {
               ret = aclrtMemcpy((char *)pictureDeviceData + pos, pictureDataSize, \
               pictureData, pictureDataSize, ACL_MEMCPY_HOST_TO_DEVICE);
           }
           else
           {
               ret = aclrtMemcpy((char *)pictureDeviceData + pos, pictureDataSize, \
               pictureData, pictureDataSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
           }
           pos += pictureDataSize;
       }
       totalDataSize = pictureDataSize * fileCount;
   }
   
   void CreateModelInput()
   {
       aclError ret;
       modelDesc_ = aclmdlCreateDesc();
       ret = aclmdlGetDesc(modelDesc_, modelId_);
       input_ = aclmdlCreateDataset();
       for(size_t index = 0; index < aclmdlGetNumInputs(modelDesc_); ++index)
       {
           // get ACL_DYNAMIC_TENSOR_NAME index
           const char* name = aclmdlGetInputNameByIndex(modelDesc_, index);
           size_t inputLen = aclmdlGetInputSizeByIndex(modelDesc_, index);
           if(strcmp(name, ACL_DYNAMIC_TENSOR_NAME) == 0)
           {
               void *data = nullptr;
               ret = aclrtMalloc(&data, inputLen, ACL_MEM_MALLOC_HUGE_FIRST);
               batchBuffer = aclCreateDataBuffer(data, inputLen);
               ret = aclmdlAddDatasetBuffer(input_, batchBuffer);
           }
           else
           {
               // totalDataSize = pictureSize * batchSize
               inputBuffer = aclCreateDataBuffer(pictureDeviceData, totalDataSize);
               ret = aclmdlAddDatasetBuffer(input_, inputBuffer);
           }
       }
   
   }
   ```


   每次推理前，设置此次推理batch数：

   ```
   int  ModelSetDynamicInfo(int batchSize)
   {
        size_t index;
        // get index of dynamic batch in input dataset
        aclError ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME, &index);
        // set index
        ret = aclmdlSetDynamicBatchSize(modelId_, input_, index, batchSize);
   }

   ```


3. 对模型的后处理函数进行改造，使其能够识别每次推理的batch数，并根据输出的batch数解析模型推理结果。

   此处提供一个多batch和非多batch的后处理函数对比，以供参考:

   - batch=1：

     ```
     void PrintResult()
     {
         // copy inference result from device to host
         float* outFloatData;
         if (runMode == ACL_HOST) {
             aclError ret = aclrtMallocHost(&outputHostData, outputDataSize);
             ret = aclrtMemcpy(outputHostData, outputDataSize, outputDeviceData, outputDataSize, ACL_MEMCPY_DEVICE_TO_HOST);
             outFloatData = reinterpret_cast < float * > (outputHostData);
         }
         else
         {
             outFloatData = reinterpret_cast < float * > (outputDeviceData);
         }
      
         // print top-five label
         map<float, unsigned int, greater<float>> resultMap;
         for (unsigned int j = 0; j < outputDataSize / (sizeof(float) * 2); ++j)
         {
             resultMap[*outFloatData] = j;
             outFloatData++;
         }
         printf("=================Result of picture=================\n");
         int cnt = 0;
         for (auto it = resultMap.begin(); it != resultMap.end(); ++it)
         {
             if(++cnt > 5)
             {
                 break;
             }
     
             printf("top %d: index[%d] value[%lf] \n", cnt, it->second, it->first);
         }
     }
     ```

   - batch=n：

     ```
     void PrintResult(int batchSize)
     {
         float* outFloatData;
         if (runMode == ACL_HOST) {
             aclError ret = aclrtMallocHost(&outputHostData, outputDataSize);
             ret = aclrtMemcpy(outputHostData, outputDataSize, outputDeviceData, outputDataSize, ACL_MEMCPY_DEVICE_TO_HOST);
             outFloatData = reinterpret_cast < float * > (outputHostData);
         }
         else
         {
             outFloatData = reinterpret_cast < float * > (outputDeviceData);
         }
         // pics inference result is continuous in memory 
         for(int i = 0;i < batchSize; i++)
         {
             map<float, unsigned int, greater<float>> resultMap;
             // print top-five label
             for (unsigned int j = 0; j < outputDataSize / (sizeof(float) * 2); ++j)
             {   
                 // outFloatData++ ：i * j times
                 resultMap[*outFloatData] = j;
                 outFloatData++;
             }
             printf("=================Result of picture %d=================\n", (i+1));
             int cnt = 0;
             for (auto it = resultMap.begin(); it != resultMap.end(); ++it)
             {
                 if(++cnt > 5)
                 {
                     break;
                 }
     
                 printf("top %d: index[%d] value[%lf] \n", cnt, it->second, it->first);
             }
         }
     }
     ```

## 动态分辨率

### 简介

动态分辨率，表示用户执行推理时，模型支持的图片的分辨率可变。适用于执行推理时，每次处理的图片宽和高不固定的场景。

当前通用目标识别样例未覆盖动态分辨率的场景，若您需要支持动态分辨率，需要自行对样例代码进行改造，本节介绍动态分辨率的实现流程及关键样例代码，有助于您针对样例代码进行动态分辨率场景的改造。

 **须知：** 动态分辨率与动态Batch场景不支持同时存在。

### 流程说明

动态分辨率场景下，首先需要将模型转换为支持该特性的离线模型文件，并设置支持的不同档位的分辨率参数；

然后在执行模型推理前，需要使用AscendCL的aclmdlSetDynamicHWSize接口设置本次模型推理时的图片分辨率大小。并根据分辨率大小创建对应分辨率大小的InputDataSet；

最后也需对模型的后处理函数进行改造，使其能识别每次推理输出的图片分辨率并进行结果解析。

### 实现步骤

下面仅介绍动态分辨率场景需要关注的步骤的实现，其他未体现步骤同非动态分辨率场景。

1. 将原始框架模型转换为支持动态分辨率特性的离线om模型。

   ATC工具提供了转换参数```--dynamic_image_size```，用于使能转换后的离线om模型支持动态分辨率的特性。   
   “--dynamic_image_size”参数需要与```--input_shape```配合使用，动态分辨率场景下，```--input_shape```的Height与Width设置为“-1”，代表高宽不固定，“--dynamic_image_size”参数则设置模型支持的图片分辨率组合，每组参数之间使用英文分号分隔，组内参数使用英文逗号分隔，例如“height1,width1;height2,width2”。

   调用示例：

   ```
   atc --model=<model_path> --weight=<weight_path> --framework=<origin_framework_ID> --output=<offlinemodel_path> --soc_version=<soc_version> ----input_shape="data:1（batches）,3（channels）,-1,-1"  --dynamic_image_size="416,416;1280,640"  
   ```

   则在模型编译时，支持的输入shape组合档数分别为：

   - 第0档：data(1,3,416,416)
   - 第1档：data(1,3,1280,640)


   ATC工具的动态分辨率功能详细描述及使用约束可参见[Ascend文档中心](https://www.hiascend.com/document?tag=community-developer)的“推理应用开发”的“ATC模型转换 > 参数说明 > 基础功能参数 > 输入选项 > --dynamic_image_size”，如在模型转换时出现问题，您可以到[modelzoo仓创建issue](https://github.com/Ascend/modelzoo/issues)。

2. 模型推理前设置此次模型推理时的图片size大小。

   以下代码仅供参考：

   ```
   int  ModelSetDynamicInfo()
   {
       size_t index;
       // get index of dynamic resolution，signed by ACL_DYNAMIC_TENSOR_NAME
       aclError ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME, &index);
       // set height and width
       uint64_t height = 224;
       uint64_t width = 224;
       ret = aclmdlSetDynamicHWSize(modelId_, input_, index, height, width);
   }
   ```


   ```
   int ModelExecute(int index)
   {
       aclError ret;
       // set dynamic resolution before execute
       ret = ModelSetDynamicInfo();
       ret = aclmdlExecute(modelId_, input_, output_);
       // ......
   }
   ```

3. 如果模型后处理函数涉及到对图片宽高的读取，则需要实时地读取图片宽高，不能写为固定的常量。

   此处提供一个固定分辨率和动态分辨率的后处理函数对比，以供参考:

   - 分辨率固定：

   ```
     AclLiteError DetectPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
         if (carDetectDataMsg->isLastFrame) 
             return ACLLITE_OK;
    
         float* detectData = (float *)carDetectDataMsg->detectInferData[kBBoxDataBufId].data.get();
         if(detectData == nullptr){
             ACLLITE_LOG_ERROR("detect inferoutput is null\n");
             return ACLLITE_ERROR;
         }
         uint32_t* boxNum = (uint32_t *)carDetectDataMsg->detectInferData[kBoxNumDataBufId].data.get();
         uint32_t totalBox = boxNum[0];
    
         // get pic scale factor
         // kModelWidth KModelHeight fixed
         float widthScale = (float)(carDetectDataMsg->imageFrame.width) / kModelWidth;
         float heightScale = (float)(carDetectDataMsg->imageFrame.height) / kModelHeight;
    
         // get box location
         for (uint32_t i = 0; i < totalBox; i++) {
             uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
             if (score < 90) {
                 continue;
             }
             CarInfo carInfo;
             carInfo.rectangle.lt.x = detectData[totalBox * TOPLEFTX + i] * widthScale;
             carInfo.rectangle.lt.y = detectData[totalBox * TOPLEFTY + i] * heightScale;
             carInfo.rectangle.rb.x = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
             carInfo.rectangle.rb.y = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
             uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
             carInfo.detect_result = yolov3Label[objIndex] + std::to_string(score) + "\%";
             carDetectDataMsg->carInfo.emplace_back(carInfo);
         }
         return ACLLITE_OK;
     }
   ```

   - 分辨率可变：

     ```
     AclLiteError DetectPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
         float* detectData = (float *)carDetectDataMsg->detectInferData[kBBoxDataBufId].data.get();
         if(detectData == nullptr){
             ACLLITE_LOG_ERROR("detect inferoutput is null\n");
             return ACLLITE_ERROR;
         }
         uint32_t* boxNum = (uint32_t *)carDetectDataMsg->detectInferData[kBoxNumDataBufId].data.get();
         uint32_t totalBox = boxNum[0];
        
         // get pic scale factor
         // modelWidth modelHeight flexible
         float widthScale = (float)(carDetectDataMsg->imageFrame.width) / modelWidth;
         float heightScale = (float)(carDetectDataMsg->imageFrame.height) / modelHeight;
     
         // get box location
         for (uint32_t i = 0; i < totalBox; i++) {
             uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
             if (score < 90) {
                 continue;
             }
             CarInfo carInfo;
             carInfo.rectangle.lt.x = detectData[totalBox * TOPLEFTX + i] * widthScale;
             carInfo.rectangle.lt.y = detectData[totalBox * TOPLEFTY + i] * heightScale;
             carInfo.rectangle.rb.x = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
             carInfo.rectangle.rb.y = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
             uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
             carInfo.detect_result = yolov3Label[objIndex] + std::to_string(score) + "\%";
             carDetectDataMsg->carInfo.emplace_back(carInfo);
         }
         return ACLLITE_OK;
     }
     ```

# 性能提升

## 多路多线程

### 简介

本节主要介绍如何对样例进行多Device、多路、多线程的定制开发，这些都是用于提升设备资源利用率的手段。

多路是每一个Device都可以处理不同路数的视频，例如若有两个视频流同时输入，则可以开启多路特性同时处理，一定程度上保证了Device的利用率；多线程则保证了每一路Device的计算资源的利用率，本样例中涉及两个模型，对每个模型的预处理及后处理环节，样例分别拉起了一个业务线程；而推理操作相对预处理及后处理耗时较小，我们则只拉起一个推理线程，通过让两个模型的预处理线程都只往唯一的推理线程发送消息数据，提升了对推理资源的利用率。在实际使用场景中，您可以通过实际测得的数据，适当调整各环节的线程数目，以便进一步地提升资源利用率。

- 对于样例的多路修改，可以通过配置文件“scripts/params.conf”中的参数“device_num”做修改，实现对样例运行总路数的设置。其中每一路都表示一个完整的预处理—推理—后处理流程，每一路支持设置不同的输入及输出数据类型，但样例能够配置的最大路数等于硬件的Device个数。
- 当前一路业务，有预处理线程数2个、推理线程数1个、后处理线程数2个，累计线程数5个。如果最终的输出数据通过Present Agent进行网页展示，则还会另外拉起一个展示线程。
  对于样例的多线程修改，请参考本节的[定制点分析-修改/新增/删除线程的场景](#thread-modify) 中的内容。此外，样例中Present Agent展示功能为一个独立模块，需要根据实际场景单独判断展示线程是否被拉起。 

### 样例解析

下面详细介绍本样例的多路多线程的实现方式。

1. 设置多Device、多路参数。

   通过配置文件“scripts/params.conf”进行多Device、多路参数的配置。

   - 设置多Device：通过参数device_num设置执行推理的设备数。

   - 设置路数：通过参数RtspNumPerDevice设置每个Device支持的路数，同时需要匹配设置对应路的数据。

   **参数配置约束：** 

   - 配置的device_num需要小于等于设备的最大Device数。
   - 若开启了多路特性，即每个Device同时处理多路视频，输出方式仅支持单路配置为presentagent。

2. 读取配置文件中配置的路数，并拉起每一路的业务线程。

   - 功能：读取并解析配置文件中参数

     代码文件：src/main.cpp

     关键代码：

     ```
     AclLiteError ParseConfig(uint32_t& deviceNum, uint32_t& rtspNumPerDevice) {
         map<string, string> config;
         if(!ReadConfig(config, kConfigFile)) {
             return ACLLITE_ERROR;
         }
     
         regex deviceNumRegex(kRegexDeviceNum.c_str());
         regex RtspNumPerDeviceRegex(kRegexRtspNumPerDevice.c_str());
         map<string, string>::const_iterator mIter = config.begin();
         for (; mIter != config.end(); ++mIter) {
             if (regex_match(mIter->first, deviceNumRegex)) {
                 deviceNum = stoi(mIter->second);
                 ACLLITE_LOG_INFO("Data config item: %s=%s", 
                                mIter->first.c_str(), mIter->second.c_str());
             }else if(regex_match(mIter->first, RtspNumPerDeviceRegex)){
               rtspNumPerDevice = stoi(mIter->second);
                 ACLLITE_LOG_INFO("Data config item: %s=%s", 
                                mIter->first.c_str(), mIter->second.c_str());
             }
         }
      return ACLLITE_OK;
     }
     ```

  ```
   - 功能：根据读取到的路数，创建对应个数的线程实例
   
     代码文件：src/main.cpp
   
     关键代码：
  ```

     void CreateThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev) {
         uint32_t deviceNum;
         uint32_t rtspNumPerDevice;
         runMode = aclDev.GetRunMode();
     
         AclLiteError ret = ParseConfig(deviceNum, rtspNumPerDevice);
         if (ret != ACLLITE_OK) {
             return;
         }
         kExitCount = deviceNum * rtspNumPerDevice;
     
         for(int32_t i=0; i < deviceNum; i++){
             ret = aclrtSetDevice(i);
             if (ret != ACL_ERROR_NONE) {
                 ACLLITE_LOG_ERROR("Acl open device %d failed", i);
                 return;
             }
             ret = aclrtCreateContext(&context, i);
             if (ret != ACL_ERROR_NONE) {
                 ACLLITE_LOG_ERROR("Create acl context failed, error:%d", ret);
                 return;
             }
             kContext.push_back(context);
             CreateInstances(threadTbl, i, context, runMode, rtspNumPerDevice);
         }
      ...
     }

  ```
   - 功能：判断是否需要使用Present Agent进行结果展示
   
     代码文件：src/main.cpp
   
     关键代码：
  ```

     void SetDisplay() {
         uint32_t deviceNum;
         AclLiteError ret = ParseConfig(deviceNum);
         if (ret != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("Parse config fail in GetDisplayInfo");
         }
         map<string, string> config;
         if(!ReadConfig(config, kConfigFile)) {
             ACLLITE_LOG_ERROR("read config fail in GetDisplayInfo");
         }
         string outputTypeKey, outputType;
         uint32_t presentAgentNum = 0;
         // judge presentagent thread whether needed or not
         for(int32_t i=0; i < deviceNum; i++){
             string outputTypeKey = "outputType_" + to_string(i);
             map<string, string>::const_iterator mIter = config.begin();
             for (; mIter != config.end(); ++mIter) {
                 if (mIter->first == outputTypeKey) {
                     outputType.assign(mIter->second.c_str());
                 }
             }
             if (!outputType.empty() && outputType == "presentagent") {
                 kDisplay = true;
                 break;
             }
      }
     }

  ```
   - 功能：若需要Present Agent进行结果展示展示，需要创建展示线程
   
     创建展示线程的代码文件：src/main.cpp
   
     关键代码：
  ```

     void CreateThreadInstance(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev) {
         ...
         if (kDisplay) {
             AclLiteThreadParam presentAgentDisplayThreadParam;
             presentAgentDisplayThreadParam.threadInst = new PresentAgentDisplayThread(kPresenterChannel);
             presentAgentDisplayThreadParam.threadInstName.assign(kPresentAgentDisplayName.c_str());
             presentAgentDisplayThreadParam.context = context;
             presentAgentDisplayThreadParam.runMode = runMode;
          threadTbl.push_back(presentAgentDisplayThreadParam);
         }
     }
     ```

3. 拉起所有线程，并发送启动信号。

   代码文件：src/main.cpp

   ```
   void StartApp(AclLiteResource& aclDev) {
       vector<AclLiteThreadParam> threadTbl;
       CreateThreadInstance(threadTbl, aclDev);
       AclLiteApp& app = CreateAclLiteAppInstance();
       // init instance in table
       AclLiteError ret = app.Start(threadTbl);
       if (ret != ACLLITE_OK) {
           ACLLITE_LOG_ERROR("Start app failed, error %d", ret);
           ExitApp(app, threadTbl);
           return;
       }
       // send start signal to all threads
       for (int i = 0; i < threadTbl.size(); i++) {
           ret = SendMessage(threadTbl[i].threadInstId, MSG_APP_START, nullptr);
       }
   ```

### 定制点分析

主要包括路数修改和线程修改两种定制场景，由于两者为并列关系，我们分别对两种场景独立分析。

- 路数修改场景。
  路数的修改可通过修改配置文件“scripts/params.conf”中的参数实现。具体方式及约束可参考文档[样例编译运行](#compile-run)。

- <a name="thread-modify">修改/新增/删除线程的场景</a>
  本篇将以样例中的InferenceThread为例，介绍如何从AclLiteThread类继承派生出一个业务子类inference，并将其初始化值传入列表中，并根据列表初始化一个AclLiteApp类对象，即一个管理多线程应用的对象的完整流程。

  1. 继承AclLiteThread类，生成一个业务子类InferenceThread。

     代码文件：src/inference/inference.h

     关键代码：

     ```
     // subclass InferenceThread 
     class InferenceThread : public AclLiteThread {
     public:
         InferenceThread(aclrtRunMode& runMode);
         ~InferenceThread();
         // init 
         AclLiteError Init();
         // main interface，finish msg process
         AclLiteError Process(int msgId, shared_ptr<void> data);
     private:
         AclLiteError InitModelInput();
         AclLiteError DetectMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
         AclLiteError DetectModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
         AclLiteError ClassifyModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg); //wait
         AclLiteError ClassifyMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
         int CopyOneBatchImages(uint8_t* buffer, uint32_t bufferSize, 
                                vector<CarInfo> &carImgs, int batchIdx);
         int CopyImageData(uint8_t *buffer, uint32_t bufferSize, ImageData& image);
         void DestroyResource();
     private:   
         AclLiteModel detectModel_;
         AclLiteModel classifyModel_;
         aclrtRunMode runMode_;
         int32_t batchSize_;
         uint32_t classifyInputSize_;
         uint8_t* classifyInputBuf_;
         uint32_t imageInfoSize_;
         void*    imageInfoBuf_;         
     };
     
     ```

  2. Process函数实现。

     Process为数据处理主函数，负责分析接收到的消息数据，并发往对应接口做处理，处理完后再发往业务下端的线程，保证线程间的串连。在基类AclLiteThread中， 该接口为虚函数，因此必须在子类InferenceThread中对该接口进行实现。实现样例如下：

     代码文件：src/inference/inference.cpp

     关键代码：

     ```
     AclLiteError InferenceThread::Process(int msgId, shared_ptr<void> data) {
         switch(msgId) {
             // send msg data to different api, according to msgId
             case MSG_DETECT_PREPROC_DATA:
                 //detect model process msg
                 DetectModelExecute(static_pointer_cast<CarDetectDataMsg>(data));
                 //send processed msg to detect postprocess
                 DetectMsgSend(static_pointer_cast<CarDetectDataMsg>(data));
                 break;
        
             case MSG_CLASSIFY_PREPROC_DATA:
                 //classify model process msg
                 ClassifyModelExecute(static_pointer_cast<CarDetectDataMsg>(data));
                 //send processed msg to classify postprocess
                 ClassifyMsgSend(static_pointer_cast<CarDetectDataMsg>(data));
                 break;
             default:
                 ACLLITE_LOG_INFO("Inference thread ignore msg %d", msgId);
                 break;
       }
     
         return ACLLITE_OK;
        
     }
     
     ```

  3. 为实现的子类InferenceThread，提供一个或多个对应的消息数据ID，以便在线程间通信时，区分消息数据收/发来源。

     代码文件：<a name="car_params.h">inc/CarParams.h</a>

     关键代码：

     ```
     ...
     namespace {
     ...
     const int MSG_DETECT_PREPROC_DATA = 3;
     ...
     const int MSG_CLASSIFY_PREPROC_DATA = 6;
     ...
     }
     ...
     ```

  4. 为实现的子类InferenceThread，提供一组对应的字符串变量，以便在创建线程时，保证线程名全局唯一。

     代码文件：inc/CarParams.h

     关键代码：

     ```
     namespace {
     ...
     const std::vector<std::string> kInferName = {"inference_0", "inference_1", "inference_2", "inference_3"};
     }
     ...
     
     ```

 5. 设置初始化InferenceThread对象的参数值，并将参数添加到线程初始化参数表中。

    代码文件：src/main.cpp

    关键代码：

    ```
    void CreateInstances(vector<AclLiteThreadParam>& threadTbl, int32_t i,
                         aclrtContext& context, aclrtRunMode& runMode) {
        ...
        AclLiteThreadParam InferParam;
        InferParam.threadInst = new InferenceThread(runMode);
        InferParam.threadInstName.assign(kInferName[i].c_str());
        InferParam.context = context;
        InferParam.runMode = runMode;
        // add params into table
        threadTbl.push_back(InferParam);
        ...
    }
    ```

  6. 设置好初始化线程的参数列表，创建AclLiteApp类对象及线程并向各线程发送消息拉起线程。

     代码文件：src/main.cpp

     关键代码：

     ```
     void StartApp(AclLiteResource& aclDev) {
         vector<AclLiteThreadParam> threadTbl;
         // init thread params table
         CreateThreadInstance(threadTbl, aclDev);
         AclLiteApp& app = CreateAclLiteAppInstance();
         // generate app&thread
         AclLiteError ret = app.Start(threadTbl);
         if (ret != ACLLITE_OK) {
             ACLLITE_LOG_ERROR("Start app failed, error %d", ret);
             ExitApp(app, threadTbl);
             return;
         }
     
         for (int i = 0; i < threadTbl.size(); i++) {
             //activate threads
             ret = SendMessage(threadTbl[i].threadInstId, MSG_APP_START, nullptr);
         }
     
         app.Wait(MainThreadProcess, nullptr);
         ExitApp(app, threadTbl);
     
         return;
     }
     ```

### <a name="other">Present Agent网页展示流程</a>

存在场景需要对Present Agent展示页面做分块展示修改时，可以参考本节了解Present Agent网页展示业务流程。

1. 判断是否需要拉起网页展示线程：

   代码文件：src/main.cpp

   关键代码：

   ```
   void SetDisplay() {
       //if need display
       if (kDisplay){
           break;
       }
       uint32_t deviceNum;
       AclLiteError ret = ParseConfig(deviceNum);
       if (ret != ACLLITE_OK) {
           ACLLITE_LOG_ERROR("Parse config fail in SetDisplay");
       }
       map<string, string> config;
       if(!ReadConfig(config, kConfigFile)) {
           ACLLITE_LOG_ERROR("read config fail in SetDisplay");
       }
       string outputTypeKey, outputType;
       uint32_t presentAgentNum = 0;
       for(int32_t i=0; i < deviceNum; i++){
           // search target output type
           string outputTypeKey = "outputType_" + to_string(i);
           map<string, string>::const_iterator mIter = config.begin();
           for (; mIter != config.end(); ++mIter) {
               if (mIter->first == outputTypeKey) {
                   outputType.assign(mIter->second.c_str());
               }
           }
           // set kDisplay true
           if (!outputType.empty() && outputType == "presentagent") {
               kDisplay = true;
           }
       }
   }
   ```

2. 样例在向Present Agent发送消息数据前，为带有推理结果的数据打上标记。

   代码文件：src/presentagentDisplay/presentagentDisplay.cpp

   关键代码：

   ```
   AclLiteError PresentAgentDisplayThread::DisplayMsgPackage(ImageFrame& packageMsg, shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
       ...
       // set presentagent identifier
       vector<DetectionResult> detection_results;
       DetectionResult one_result;
       one_result.result_text.append("device-");
       one_result.result_text.append(to_string(carDetectDataMsg->deviceId));
       detection_results.emplace_back(one_result);
       ...
   }
   
   ```

3. Present Agent网页的前端代码会将展示页面分为若干块（当前为四块）。

   代码文件：display/presenterserver/display/ui/templates/view.html

   关键代码：

   ```
   // set canvas resolution，canvas_x，canvas_y
   canvas.setAttribute("width",1280)
   canvas.setAttribute("height",720)
   ...
   if(current_channel == 0){
   // block_nums : x * y 
   // x_var : canvas_x/x
   // y_var : canvas_y/y
   ctx.drawImage(img,40,20, wantedWidth, img.height*scale_factor)
   }
   else if(current_channel == 1){
   ctx.drawImage(img,680,20, wantedWidth, img.height*scale_factor)
   }
   else if(current_channel == 2){
   ctx.drawImage(img,40,380, wantedWidth, img.height*scale_factor)
   }
   else{ 
   ctx.drawImage(img,680,380, wantedWidth, img.height*scale_factor)
   }
   ```

4. Present Agent网页的前端代码会识别数据中的标记，并将其至于对应区块进行展示；因此，假设device_num配置为4，第一路和第四路都采取Present Agent网页展示的形式输出推理结果，那么Present Agent网页页面，则只有第一块和第四块区域有画面。

   代码文件：display/presenterserver/display/ui/templates/view.html

   关键代码：

   ```
   var channel_tmp = rectangles[0].slice(4,5).toString();
   var current_channel = parseInt(channel_tmp.split("device-")[1].trim())
   ```

5. 因此，当需要修改展示路数时，可以自行参考上方代码及备注，设置网页展示区域及宽高，自行划块。

6. Present Agent网页展示业务代码

   | **业务说明**                           | **业务代码文件**                                       | **业务接口**                                                 |
   | -------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------------ |
   | 标记推理数据                           | src/presentagentDisplay/presentagentDisplay.cpp        | DisplayMsgPackage()                                          |
   | 网页识别标记                           | display/presenterserver/display/ui/templates/view.html | ...<br>var channel_tmp = rectangles[0].slice(4,5).toString();<br>var current_channel = parseInt(channel_tmp.split("device-")[1].trim())<br>... |
   | 展示页面划块，并根据标记贴图至指定区域 | display/presenterserver/display/ui/templates/view.html | ...<br>canvas.setAttribute("width",1280)<br>canvas.setAttribute("height",720)<br>...<br>if(current_channel == 0){<br> ctx.drawImage(img,40,20, wantedWidth, img.height*scale_factor)<br>}<br>                  else if(current_channel == 1){<br>ctx.drawImage(img,680,20, wantedWidth, img.height*scale_factor)<br>}<br>else if(current_channel == 2){<br>ctx.drawImage(img,40,380, wantedWidth, img.height*scale_factor)<br>}<br>else{<br>ctx.drawImage(img,680,380, wantedWidth, img.height*scale_factor)<br>}<br>... |

**为方便展示，基础代码提供的是单路展示功能，并没有将画面进行分块。**

## 模型压缩

昇腾模型压缩工具（Ascend Model Compression Toolkit，简称AMCT），是一个对昇腾AI处理器亲和的深度学习模型压缩工具包，提供了量化、稀疏等多种模型压缩特性，压缩后模型体积变小、达到轻量化，部署到昇腾AI处理器上后可使能低比特运算，在不影响精度的前提下提高存储、计算效率，从而达到性能提升的目标。

AMCT工具支持对Caffe、TensorFlow、PyTorch、ONNX、MindSpore等框架的原始网络模型进行压缩，例如，可将fp32/fp16的类型量化为int8类型，在保证模型精度不损失的情况下，得到性能更好的模型。

若您的模型较大，且性能达不到预期，您可以使用AMCT工具对原始框架模型进行压缩，然后再进行离线模型转换并进行推理，详细的使用方法可参见[昇腾文档中心](https://www.hiascend.com/document?tag=community-developer)的“AMCT工具使用指导”。

 **需要注意：** 
针对轻量级模型，使用AMCT工具收益较小，且可能存在精度达不到预期的情况。

## 模型调优

CANN提供了自动化调优工具Auto Tune，可充分利用硬件资源对模型中的算子进行自动调优，从而提升模型执行的性能。
Auto Tune工具的使用方式简单，只需要在生成离线模型时打开auto_tune_mode开关，即可使用Auto Tune调优工具。调优后的结果会放在自定义知识库中，后续执行模型时只需要加载自定义知识库即可享受调优后的算子性能。

Auto Tune工具的详细使用方法可参见[昇腾文档中心](https://www.hiascend.com/document?tag=community-developer)中的“算子自动调优Auto Tune”使用指导。