中文|[EN](README_EN.md)

# 文本检测与识别（输入：文本图片  输出：文本识别结果）

## 简介

本项目实现输入的任意文本图片，实时得到文件识别结果的过程。程序在华为Atlas200DK和AI加速云服务器上实现摄像头的输入图片，经过文本检测得到文本区域，传输到文本检测模型，对文本检测的结果区域进行文本识别，最终确定输入图片中文本的内容并输出结果。本项目旨在使读者对基于Atlas 200DK开发板构建文本检测与识别应用系统有比较全面的认识，为读者提供一个文本检测与识别相关应用在华为Atlas 200DK上部署的参考。


### 模型推理阶段：

1. **运行管理资源申请**：用于初始化系统内部资源，ACL固定的调用流程。
2. **加载模型文件并构建输出内存**：从文件加载离线模型数据，需要由用户自行管理模型运行的内存，根据内存中加载的模型获取模型的基本信息包含模型输入、输出数据的数据buffer大小；由模型的基本信息构建模型输出内存，为接下来的模型推理做好准备。
3. **获取文本并进行数据预处理**：从本地存放有文本的文件中循环读取文本数据，将根据字典将文本映射为词向量；然后构建模型的输入数值矩阵/张量。
4. **模型推理**：根据构建好的模型输入数据进行模型推理。
5. **解析推理结果**：根据模型输出，解析文本检测的结果，获取当前文本的识别结果及置信度。


## 总体设计

![总体设计](./for_atlas200dk_1.7x.0.0_c++/tf_total_sentiment/figures/full_pipline.png  "full_pipline.png")

功能：检测与识别摄像头中的文字，并在presenter界面中给出检测与识别的结果。

样例输入：树莓派摄像头。

样例输出：presenter界面展现检测与识别的结果。


### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../../environment)准备好环境。

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

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/TextRecognize。
   ## 模型地址
   pb文件地址：
   文本检测：
   **https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Model/AE/ATC%20Model/dbnet/dbnet.pb**

   文本识别：
   **https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Model/AE/ATC%20Model/crnn_static/crnn_static.pb**

   

3. 将原始模型转换为Davinci模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/TextRecognize**  

        **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/textrecognize/insert_op.cfg**

        文本检测模型转换
        **atc --model=$TextRecognize/model/${pb_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${first_model_name} --soc_version=Ascend310 --output_type=FP32 --input_shape="input_images:1,736,1312,3" --input_format=NHWC**
        文本识别模型转变
        **atc --model=$TextRecognize/model/${pb_model##*/} --framework=3 --output=${HOME}/models/${project_name}/${first_model_name} --soc_version=Ascend310 --output_type=FP32 --input_shape="new_input:1,32,100,3" --input_format=NHWC**
        
        本社区也提供转换好文本检测和识别的om模型
        om模型地址：
        文本检测：
        **https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Model/AE/ATC%20Model/dbnet/dbnet.om**
        文本识别：
        **https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Model/AE/ATC%20Model/crnn_static/crnn_static.om**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。
       文本检测模型
       **cp ./dbnet.om $HOME/samples/cplusplus/contrib/TextRecognzie/model/**
       文本识别模型
       **cp ./crnn_static.om $HOME/samples/cplusplus/contrib/TextRecognize/model/**

### 样例部署

1. 修改present相关配置文件。

    将样例目录下**scripts/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为开发环境中可以ping通运行环境的ip地址。   
        1. 开发环境中使用ifconfig查看可用ip。   
        2. 在开发环境中将**scripts/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为该ip地址。   
        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 1.开发环境和运行环境分离部署，一般使用配置的虚拟网卡ip，例如192.168.1.223。   
        > - 2.开发环境和运行环境合一部署，一般使用200dk固定ip，例如192.168.1.2。

  
 
2. 开发环境命令行中设置编译依赖的环境变量。

  
     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**  
 
     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**   
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 如果是20.0版本，此处 **DDK_PATH** 环境变量中的 **arm64-liunx** 应修改为 **arm64-linux_gcc7.3.0**。

3. 切换到TextRecognize目录，创建目录用于存放编译文件，例如，本文中，创建的目录为 **build/intermediates/host**。

    **cd $HOME/samples/cplusplus/contrib/TextRecognize**

    **mkdir -p build/intermediates/host**

4. 切换到 **build/intermediates/host** 目录，执行cmake生成编译文件。


      **cd build/intermediates/host**   
      **make clean**   
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

5. 执行make命令，生成的可执行文件main在 **TextRecognize/out** 目录下。

    **make**


### 样例运行

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
> - 以下出现的**xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2。

1. 执行以下命令,将开发环境的 **TextRecognize** 目录上传到运行环境中，例如 **/home/HwHiAiUser**。   

    **开发环境与运行环境合一部署，请跳过此步骤！**   

    **scp -r $HOME/samples/cplusplus/contrib/TextRecognize HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. 启动presenterserver并登录运行环境。    
        1. 开发环境中执行以下命令启动presentserver。   
            **cd $HOME/samples/cplusplus/contrib/TextRecognize**   
            **bash script/run_presenter_server.sh**   
        2. 执行以下命令登录运行环境。   
            **开发环境与运行环境合一部署，请跳过此步骤！**   
            **ssh HwHiAiUser@xxx.xxx.xxx.xxx** 


3. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。   
      **export LD_LIBRARY_PATH=**   
      **source ~/.bashrc**     
      **cd $HOME/samples/cplusplus/contrib/HandWrite/out**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。 
  
      **cd $HOME/TextRecognize/out**

    切换目录后，执行以下命令运行样例。

    **./main**

### 查看结果

1. 打开presentserver网页界面。

      打开启动Presenter Server服务时提示的URL即可。
      

2. 等待Presenter Agent传输数据给服务端，单击“Refresh“刷新，当有数据时相应的Channel 的Status变成绿色。

3. 单击右侧对应的View Name链接，查看结果




