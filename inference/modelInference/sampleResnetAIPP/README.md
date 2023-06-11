## 目录

  - [样例介绍](#样例介绍)
  - [获取源码包](#获取源码包)  
  - [第三方依赖安装](#第三方依赖安装)
  - [样例运行](#样例运行)
  - [其他资源](#其他资源)
  - [更新说明](#更新说明)
  - [已知issue](#已知issue)
    
## 样例介绍

以ResNet50网络模型为例，演示如何通过模型转换使能静态AIPP功能，使能AIPP功能后，若实际提供给模型推理的测试图片不满足要求（包括图片格式，图片尺寸等），经过模型转换后，会输出满足模型要求的图片，并将该信息固化到转换后的离线模型中（模型转换后AIPP功能会以aipp算子形式插入离线模型中）。

样例输入：图片。    
样例输出：打屏显示置信度TOP5的类别标识、置信度信息和相应的类别信息。

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
- opencv

  执行以下命令安装opencv(注:确保是3.x版本)
  ```
  sudo apt-get install libopencv-dev
  ```    

## 样例运行

  - 数据准备

    请从以下链接获取该样例的输入图片，放在data目录下。
        
    ```    
    cd $HOME/samples/inference/modelInference/sampleResnetAIPP/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg
    ```

  - ATC模型转换

    将ResNet-50原始模型转换为适配昇腾310处理器的离线模型（\*.om文件），放在model路径下。

    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd $HOME/samples/inference/modelInference/sampleResnetAIPP/model
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50.onnx
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50_CV/aipp.cfg
    atc --model=resnet50.onnx --framework=5 --output=resnet50 --input_shape="actual_input_1:1,3,224,224"  --soc_version=Ascend310  --insert_op_conf=aipp.cfg
    ```

  - 样例编译

    设置环境变量，配置程序编译依赖的头文件与库文件路径。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。
    ```
    export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
    export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
    ```

    执行以下命令，执行编译脚本，开始样例编译。
    ```
    cd $HOME/samples/inference/modelInference/sampleResnetAIPP/scripts
    bash sample_build.sh
    ```
  - 样例运行

    执行运行脚本，开始样例运行。
    ```
    bash sample_run.sh
    ```
  - 样例结果展示
    
    执行成功后，在屏幕上的关键提示信息示例如下，提示信息中的top1-5表示图片置信度的前5种类别、index表示类别标识、value表示该分类的最大置信度，class表示所属类别。这些值可能会根据版本、环境有所不同，请以实际情况为准：

    ```
    [INFO] The sample starts to run
    [INFO]  top 1: index[162] value[0.908666] class[beagle]
    [INFO]  top 2: index[161] value[0.089970] class[bassetbasset hound]
    [INFO]  top 3: index[166] value[0.000726] class[Walker houndWalker foxhound]
    [INFO]  top 4: index[167] value[0.000499] class[English foxhound]
    [INFO]  top 5: index[163] value[0.000053] class[bloodhound sleuthhound]
    [INFO] The program runs successfully
    ```

## 其他资源

以下资源提供了对ONNX项目和Renet50模型的更深入理解：

**ONNX**
- [GitHub: ONNX](https://github.com/onnx/onnx)

**Models**
- [Resnet50 - image classification](https://github.com/Ascend/ModelZoo-PyTorch/tree/master/ACL_PyTorch/built-in/cv/Resnet50_Pytorch_Infer)

**Documentation**
- [AscendCL Samples介绍](../README_CN.md)
- [使用AscendCLC API库开发深度神经网络应用](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha006/infacldevg/aclcppdevg/aclcppdevg_000000.html)
- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/02/17 | 新增sampleResnetAIPP/README.md |
  

## 已知issue

  暂无
