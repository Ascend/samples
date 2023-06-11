
## 目录

- [目录](#目录)
- [样例介绍](#样例介绍)
- [获取源码包](#获取源码包)
- [第三方依赖安装](#第三方依赖安装)
- [样例运行](#样例运行)
- [其他资源](#其他资源)
- [更新说明](#更新说明)
- [已知issue](#已知issue)
    
## 样例介绍

使用yolov7模型对输入图片进行预测推理，后处理使用onnx构建cann的后处理算子模型进行加速处理，其中用到了两个cann算子进行图构建，包括YoloPreDetection，和YoloV5DetectionOutput算子，进行物体检测。在图片上给出物体标注框，类别以及置信度，并将结果打印到输出图片上。
  
样例输入：图片。    
样例输出：图片物体检测，并且在图片上给出物体标注框，类别以及置信度，最后保存。

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

- python-acllite

   设置第三方库以及PYTHONPATH环境变量。
    
   ```   
    echo 'export THIRDPART_PATH=$HOME/Ascend/ascend-toolkit/latest/thirdpart'>> ~/.bashrc    
    echo 'export PYTHONPATH=${THIRDPART_PATH}/acllitePY:$PYTHONPATH'>> ~/.bashrc
   ```

   执行以下命令使环境变量生效并创建文件夹
   
   ```     
    source ~/.bashrc
    mkdir -p ${THIRDPART_PATH}
   ```

   python-acllite库以源码方式提供，安装时将acllite目录拷贝到运行环境的第三方库目录

    ```
    cp -r ${HOME}/samples/inference/acllite/acllitePY ${THIRDPART_PATH}

    ```   
 - opencv

   执行以下命令安装opencv-python。
   ```
   pip3 install opencv-python
   ```
 - numpy

   执行以下命令安装numpy库。
   ```
   pip3 install numpy
   ``` 
 - onnx

   执行以下命令安装onnx库。
   ```
   pip3 install onnx
   ``` 
## 样例运行

  - 数据准备

    请从以下链接获取该样例的输入图片，放在data目录下。
        
    ```    
    cd $HOME/samples/inference/modelInference/sampleYOLOV7NMSONNX/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/sampleYOLOvxWithPostprocessOp/dog.jpg --no-check-certificate
    ```

  - ATC模型转换

    进入src文件夹运行以下命令构建后处理算子postprocess.onnx保存在model文件夹下

    ```
    python3.7 postProcessOperator.py
    ```

    将YOLOV7原始模型和后处理算子转换为适配昇腾310处理器的离线模型（\*.om文件），放在model路径下。
    
    --soc_version：昇腾AI处理器的版本。进入“CANN软件安装目录/compiler/data/platform_config”目录，".ini"文件的文件名即为昇腾AI处理器的版本，请根据实际情况选择。

    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd $HOME/samples/inference/modelInference/sampleYOLOV7NMSONNX/model
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/sampleYOLOvxWithPostprocessOp/yolov7x.onnx --no-check-certificate
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/sampleYOLOvxWithPostprocessOp/aipp.cfg --no-check-certificate
    atc --model=yolov7x.onnx --framework=5 --output=yolov7x --soc_version=Ascend310 --out_nodes="Conv_404:0;Conv_374:0;Conv_344:0" \
    --input_shape="images:1,3,640,640" --insert_op_conf=aipp.cfg
    atc --model=postprocess.onnx --soc_version=Ascend310 --output=postprocess --framework=5 --input_shape='img_info:1,4'                                                              
    ```

    后处理算子图构建如下
    
    ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/sampleYOLOvxWithPostprocessOp/a.png "image-20211028101534906.png")

  - 样例运行
    

    执行运行脚本，开始样例运行。
    ```
    bash sample_run.sh
    ```
  - 样例结果展示
    
    运行完成后，会在样例工程的out目录下生成推理后的图片，显示对比结果如下所示。
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/sampleYOLOvxWithPostprocessOp/out.jpg "image-20211028101534905.png")
   

## 其他资源

以下资源提供了对ONNX项目和YOLOV7模型的更深入理解：

**ONNX**
- [GitHub: ONNX](https://github.com/onnx/onnx)

**Models**
- [YOLOV7- object detect](https://github.com/Ascend/modelzoo-GPL/tree/master/built-in/ACL_Pytorch/Yolov7_for_Pytorch)

**Documentation**
- [AscendCL Samples介绍](../README_CN.md)
- [使用AscendCLC API库开发深度神经网络应用](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha006/infacldevg/aclcppdevg/aclcppdevg_000000.html)
- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/05/23 | sampleYOLOV7NMSONNX/README.md |
  

## 已知issue

  暂无
