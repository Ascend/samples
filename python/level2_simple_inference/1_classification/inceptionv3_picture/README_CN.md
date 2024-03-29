**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 图片InceptionV3分类样例
功能：使用InceptionV3模型对输入图片进行分类推理。    
样例输入：待推理的jpg图片。   
样例输出：推理后的jpg图片。

### 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。

获取Atlas 200I DK A2硬件设备。根据官方文档`https://www.hiascend.com/document/detail/zh/Atlas200IDKA2DeveloperKit/23.0.RC1/qs/qs_0003.html`获取SD卡，并完成一键制卡。

本样例将通过远程登录的方式在Atlas 200I DK A2上完成。

### 样例准备

1. 获取源码包。

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

2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  InceptionV3| 图片分类推理模型。是Pytorch框架模型。  |  请参考[https://github.com/Ascend/ModelZoo-PyTorch/tree/master/ACL_PyTorch/built-in/cv/InceptionV3_for_Pytorch](https://github.com/Ascend/ModelZoo-PyTorch/tree/master/ACL_PyTorch/built-in/cv/InceptionV3_for_Pytorch)目录中README.md下载原始模型。 |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    cd $HOME/samples/python/level2_simple_inference/1_classification/inceptionv3_picture/model
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV3/inceptionv3.onnx
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV3/aipp_inceptionv3_pth.config
    atc --model=./inceptionv3.onnx --framework=5 --output=InceptionV3 --soc_version=Ascend310B1 --insert_op_conf=./aipp_inceptionv3_pth.config --input_shape="actual_input_1:1,3,300,300" --input_format=NCHW
    ```

3. 获取样例需要的测试图片。
    ```
    执行以下命令，进入样例的data文件夹中，下载对应的测试图片。
    cd $HOME/samples/python/level2_simple_inference/1_classification/inceptionv3_picture/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV3/pic/rabit.jpg
    cd ../src
    ```

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **inceptionv3_picture** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r $HOME/samples/python/level2_simple_inference/1_classification/inceptionv3_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/inceptionv3_picture/src    
    ```

2. <a name="step_2"></a>运行样例。
   ```
   python3 classify.py ../data/
   ```

### 查看结果

运行完成后，会在样例工程的out目录下生成带推理结果的jpg图片，显示对比结果如下所示。
![输入图片说明](https://images.gitee.com/uploads/images/2021/1102/182408_a7f6f173_8070502.png "屏幕截图.png")

### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。