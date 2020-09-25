中文

**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**

# classification_dynamicbatch样例运行指导

本Sample实现了googlenet网络的推理功能，运行成功后简单打印成功信息。

### 软件准备

运行此Sample前，需要执行以下步骤获取源码包并转换模型。

1. 普通用户在开发环境中下载样例源码包

   **cd $HOME/AscendProjects     
   wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/classification_dynamicbatch.zip   
   unzip classification_dynamicbatch.zip**

2. 获取此应用中所需要的原始网络模型。

   获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到Ubuntu服务器的任意目录，例如：$HOME/models/classification_dynamicbatch。

   下载原始网络模型文件：

   [googlenet.prototxt](https://gitee.com/HuaweiAscend/models/blob/master/computer_vision/classification/googlenet/googlenet.prototxt)

   对应的权重文件：

   https://obs-model-ascend.obs.cn-east-2.myhuaweicloud.com/googlenet/googlenet.caffemodel

   **mkdir -p \\$HOME/models/classification_dynamicbatch**

   **cd \\$HOME/models/classification_dynamicbatch**

   将原始网络模型及其对应的权重文件存放到$HOME/models/classification_dynamicbatch目录下：

   ![](figures/模型转换1.png "模型转换1")  

3. 将原始网络模型转换为适配昇腾AI处理器的模型。

   1.设置环境变量

   命令行中输入以下命令设置环境变量。（仅在当前窗口生效）

   **export install_path=\$HOME/Ascend/ascend-toolkit/latest/x86_64-linux_gcc7.3.0**  

   **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  

   **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH**  

   **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64:\\$LD_LIBRARY_PATH**  

   **export ASCEND_OPP_PATH=\\${install_path}/opp**  

    ![](figures/模型转换2.png "模型转换2")  

   2.执行以下命令转换模型

   转换好的模型文件（**googlenet_dynamicbatch.om**）位于**$HOME/AscendProjects/classification_dynamicbatch/model**目录下。

   **atc --model=\\$HOME/models/classification_dynamicbatch/googlenet.prototxt --weight=\\$HOME/models/classification_dynamicbatch/googlenet.caffemodel --framework=0 --output=\\$HOME/AscendProjects/classification_dynamicbatch/model/googlenet_dynamicbatch --soc_version=Ascend310 --input_shape="data:-1,3,224,224" --dynamic_batch_size="1,2" --output_type=FP32 --input_format=NCHW**
   
    ![](figures/模型转换3.png "模型转换3")

## 环境配置   

**注：服务器上已安装OpenCV、交叉编译工具可跳过此步骤。**  

- 安装编译工具  

  **sudo apt-get install -y g++\-aarch64-linux-gnu g++\-5-aarch64-linux-gnu** 

- 安装OpenCV 
  请参考 **https://gitee.com/ascend/samples/tree/master/common/install_opencv/for_atlas200dk**    

##  样例运行

1.打开Mindstudio。

执行以下命令，打开Mindstduio，并选择classification_dynamicbatch工程。

**cd ~/MindStudio-ubuntu/bin**

**./MindStudio.sh**

打开后，选择“Open project”，打开classification_dynamicbatch工程。
![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/172207_62f86791_7985487.png "屏幕截图.png")

此时，工程的model文件夹下已经有对应的om模型了。    
![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/172227_5f74dfdc_7985487.png "屏幕截图.png")

2.编译

在**Mindstudio**的工具栏中点击**Build > Edit Build Configuration**。选择Target OS 为Centos7.6
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/172251_55be038e_7985487.png "屏幕截图.png")

之后点击**Build > Build > Build Configuration**，会在目录下生成build和out文件夹。
   ![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/172310_6a944499_7985487.png "屏幕截图.png")

3.运行

1. Mindstudio配置开发板RC连接。

   **注：此时默认开发板已经连接到开发环境了**

   在Mind Studio工具的工具栏中找到**Tools**按钮，单机**Device Manager**。

   点击Device Manager界面右上方的 **“+”** 按钮，填写**Host IP**，点击OK。
   

   看到Device Manager界面**Connetctivity**为**Yes**即为连接成功，点击**OK**即可。
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/172419_be101d37_7985487.png "屏幕截图.png")
在Mind Studio工具的工具栏中找到**Run**按钮，单击 **Run > Edit Configurations**。

在Command Arguments 中添加运行参数 ../data（输入图片的路径），之后分别点击Apply、OK。
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/172435_6fdef687_7985487.png "屏幕截图.png")

单击 **Run > Run 'classification_dynamicbatch'**，如下图，可执行程序已经在开发者板成功执行。
   ![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/172446_7cd441e9_7985487.png "屏幕截图.png")
