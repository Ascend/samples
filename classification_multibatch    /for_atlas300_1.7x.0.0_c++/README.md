中文|[English](README_EN.md)

**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**

# classification_multibatch样例运行指导

本Sample实现了googlenet网络的推理功能，运行成功后简单打印成功信息。

### 软件准备

运行此Sample前，需要执行以下步骤获取源码包并转换模型。

1. 普通用户在开发环境中下载样例源码包
        
        
     cd $HOME/AscendProjects     
     wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/classification_multibatch.zip
     unzip classification_multibatch.zip
        

2. 获取此应用中所需要的原始网络模型。

   下载原始网络模型及权重文件至Ubuntu下的任意目录。如$HOME/models.

   <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。

    参考[表 分类网络应用使用模型](#zh-cn_topic_0219108795_table19942111763710)获取此应用中所用到的原始网络模型及其对应的权重文件。

    **表 1**  分类网络应用使用模型

<a name="zh-cn_topic_0219108795_table19942111763710"></a>
<table><thead align="left"><tr id="zh-cn_topic_0219108795_row611318123710"><th class="cellrowborder" valign="top" width="11.959999999999999%" id="mcps1.2.4.1.1"><p id="zh-cn_topic_0219108795_p81141820376"><a name="zh-cn_topic_0219108795_p81141820376"></a><a name="zh-cn_topic_0219108795_p81141820376"></a>模型名称</p>
</th>
<th class="cellrowborder" valign="top" width="8.07%" id="mcps1.2.4.1.2"><p id="zh-cn_topic_0219108795_p13181823711"><a name="zh-cn_topic_0219108795_p13181823711"></a><a name="zh-cn_topic_0219108795_p13181823711"></a>模型说明</p>
</th>
<th class="cellrowborder" valign="top" width="79.97%" id="mcps1.2.4.1.3"><p id="zh-cn_topic_0219108795_p1717182378"><a name="zh-cn_topic_0219108795_p1717182378"></a><a name="zh-cn_topic_0219108795_p1717182378"></a>模型下载路径</p>
</th>
</tr>
</thead>
<tbody><tr id="zh-cn_topic_0219108795_row1119187377"><td class="cellrowborder" valign="top" width="11.959999999999999%" headers="mcps1.2.4.1.1 "><p id="zh-cn_topic_0219108795_p4745165253920"><a name="zh-cn_topic_0219108795_p4745165253920"></a><a name="zh-cn_topic_0219108795_p4745165253920"></a>googlenet</p>
</td>
<td class="cellrowborder" valign="top" width="8.07%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0219108795_p1874515218391"><a name="zh-cn_topic_0219108795_p1874515218391"></a><a name="zh-cn_topic_0219108795_p1874515218391"></a>图片分类推理模型。

是基于Caffe的GoogLeNet模型。</p>
</td>
<td class="cellrowborder" valign="top" width="79.97%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0219108795_p611318163718"><a name="zh-cn_topic_0219108795_p611318163718"></a><a name="zh-cn_topic_0219108795_p611318163718"></a>请参考<a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/googlenet" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/googlenet</a>目录中README.md下载原始网络模型文件及其对应的权重文件。</p>
</td>
</tr>
</tbody>
</table>

3. 将原始网络模型转换为适配昇腾AI处理器的模型。
   
   下边介绍的是使用ATC工具对网络模型进行转换的方法。如果本机没有配置过ATC工具，那么需要进行以下操作。

   1.设置环境变量

   打开Ubuntu终端，输入以下命令
   vim ~/.bashrc
   打开.bashrc文件，在最后边添加环境变量


    **export install_path=\$HOME/Ascend/ascend-toolkit/latest/x86_64-linux_gcc7.3.0**

    **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  

    **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH** 

    **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64:\\$LD_LIBRARY_PATH**

    **export ASCEND_OPP_PATH=\\${install_path}/opp**  






    
   2.完成ATC配置

   修改完成.bashrc之后，需要同步一下，在当前窗口输入以下指令

   `source ~/.bashrc`

   这样，当前窗口就完成同步，可以使用刚才修改过的环境变量。
   做完这些之后，输入指令atc --help,窗口会打印出ATC start working now等，说明ATC安装成功。如下图所示

### ![输入图片说明](https://images.gitee.com/uploads/images/2020/0918/192233_61f80ae1_7990837.png "屏幕截图.png")


    3.执行以下命令转换模型。这里需要注意原始模型和权重文件的路径。

    atc --model=\\$HOME/models/googlenet.prototxt --weight=\\$HOME/models/googlenet.caffemodel --output_type=FP32 --input_shape="data:2,3,224,224" --input_format=NCHW --output=\\$HOME/models/googlenet_multibatch --soc_version=Ascend310 --framework=0
    
    这条指令的参数设置可以参考

    https://support.huaweicloud.com/ti-atc-A300_3000_3010/altasatc_16_003.html

    转换成功后，终端会输出如下

![输入图片说明](https://images.gitee.com/uploads/images/2020/0918/192252_3d1e24b9_7990837.png "屏幕截图.png")
    
由于我们在--output参数中设置的是”--output=\\$HOME/models/googlenet_multibatch”,所以执行结束后还可以在models文件夹里看到生成的googlenet_multibatch.om

## 环境配置   

**注：服务器上已安装OpenCV、交叉编译工具可跳过此步骤。**  

- 安装编译工具  

  **sudo apt-get install -y g++\-aarch64-linux-gnu g++\-5-aarch64-linux-gnu** 

- 安装OpenCV 
  请参考 **https://gitee.com/ascend/samples/tree/master/common/install_opencv/for_atlas200dk**    

##  样例运行

1.打开打开classification_multibatch工程。
以Mind Studio安装用户在命令行进入安装包解压后的“MindStudio-ubuntu/bin”目录，如：$HOME/MindStudio-ubuntu/bin。执行如下命令启动Mind Studio。

./MindStudio.sh


![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/173800_ad83e919_7985487.png "屏幕截图.png")

打开之后，在工程文件上右键-add model，添加刚才生成的om文件。

2.编译

在**Mindstudio**的工具栏中点击**Build > Edit Build Configuration**。选择Target OS 为Centos7.6，Target Architecture选择x86_64.
![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/173856_9df2ef4d_7985487.png "屏幕截图.png")
   

之后点击**Build > Build > Build Configuration**，会开始编译。
![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/173945_8320017e_7985487.png "屏幕截图.png")
3.运行

在Mind Studio工具的工具栏中找到Run按钮，单击 Run > Edit Configurations。
在Command Arguments 中添加运行参数 ../data/detection.mp4.
由于本用例是在x86_64环境下运行，所以target host ip设置的是一个x86_64环境的云端服务ip.
![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/174003_b4d47e75_7985487.png "屏幕截图.png")

添加ip成功后，开始运行，结束时可以看到终端打印出信息，即表明样例运行成功。

![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/174025_6d3b76db_7985487.png "屏幕截图.png")