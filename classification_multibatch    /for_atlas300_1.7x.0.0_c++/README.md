中文|[English](README_EN.md)

**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**

# classification_multibatch样例运行指导

本Sample实现了googlenet网络的推理功能。在推理前，需要编写一段代码逻辑：等输入数据满足多Batch（例如本案例：2Batch）的要求，申请Device上的内存存放多Batch的数据，作为模型推理的输入。如果最后循环遍历所有的输入数据后，仍不满足多Batch的要求，则直接将剩余数据作为模型推理的输入。运行成功后简单打印成功信息。

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


## 环境配置   

**注：服务器上已安装交叉编译工具可跳过此步骤。**  

- 安装编译工具  

  **sudo apt-get install -y g++\-aarch64-linux-gnu g++\-5-aarch64-linux-gnu** 

 

##  样例运行

1.打开打开classification_multibatch工程。
以Mind Studio安装用户在命令行进入安装包解压后的“MindStudio-ubuntu/bin”目录，如：$HOME/MindStudio-ubuntu/bin。执行如下命令启动Mind Studio。

./MindStudio.sh


![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/191352_8b407ee1_7985487.png "屏幕截图.png")

2.模型转换

在工具栏找到Tools,单击Model Converter.加载之前下载的模型文件和权重文件。

![输入图片说明](https://images.gitee.com/uploads/images/2020/1010/181751_937b37cb_7990837.png "屏幕截图.png")

本例中，batch值为2，设置N的值是2。

![输入图片说明](https://images.gitee.com/uploads/images/2020/1010/181835_19a37d16_7990837.png "屏幕截图.png")

继续设置

![输入图片说明](https://images.gitee.com/uploads/images/2020/1010/182024_47100f27_7990837.png "屏幕截图.png")

3.添加模型  
  
![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/194013_46aca0e8_7985487.png "屏幕截图.png")

4.编译

在**Mindstudio**的工具栏中点击**Build > Edit Build Configuration**。选择Target OS 为Centos7.6，Target Architecture选择x86_64.

![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/191419_f4672648_7985487.png "屏幕截图.png")
   

之后点击**Build > Build > Build Configuration**，会开始编译。
![输入图片说明](https://images.gitee.com/uploads/images/2020/0925/192627_146f85be_7985487.png "屏幕截图.png")
5.运行

在Mind Studio工具的工具栏中找到Run按钮，单击 Run > Edit Configurations。
在Command Arguments 中添加运行参数 ../data/detection.mp4.
由于本用例是在x86_64环境下运行，所以target host ip设置的是一个x86_64环境的云端服务ip.本例中使用如下ip.

![输入图片说明](https://images.gitee.com/uploads/images/2020/1010/163919_6d305d40_7990837.png "屏幕截图.png")
添加ip成功后，开始运行，结束时可以看到终端打印出信息，即表明样例运行成功。

![输入图片说明](https://images.gitee.com/uploads/images/2020/1010/180548_886fc416_7990837.png "屏幕截图.png")
