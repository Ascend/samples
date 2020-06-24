中文|[English](Readme_EN.md)

#  检测网络应用（C++）<a name="ZH-CN_TOPIC_0219122211"></a>

本应用支持运行在AI云上加速环境(Atlas300)，实现了对yolov3目标检测网络的推理功能。 


## 软件准备<a name="zh-cn_topic_0219108795_section181111827718"></a>

运行此Sample前，需要按照此章节获取源码包。

1.  <a name="zh-cn_topic_0228757084_section8534138124114"></a>获取源码包。

    1. 下载压缩包方式获取。

        将 **https://gitee.com/chen68/colorization_c7x(要更换)** 仓中的代码下载至Ubuntu服务器的任意目录。例如代码存放路径为：$HOME/AscendProjects/sample-objectdetection。

    2. 命令行使用git命令方式获取。

        在命令行中：$HOME/AscendProjects目录下执行以下命令下载代码。

        **git clone https://gitee.com/chen68/colorization_c7x.git(要更换)**
    
2.  <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。

    参考[表 检测网络应用使用模型](#zh-cn_topic_0219108795_table19942111763710)获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到“$HOME/AscendProjects/sample-objectdetection/caffe_model”目录下。

    **表 1**  检测网络应用使用模型

  <a name="zh-cn_topic_0219108795_table19942111763710"></a>
<table><thead align="left"><tr id="zh-cn_topic_0219108795_row611318123710"><th class="cellrowborder" valign="top" width="11.959999999999999%" id="mcps1.2.4.1.1"><p id="zh-cn_topic_0219108795_p81141820376"><a name="zh-cn_topic_0219108795_p81141820376"></a><a name="zh-cn_topic_0219108795_p81141820376"></a>模型名称</p>
</th>
<th class="cellrowborder" valign="top" width="8.07%" id="mcps1.2.4.1.2"><p id="zh-cn_topic_0219108795_p13181823711"><a name="zh-cn_topic_0219108795_p13181823711"></a><a name="zh-cn_topic_0219108795_p13181823711"></a>模型说明</p>
</th>
<th class="cellrowborder" valign="top" width="79.97%" id="mcps1.2.4.1.3"><p id="zh-cn_topic_0219108795_p1717182378"><a name="zh-cn_topic_0219108795_p1717182378"></a><a name="zh-cn_topic_0219108795_p1717182378"></a>模型下载路径</p>
</th>
</tr>
</thead>
<tbody><tr id="zh-cn_topic_0219108795_row1119187377"><td class="cellrowborder" valign="top" width="11.959999999999999%" headers="mcps1.2.4.1.1 "><p id="zh-cn_topic_0219108795_p4745165253920"><a name="zh-cn_topic_0219108795_p4745165253920"></a><a name="zh-cn_topic_0219108795_p4745165253920"></a>yolov3</p>
</td>
<td class="cellrowborder" valign="top" width="8.07%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0219108795_p1874515218391"><a name="zh-cn_topic_0219108795_p1874515218391"></a><a name="zh-cn_topic_0219108795_p1874515218391"></a>图片分类推理模型。

是基于Caffe的yolov3模型。</p>
</td>
<td class="cellrowborder" valign="top" width="79.97%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0219108795_p611318163718"><a name="zh-cn_topic_0219108795_p611318163718"></a><a name="zh-cn_topic_0219108795_p611318163718"></a>请参考<a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/object_detect/yolov3" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/object_detect/yolov3</a>目录中README.md下载原始网络模型文件及其对应的权重文件。</p>
</td>
</tr>
</tbody>
</table>

3.  将原始网络模型转换为适配昇腾AI处理器的模型。
    1.  切换到模型所在目录。  
        
         **cd $HOME/AscendProjects/sample-objectdetection/caffe_model** 

    2.  执行模型转换的命令。  
        
         **atc --model=yolov3.prototxt --weight=yolov3.caffemodel --framework=0 --output=yolov3_BGR --soc_version=Ascend310 --insert_op_conf=aipp_bgr.cfg** 
    
3.  将转换好的模型文件（.om文件）上传到[步骤1](#zh-cn_topic_0219108795_li953280133816)中源码所在路径下的“**sample-objectdetection/model**”目录下。
    
     **cp yolov3_BGR.om $HOME/AscendProjects/sample-objectdetection/model/**  



## 环境配置   

**注：服务器上已安装opencv库和ffmpeg库可跳过此步骤。**  
    

- 安装opencv和ffmpeg      
  请参考 **https://gitee.com/atlasdevelop/c7x_samples/blob/master/Ai1_sample/sample-colorization/INSTALL_OPENCV.md(要更换)**

## 编译<a name="zh-cn_topic_0219108795_section3723145213347"></a>

1.  以HwHiAiUser（运行用户）登录开发环境。

2.  设置环境变量。 
   
    执行如下命令。 

     **vim ~/.bashrc** 

    在最后一行添加DDK_PATH及NPU_HOST_LIB的环境变量。

     **export DDK\_PATH=/home/HwHiAiUser/Ascend** 

     **export NPU\_HOST\_LIB=/home/HwHiAiUser/Ascend/acllib/lib64/stub**  

     **export LD\_LIBRARY_PATH=\\$DDK\_PATH/acllib/lib64:/usr/local/Ascend/add-ons:\\$HOME/ascend_lib/host/lib:\\$DDK_PATH/atc/lib64** 

     >![](public_sys-resources/icon-note.gif) **说明：**   
            **请将/home/HwHiAiUser/Ascend替换为ACLlib标准形态安装包的实际安装路径。** 
    

    输入:wq!保存退出。

    执行如下命令使环境变量生效。

     **source ~/.bashrc**   

3.  创建用于存放编译文件的目录。  


    **cd $HOME/AscendProjects/sample-objectdetection**  
    
    **mkdir -p build/intermediates/host**  

4.  执行cmake生成编译文件。

     **cd build/intermediates/host**   

    **cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**  

5.  执行make命令，生成的可执行文件main在“$HOME/AscendProjects/sample-objectdetection/out”目录下。 

    **make**
## 运行<a name="zh-cn_topic_0219108795_section1620073406"></a>


1.  将需要检测的图片上传至“$HOME/AscendProjects/sample-objectdetection/data/”目录下。  

2.  切换到可执行文件main所在的目录，给该目录下的main文件加执行权限。  
   
    **cd $HOME/AscendProjects/sample-objectdetection/out**  
    
    **chmod +x main**   

3.  运行可执行文件。  
    **./main \.\./data/**

4.  查看运行结果。 

    **cd result**  

    进入result目录，查看推理结果的图片。


![结果1](figures/result.png)

