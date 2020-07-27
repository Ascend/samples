中文|[English](Readme_EN.md)

#  检测网络应用（C++）<a name="ZH-CN_TOPIC_0219122211"></a>

本应用支持运行在AI云上加速环境(Atlas300)，实现了对yolov3目标检测网络的推理功能。 


## 软件准备<a name="zh-cn_topic_0219108795_section181111827718"></a>

运行此Sample前，需要按照此章节获取源码包。

1.  <a name="zh-cn_topic_0228757084_section8534138124114"></a>获取源码包。

    **cd $HOME/AscendProjects**   
 
    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/ubuntu/sample-objectdetection.zip**  
 
    **unzip sample-objectdetection.zip**
    >![](public_sys-resources/icon-note.gif) **说明：**   
    >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/ubuntu/sample-objectdetection.zip** 
    >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。
    
2.  <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。    
    1.  切换目录。  
        **cd $HOME/AscendProjects/sample-objectdetection/caffe_model**     
    2.  下载原始网络模型及权重文件。  
        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.caffemodel**  
        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.prototxt**

3.  将原始网络模型转换为适配昇腾AI处理器的模型。  

    执行模型转换的命令。         
    **atc --model=yolov3.prototxt --weight=yolov3.caffemodel --framework=0 --output=yolov3 --soc_version=Ascend310 --insert_op_conf=aipp_bgr.cfg** 
    
3.  将转换好的模型文件（.om文件）上传到[步骤1](#zh-cn_topic_0219108795_li953280133816)中源码所在路径下的“**sample-objectdetection/model**”目录下。
    
     **cp yolov3.om $HOME/AscendProjects/sample-objectdetection/model/**  



## 环境配置   

**注：服务器上已安装opencv库和ffmpeg库可跳过此步骤。**  
      
- 安装opencv和ffmpeg  
    - centos系统：  
    请参考 **https://gitee.com/ascend/common/blob/master/centos_install_opencv/CENTOS_INSTALL_OPENCV.md**  
    - ubuntu系统：  
    请参考 **https://gitee.com/ascend/common/blob/master/ubuntu_install_opencv/UBUNTU_INSTALL_OPENCV.md**

## 编译<a name="zh-cn_topic_0219108795_section3723145213347"></a>

1.  以HwHiAiUser（运行用户）登录开发环境。

2.  设置环境变量。 
   
    执行如下命令。 

     **vim ~/.bashrc** 

    在最后一行添加DDK_PATH及NPU_HOST_LIB的环境变量。

     **export DDK\_PATH=/home/HwHiAiUser/Ascend** 

     **export NPU\_HOST\_LIB=/home/HwHiAiUser/Ascend/acllib/lib64/stub**  

     **export LD\_LIBRARY_PATH=\\$DDK\_PATH/acllib/lib64:/usr/local/Ascend/add-ons:\\$HOME/ascend_ddk/host/lib:\\$DDK_PATH/atc/lib64** 

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

    **cd out**  

    进入out目录，查看推理结果的图片。


![结果1](figures/result.png)

