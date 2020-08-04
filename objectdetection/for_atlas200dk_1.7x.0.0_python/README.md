# 检测网络应用 （python）

本Application支持运行在Atlas 200 DK上，实现了yolov3网络的推理功能并在终端打印出检测结果。

## 软件准备

运行此Sample前，需要按照此章节获取源码包。

1.  <a name="zh-cn_topic_0228757084_section8534138124114"></a>获取源码包。  
    **mkdir -p $HOME/AscendProjects**
    
    **cd $HOME/AscendProjects**  

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/objectdetection_python.zip --no-check-certificate** 
              
    **unzip objectdetection_python.zip**  
    
    >![](public_sys-resources/icon-note.gif) **说明：**  
    >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/objectdetection_python.zip** 
    >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。


2.  <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。    

     -  下载原始网络模型及权重文件至ubuntu服务器任意目录，如:$HOME/yolov3_yuv。

        **mkdir -p $HOME/yolov3_yuv**

        **wget -P $HOME/yolov3_yuv https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.caffemodel** 

        **wget -P $HOME/yolov3_yuv https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.prototxt**  
        >![](public_sys-resources/icon-note.gif) **说明：**   
        >- yolov3原始模型网络： https://github.com/maxuehao/YOLOV3/blob/master/yolov3_res18.prototxt 
        >- yolov3原始网络LICENSE地址： https://github.com/maxuehao/caffe/blob/master/LICENSE
        >- C7x对prototxt文件有修改要求，按照[SSD网络模型prototxt修改](https://support.huaweicloud.com/usermanual-mindstudioc73/atlasmindstudio_02_0114.html)文档对prototxt文件进行修改。这里已经修改完成，直接执行以上命令下载即可。

3.  将原始网络模型转换为适配昇腾AI处理器的模型。  

    1.  设置环境变量
        
        命令行中输入以下命令设置环境变量。

        **cd \$HOME/yolov3_yuv**
        
        **export install_path=\$HOME/Ascend/ascend-toolkit/20.0.RC1/x86_64-linux_gcc7.3.0**  

        **export PATH=/usr/local/python3.7.5/bin:\\${install_path}/atc/ccec_compiler/bin:\\${install_path}/atc/bin:\\$PATH**  

        **export PYTHONPATH=\\${install_path}/atc/python/site-packages/te:\\${install_path}/atc/python/site-packages/topi:\\$PYTHONPATH**  

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64:\\$LD_LIBRARY_PATH**  

        **export ASCEND_OPP_PATH=\\${install_path}/opp**  

    2.  执行以下命令转换模型。

        **atc --model=yolov3.prototxt --weight=yolov3.caffemodel --framework=0 --output=yolov3_yuv --soc_version=Ascend310 --insert_op_conf=aipp_nv12.cfg**

    
4.  将转换好的模型文件（.om文件）上传到[步骤1](#zh-cn_topic_0219108795_li953280133816)中源码所在路径下的“**objectdetection_python/model**”目录下。
    
    **cp ./yolov3_yuv.om \$HOME/AscendProjects/objectdetection_python/model/**

## 环境部署<a name="zh-cn_topic_0228757083_section1759513564117"></a>

1.  应用代码拷贝到开发板。

    以Mind Studio安装用户进入分类网络应用\(python\)代码所在根目录，如：AscendProjects/objectdetection_python，执行以下命令将应用代码拷贝到开发板。

    **scp -r \\$HOME/AscendProjects/objectdetection_python HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI\_PROJECTS**

    提示password时输入开发板密码，开发板默认密码为**Mind@123**，如[图 应用代码拷贝](#zh-cn_topic_0228757083_zh-cn_topic_0198304761_fig1660453512014)。

    **图** **应用代码拷贝**<a name="zh-cn_topic_0228757083_zh-cn_topic_0198304761_fig1660453512014"></a>  
    

    ![](figures/zh-cn_image_0228832431.png)


2. acl.so拷贝到开发板。

   **scp ${HOME}/Ascend/ascend-toolkit/X.X.X/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl/acl.so HwHiAiUser@192.168.1.2:/home/HwHiAiUser/Ascend/**  
   >![](public_sys-resources/icon-note.gif) **说明：**   
            **请将X.X.X替换为Ascend-Toolkit开发套件包的实际版本号。**   
            **例如：Toolkit包的包名为Ascend-Toolkit-20.0.RC1-x86_64-linux_gcc7.3.0.run，则此Toolkit的版本号为20.0.RC1。**

3. 登录开发板，添加环境变量。  

   **ssh HwHiAiUser@192.168.1.2**  
   **vim \${HOME}/.bashrc**   
   在最后添加两行    
   **export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64**   
   **export PYTHONPATH=/home/HwHiAiUser/Ascend/:\\${PYTHONPATH}**  
   ![](figures/pythonpath.png)   
   执行如下命令，使环境变量生效   
   **source \${HOME}/.bashrc**

4. 安装环境依赖。

   请参考 https://gitee.com/ascend/common/blob/master/install_python3env/for_atlas200dk/README.md 进行环境依赖配置。  
    

## 运行

1. 登录到开发板上，进入工程目录下，执行如下命令运行程序。  

   **cd \${HOME}/HIAI_PROJECTS/objectdetection_python/**   
   **python3.7.5 object_detect.py ./data/**

2. 推理结果在终端中显示。

   ![image-20200725185820768](figures/obj_res.png)