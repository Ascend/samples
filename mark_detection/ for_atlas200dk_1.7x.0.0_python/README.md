# 口罩检测 （python）  

本Application支持运行在Atlas 200 DK上、实现了对图片中的口罩、人脸、人信息进行预测的功能。

## 软件准备

运行此Sample前，需要下载本仓中的源码包。

1. <a name="zh-cn_topic_0228757084_section8534138124114"></a>获取源码包。  
    **mkdir -p $HOME/AscendProjects**

    **cd $HOME/AscendProjects**  

    **wget https://c7xcode.obs.myhuaweicloud.com/code_Ascend/mask_detection_python.zip --no-check-certificate** 
              
    **unzip mask_detection_python.zip**  
    
    >![](public_sys-resources/icon-note.gif) **说明：**   
    >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.myhuaweicloud.com/code_Ascend/mask_detection_python.zip** 
    >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。


2.  <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。
参考[表1](#zh-cn_topic_0203223294_table144841813177)获取此应用中所用到的原始网络模型，并将其存放到Mind Studio所在Ubuntu服务器的任意目录。例如：$HOME/models/maskdetection_python。

    **表 1**  Mask Detection中使用模型

    <a name="zh-cn_topic_0203223294_table144841813177"></a>
    <table><thead align="left"><tr id="zh-cn_topic_0203223294_row161061318181712"><th class="cellrowborder" valign="top" width="13.61%" id="mcps1.2.4.1.1"><p id="zh-cn_topic_0203223294_p1410671814173"><a name="zh-cn_topic_0203223294_p1410671814173"></a><a name="zh-cn_topic_0203223294_p1410671814173"></a>模型名称</p>
    </th>
    <th class="cellrowborder" valign="top" width="10.03%" id="mcps1.2.4.1.2"><p id="zh-cn_topic_0203223294_p1106118121716"><a name="zh-cn_topic_0203223294_p1106118121716"></a><a name="zh-cn_topic_0203223294_p1106118121716"></a>模型说明</p>
    </th>
    <th class="cellrowborder" valign="top" width="76.36%" id="mcps1.2.4.1.3"><p id="zh-cn_topic_0203223294_p14106218121710"><a name="zh-cn_topic_0203223294_p14106218121710"></a><a name="zh-cn_topic_0203223294_p14106218121710"></a>模型下载路径</p>
    </th>
    </tr>
    </thead>
    <tbody><tr id="zh-cn_topic_0203223294_row1710661814171"><td class="cellrowborder" valign="top" width="13.61%" headers="mcps1.2.4.1.1 "><p id="zh-cn_topic_0203223294_p13106121801715"><a name="zh-cn_topic_0203223294_p13106121801715"></a><a name="zh-cn_topic_0203223294_p13106121801715"></a>mask_detection</p>
    </td>
    <td class="cellrowborder" valign="top" width="10.03%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0203223294_p13106171831710"><a name="zh-cn_topic_0203223294_p13106171831710"></a><a name="zh-cn_topic_0203223294_p13106171831710"></a>口罩检测网络模型。</p>
    <p id="zh-cn_topic_0203223294_p18106718131714"><a name="zh-cn_topic_0203223294_p18106718131714"></a><a name="zh-cn_topic_0203223294_p18106718131714"></a></p>
    </td>
    <td class="cellrowborder" valign="top" width="76.36%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0203223294_p110671813170"><a name="zh-cn_topic_0203223294_p110671813170"></a><a name="zh-cn_topic_0203223294_p110671813170"></a>请参考<a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/object_detect/mask_detection" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/object_detect/mask_detection</a>目录中README.md下载原始网络PB模型文件。</p>
    </td>
    </tr>
    </tbody>
    </table>

3. 将原始网络模型转换为适配昇腾AI处理器的模型。

    1.  在Mind Studio操作界面的顶部菜单栏中选择**Tools \> Model Converter**，进入模型转换界面。
    2.  在弹出的**Model Conversion**操作界面中，进行模型转换配置。
    3.  参照以下图片进行参数配置。    
        -   Model File选择[步骤2](#zh-cn_topic_0219108795_li2074865610364)中下载的模型文件。  
        -   Input Type 选择FP32。  
        -   关闭Data Preprocessing,本应用没有使用Aipp。
    ![](figures/模型转换1.png "模型转换1")  
    ![](figures/模型转换2.png "模型转换2")  
    ![](figures/模型转换3.png "模型转换3")

4. 将转换好的模型放到工程文件中的model 目录下。  
	**cp ~/modelzoo/mask_detection/device/mask_detection.om ~/AscendProjects/mask_detection_python/model/**   

## 环境部署<a name="zh-cn_topic_0228757083_section1759513564117"></a>

1.  应用代码拷贝到开发板。

    以Mind Studio安装用户进入口罩检测应用\(python\)代码所在根目录，如：AscendProjects/mask_detection_python，执行以下命令将应用代码拷贝到开发板。若拷贝失败，请检查开发板上是否有HIAI\_PROJECTS这个目录，没有就创建一下。
    
    **scp -r ~/AscendProjects/mask_detection_python HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI\_PROJECTS**
    
     提示password时输入开发板密码，开发板默认密码为**Mind@123**，如[图 应用代码拷贝](#zh-cn_topic_0228757083_zh-cn_topic_0198304761_fig1660453512014)。
    
     **图** **应用代码拷贝**<a name="zh-cn_topic_02287570831_zh-cn_topic_0198304761_fig1660453512014"></a>  
    

    ![](figures/cp-success.png)
    

    
2. acl.so拷贝到开发板。

    **scp ~/Ascend/ascend-toolkit/20.0.RC1/arm64-linux_gcc7.3.0/pyACL/python/site-packages/acl/acl.so HwHiAiUser@192.168.1.2:/home/HwHiAiUser/Ascend/**  
   >![](public_sys-resources/icon-note.gif) **说明：**   
            **请将X.X.X替换为Ascend-Toolkit开发套件包的实际版本号。**   
            **例如：Toolkit包的包名为Ascend-Toolkit-20.0.RC1-x86_64-linux_gcc7.3.0.run，则此Toolkit的版本号为20.0.RC1。**

3. 登录开发板，添加环境变量。  

   **ssh HwHiAiUser@192.168.1.2**  
   **vim ~/.bashrc**   
   在最后添加两行    
   **export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64**   
   **export PYTHONPATH=/home/HwHiAiUser/Ascend/:\\${PYTHONPATH}**  
   ![](figures/bashrc.png)   
   执行如下命令，使环境变量生效   
   **source ~/.bashrc**  

4. 安装环境依赖。 
   - 安装numpy、pillow   
       请参考 https://gitee.com/ascend/common/blob/master/install_python3env/for_atlas200dk/README.md 进行安装。   
   - 安装opencv  
       请参考 https://gitee.com/ascend/common/tree/master/install_opencv/for_atlas200dk 进行安装。
   
## 运行

1. 登录到开发板上，进入工程目录下，执行如下命令运行程序。  

   **cd ~/HIAI_PROJECTS/mask_detection_python/**   
   **python3 mask_detect.py**

2. 在终端可看到推理结果。

   ![](figures/result.png) 

3. 查看推理图片。  

   推理产生的结果图片保存在outputs文件夹，可传到Mindstudio安装用户的家目录中查看。  
   **scp -r username@host\_ip:/home/username/HIAI\_PROJECTS/mask_detection_python/out \~**

    -   username：开发板用户﻿名，默认为HwHiAiUser。
    -   host\_ip：开发板ip，USB连接一般为192.168.1.2.网线连接时一般为192.168.0.2。

    **命令示例：**  
    **scp -r HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI\_PROJECTS/mask_detection_python/out \~** 
   