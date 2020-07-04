中文|[English](Readme_EN.md)

#  检测网络应用视频版（C++）<a name="ZH-CN_TOPIC_0219122211"></a>

本Application支持运行在Atlas 200 DK，实现了对yolov3目标检测网络的推理功能。 


## 软件准备<a name="zh-cn_topic_0219108795_section181111827718"></a>

运行此Sample前，需要按照此章节获取源码包。

1.  <a name="zh-cn_topic_0228757084_section8534138124114"></a>获取源码包。

    **cd $HOME/AscendProjects**    

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/200dk/sample-objectdetection_video.zip** 
  
    **unzip sample-objectdetection_video.zip**    
    
    >![](public_sys-resources/icon-note.gif) **说明：**   
    >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/200dk/sample-objectdetection_video.zip** 
    >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。
    
2.  <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。    
    1.  切换目录。  
        **cd $HOME/AscendProjects/sample-objectdetection_video/caffe_model**     
    2.  下载原始网络模型及权重文件。  
        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.caffemodel**  
        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/yolov3/yolov3.prototxt**

3.  将原始网络模型转换为适配昇腾AI处理器的模型。  

    执行模型转换的命令。         
    **atc --model=yolov3.prototxt --weight=yolov3.caffemodel --framework=0 --output=yolov3_BGR --soc_version=Ascend310 --insert_op_conf=aipp_bgr.cfg**
    
4.  将转换好的模型文件（.om文件）上传到[步骤1](#zh-cn_topic_0219108795_li953280133816)中源码所在路径下的“**sample-objectdetection_video/model**”目录下。
    
     **cp yolov3_BGR.om $HOME/AscendProjects/sample-objectdetection_video/model/**  

## 环境配置   

**注：服务器上已安装OpenCV和PresentAgent可跳过此步骤。**  
    
- 安装OpenCV和PresentAgent  
      
    请参考 **https://gitee.com/ascend/common/blob/master/200dk_install_opencv/200DK_INSTALL_OPENCV_PRESENTAGENT.md**  


## 编译<a name="zh-cn_topic_0219108795_section3723145213347"></a>

1.  打开对应的工程。

    以Mind Studio安装用户在命令行进入安装包解压后的“MindStudio-ubuntu/bin”目录，如：$HOME/MindStudio-ubuntu/bin。执行如下命令启动Mind Studio。

    **./MindStudio.sh**

    启动成功后，打开**sample-objectdetection_video**工程，如[图 打开objectdetection_video工程](#zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig11106241192810)所示。

    **图 1**  打开objectdetection_video工程<a name="zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig11106241192810"></a>  
    ![](figures/打开objectdetection_video工程.png "打开objectdetection_video工程")

2.  修改Presenter Server的ip。  
    -  将**Script/presentserver/display/config/config.conf**中的**presenter_server_ip**修改为Mind Studio所在Ubuntu服务器的虚拟网卡的ip地址，如[图 presenter_server_ip](#zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig1110624110)所示。

      **图 2**  修改presenter_server_ip<a name="zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig1110624110"></a>  
      ![](figures/presenter_server_ip.png "修改presenter_server_ip")      
    -  将**src/sample_process.cpp**中的 **sample_process** 修改为Mind Studio所在Ubuntu服务器的虚拟网卡的ip地址，如[图 param_host_ip](#zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig11)所示。

      **图 3**  修改param_host_ip<a name="zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig11"></a>  
      ![](figures/param_host_ip.png "修改param_host_ip")    

    >![](public_sys-resources/icon-note.gif) **说明：**    
    >-  虚拟网卡的ip地址请通过ifconfig命令查看。    
3.  开始编译，打开Mind Studio工具，在工具栏中点击**Build \> Edit Build Configuration**。  
    选择**Target OS** 为**Euleros2.8**，如[图 配置编译](#zh-cn_topic_0203223265_fig17414647130)所示。

    **图 4**  配置编译<a name="zh-cn_topic_0203223265_fig17414647130"></a>  
    ![](figures/配置build.png "配置编译")  
    
    之后点击**Build \> Build \> Build Configuration**，如[图 编译操作及生成文件](#zh-cn_topic_0203223265_fig1741464713019)所示，会在目录下生成build和out文件夹。

    **图 5**  编译操作及生成文件<a name="zh-cn_topic_0203223265_fig1741464713019"></a>  
    ![](figures/编译操作及生成文件.png "编译操作及生成文件")

    >![](public_sys-resources/icon-notice.gif) **须知：**   
    >首次编译工程时，**Build \> Build**为灰色不可点击状态。需要点击**Build \> Edit Build Configuration**，配置编译参数后再进行编译。  

4.  启动Presenter Server。

    打开Mind Studio工具的Terminal，在应用代码存放路径下，执行如下命令在后台启动objectdetection_video应用的Presenter Server主程序。如[图 启动PresenterServer](#zh-cn_topic_0228461904_zh-cn_topic_0203223294_fig423515251067)所示。

    **python3 script/presenterserver/presenter_server.py --app=display &**

    **图 6**  启动PresenterServer<a name="zh-cn_topic_0228461904_zh-cn_topic_0203223294_fig423515251067"></a>  
    ![](figures/presentserver1.png)
   
  
    如[图 启动PresenterServer](#zh-cn_topic_0228461904_zh-cn_topic_0203223294_fig423)所示，表示presenter_server的服务启动成功。  
    **图 7**  启动PresenterServer<a name="zh-cn_topic_0228461904_zh-cn_topic_0203223294_fig423"></a>    
    ![](figures/presentserver2.png)

## 运行<a name="zh-cn_topic_0219108795_section1620073406"></a>
1.  在Mind Studio工具的工具栏中找到Run按钮，单击  **Run \> Edit Configurations**。  
    在Command Arguments 中添加运行参数 **../data**（输入图片的路径），之后分别点击Apply、OK。如[图 配置运行](#zh-cn_topic_0203223265_fig93931954162720)所示。   

    **图 8**  配置运行<a name="zh-cn_topic_0203223265_fig93931954162720"></a>   
    ![](figures/配置run.png "配置运行")
 
2.  单击  **Run \> Run 'sample-objectdetection_video'**，如[图 程序已执行示意图](#zh-cn_topic_0203223265_fig93931954162719)所示，可执行程序已经在开发者板执行。  

    **图 9**  程序已执行示意图<a name="zh-cn_topic_0203223265_fig93931954162719"></a>  
    ![](figures/程序已执行示意图.png "程序已执行示意图")  
3.  使用启动Presenter Server服务时提示的URL登录 Presenter Server 网站。

    等待Presenter Agent传输数据给服务端，单击“Refresh“刷新，当有数据时相应的Channel 的Status变成绿色，如下图所示。

    **图 10**  Presenter Server界面<a name="zh-cn_topic_0228461904_zh-cn_topic_0203223294_fig113691556202312"></a>  
    ![](figures/Presenter-Server界面.png "Presenter-Server界面") 

4.  单击右侧对应的View Name链接，比如上图的“video”，查看结果。





