中文|[English](README_EN.md)

**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**

# 检测网络应用（C++）<a name="ZH-CN_TOPIC_0232337690"></a>  


本Application支持运行在Atlas 200 DK 或是 在AI云上加速环境(Atlas300)上，实现了对vgg_ssd目标检测网络的推理功能。 

## 软件准备<a name="zh-cn_topic_0219108795_section181111827718"></a>

运行此Sample前，需要按照此章节获取源码包。

1.  <a name="zh-cn_topic_0228757084_section8534138124114"></a>获取源码包。

    **cd $HOME/AscendProjects**  

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/classification.zip** 
              
    **unzip classification.zip**  
    
    >![](public_sys-resources/icon-note.gif) **说明：**   
    >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/classification.zip** 
    >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。
    
2.  <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的原始网络模型。    
 
     -  下载原始网络模型及权重文件至ubuntu服务器任意目录，如:$HOME/vgg_ssd。

        **mkdir -p $HOME/vgg_ssd**

        **wget https://obs-book.obs.cn-east-2.myhuaweicloud.com/shaxiang/C73/vgg_ssd.caffemodel --no-check-certificate** 
 
        **wget https://obs-book.obs.cn-east-2.myhuaweicloud.com/shaxiang/C73/vgg_ssd.prototxt --no-check-certificate**

       
            
        >![](public_sys-resources/icon-note.gif) **说明：**   
        >- vgg_ssd原始模型网络： https://github.com/weiliu89/caffe/tree/ssd
        >- vgg_ssd原始网络LICENSE地址： https://github.com/weiliu89/caffe/blob/ssd/LICENSE
        



## 环境配置   

**注：服务器上已安装OpenCV、PresenterAgent、交叉编译工具可跳过此步骤。**   
      
- 安装编译工具  
  **sudo apt-get install -y g++\-aarch64-linux-gnu g++\-5-aarch64-linux-gnu** 

- 安装OpenCV 
      
    请参考 **https://gitee.com/ascend/samples/tree/master/common/install_opencv/for_atlas200dk**   
  

## 编译<a name="zh-cn_topic_0219108795_section3723145213347"></a>

1.  打开对应的工程。

    以Mind Studio安装用户在命令行进入安装包解压后的“MindStudio-ubuntu/bin”目录，如：$HOME/MindStudio-ubuntu/bin。执行如下命令启动Mind Studio。

    **./MindStudio.sh**

    启动成功后，打开**sample-objectdetection_cvwithaipp**工程，如[图 打开objectdetection_cvwithaipp工程](#zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig11106241192810)所示。

    **图 **  打开objectdetection_cvwithaipp工程<a name="zh-cn_topic_0228461902_zh-cn_topic_0203223265_fig11106241192810"></a>  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/144507_906ee56f_5395865.png "屏幕截图.png")   

2.  将原始网络模型转换为适配昇腾AI处理器的模型。  

    1.  在Mind Studio操作界面的顶部菜单栏中选择**Tools \> Model Converter**，进入模型转换界面。
    2.  在弹出的**Model Conversion**操作界面中，进行模型转换配置。
    3.  参照以下图片进行参数配置。    
        -   Model File选择[步骤2](#zh-cn_topic_0219108795_li2074865610364)中下载的模型文件，此时会自动匹配到权重文件并填写在Weight File中。
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/144557_d189697b_5395865.png "屏幕截图.png")
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/144610_dc2f2b12_5395865.png "屏幕截图.png")
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/144621_d560fbfe_5395865.png "屏幕截图.png")   
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/144850_4c2a7af6_5395865.png "屏幕截图.png")

    
3.  将转换好的模型文件（.om文件）上传到[步骤1](#zh-cn_topic_0228757084_section8534138124114)中源码所在路径下的“**sample-objectdetection_cvwithaipp/model**”目录下。
    
     **cp \\$HOME/modelzoo/vgg_ssd/device/vgg_ssd.om \\$HOME/AscendProjects/objectdetection_cvwithaipp/model/**  
  

4.  开始编译，打开Mind Studio工具，在工具栏中点击**Build \> Edit Build Configuration**。  
    选择Target OS 为Centos7.6，如[图 配置编译](#zh-cn_topic_0203223265_fig17414647130)所示。

    **图 **  配置编译<a name="zh-cn_topic_0203223265_fig17414647130"></a>  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0919/105928_f1a02038_5408865.png "屏幕截图.png")
    
    之后点击**Build \> Build \> Build Configuration**，如[图 编译操作及生成文件](#zh-cn_topic_0203223265_fig1741464713019)所示，会在目录下生成build和out文件夹。

    **图 **  编译操作及生成文件<a name="zh-cn_topic_0203223265_fig1741464713019"></a>  
   ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/145013_877d1391_5395865.png "屏幕截图.png")

    >![](public_sys-resources/icon-notice.gif) **须知：**   
    >首次编译工程时，**Build \> Build**为灰色不可点击状态。需要点击**Build \> Edit Build Configuration**，配置编译参数后再进行编译。  
## 运行<a name="zh-cn_topic_0219108795_section1620073406"></a>
1.  在Mind Studio工具的工具栏中找到Run按钮，单击  **Run \> Edit Configurations**。  
    之后分别点击Apply、OK。如[图 配置运行](#zh-cn_topic_0203223265_fig93931954162720)所示。   

    **图 **  配置运行<a name="zh-cn_topic_0203223265_fig93931954162720"></a>   
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/145041_f24237ff_5395865.png "屏幕截图.png")
 
2.  单击  **Run \> Run 'sample-objectdetection_cvwithaipp'**，如[图 程序已执行示意图](#zh-cn_topic_0203223265_fig93931954162719)所示，可执行程序已经在开发者板执行。  

    **图 5**  程序已执行示意图<a name="zh-cn_topic_0203223265_fig93931954162719"></a>  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0929/145103_83731901_5395865.png "屏幕截图.png")

3.  查看运行结果。

    推理结果图片保存在工程下的“output \> outputs”目录下以时间戳命名的文件夹内。
 
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0919/110810_31c59ca0_5408865.png "屏幕截图.png")
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/0919/110827_61600ed7_5408865.png "屏幕截图.png")

