中文|[English](Readme_EN.md)

**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**

# 人脸识别<a name="ZH-CN_TOPIC_0208835545"></a>

开发者可以将本Application部署至Atlas 200 DK上实现人脸注册、并通过摄像头对视频中的人脸信息进行预测，与已注册的人脸进行比对，预测出最可能的用户。


## 前提条件<a name="zh-cn_topic_0203223340_section137245294533"></a>

部署此Sample前，需要准备好以下环境：

-   已完成Mind Studio的安装。
-   已完成Atlas 200 DK开发者板与Mind Studio的连接，交叉编译器的安装，SD卡的制作及基本信息的配置等。

## 软件准备<a name="zh-cn_topic_0203223340_section8534138124114"></a>

运行此Sample前，需要按照此章节获取源码包，并进行相关的环境配置。

1.  <a name="zh-cn_topic_0228757084_section8534138124114"></a>获取源码包。

    **cd $HOME/AscendProjects**  

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/facerecognition.zip** 
              
    **unzip facerecognition.zip**  
    
    >![](public_sys-resources/icon-note.gif) **说明：**   
    >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/code_Ascend/facerecognition.zip** 
    >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。

2.  <a name="zh-cn_topic_0203223340_li99811487013"></a>获取此应用中所需要的网络模型。

    参考[表1](#zh-cn_topic_0203223340_table97791025517)获取此应用中所用到的网络模型及其对应的权重文件，并将其存放到Mind Studio所在Ubuntu服务器的任意目录，这两个文件必须存放到同一个目录下。例如：$HOME/models/。

<a name="zh-cn_topic_0203223340_table97791025517"></a>
    <table><thead align="left"><tr id="zh-cn_topic_0203223340_row48791253115"><th class="cellrowborder" valign="top" width="13.309999999999999%" id="mcps1.2.4.1.1"><p id="zh-cn_topic_0203223340_p187902511114"><a name="zh-cn_topic_0203223340_p187902511114"></a><a name="zh-cn_topic_0203223340_p187902511114"></a>模型名称</p>
    </th>
    <th class="cellrowborder" valign="top" width="12.04%" id="mcps1.2.4.1.2"><p id="zh-cn_topic_0203223340_p148791259118"><a name="zh-cn_topic_0203223340_p148791259118"></a><a name="zh-cn_topic_0203223340_p148791259118"></a>模型说明</p>
    </th>
    <th class="cellrowborder" valign="top" width="74.65%" id="mcps1.2.4.1.3"><p id="zh-cn_topic_0203223340_p987922511111"><a name="zh-cn_topic_0203223340_p987922511111"></a><a name="zh-cn_topic_0203223340_p987922511111"></a>模型下载路径</p>
    </th>
    </tr>
    </thead>
    <tbody><tr id="zh-cn_topic_0203223340_row38791825912"><td class="cellrowborder" valign="top" width="13.309999999999999%" headers="mcps1.2.4.1.1 "><p id="zh-cn_topic_0203223340_p0879152519115"><a name="zh-cn_topic_0203223340_p0879152519115"></a><a name="zh-cn_topic_0203223340_p0879152519115"></a>face_detection</p>
    </td>
    <td class="cellrowborder" valign="top" width="12.04%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0203223340_p9879112516111"><a name="zh-cn_topic_0203223340_p9879112516111"></a><a name="zh-cn_topic_0203223340_p9879112516111"></a>人脸检测网络模型。</p>
    <p id="zh-cn_topic_0203223340_p1087912253112"><a name="zh-cn_topic_0203223340_p1087912253112"></a><a name="zh-cn_topic_0203223340_p1087912253112"></a>是基于Caffe的Resnet10-SSD300模型转换后的网络模型。</p>
    </td>
    <td class="cellrowborder" valign="top" width="74.65%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0203223340_p188801525813"><a name="zh-cn_topic_0203223340_p188801525813"></a><a name="zh-cn_topic_0203223340_p188801525813"></a>    
</p>
    </td>
    </tr>
    <tr id="zh-cn_topic_0203223340_row11880162511114"><td class="cellrowborder" valign="top" width="13.309999999999999%" headers="mcps1.2.4.1.1 "><p id="zh-cn_topic_0203223340_p1388012251117"><a name="zh-cn_topic_0203223340_p1388012251117"></a><a name="zh-cn_topic_0203223340_p1388012251117"></a>vanillacnn</p>
    </td>
    <td class="cellrowborder" valign="top" width="12.04%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0203223340_p1988018251110"><a name="zh-cn_topic_0203223340_p1988018251110"></a><a name="zh-cn_topic_0203223340_p1988018251110"></a>人脸特征点标记网络模型。</p>
    <p id="zh-cn_topic_0203223340_p588013251514"><a name="zh-cn_topic_0203223340_p588013251514"></a><a name="zh-cn_topic_0203223340_p588013251514"></a>是基于Caffe的VanillaCNN模型转换后的网络模型。</p>
    </td>
    <td class="cellrowborder" valign="top" width="74.65%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0203223340_p28801025319"><a name="zh-cn_topic_0203223340_p28801025319"></a><a name="zh-cn_topic_0203223340_p28801025319"></a>请参考<a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/vanillacnn" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/vanillacnn</a><span>目录中</span>README.md下载网络模型文件（vanillacnn.prototxt）及其对应的权重文件（vanillacnn.caffemodel）。</p>
    </td>
    </tr>
    <tr id="zh-cn_topic_0203223340_row988092511120"><td class="cellrowborder" valign="top" width="13.309999999999999%" headers="mcps1.2.4.1.1 "><p id="zh-cn_topic_0203223340_p108806251513"><a name="zh-cn_topic_0203223340_p108806251513"></a><a name="zh-cn_topic_0203223340_p108806251513"></a>sphereface</p>
    </td>
    <td class="cellrowborder" valign="top" width="12.04%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0203223340_p68802251019"><a name="zh-cn_topic_0203223340_p68802251019"></a><a name="zh-cn_topic_0203223340_p68802251019"></a>特征向量获取网络模型。</p>
    <p id="zh-cn_topic_0203223340_p148801125512"><a name="zh-cn_topic_0203223340_p148801125512"></a><a name="zh-cn_topic_0203223340_p148801125512"></a>是基于Caffe的SphereFace模型转换后的网络模型</p>
    </td>
    <td class="cellrowborder" valign="top" width="74.65%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0203223340_p128806251116"><a name="zh-cn_topic_0203223340_p128806251116"></a><a name="zh-cn_topic_0203223340_p128806251116"></a>请参考<a href="https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/sphereface" target="_blank" rel="noopener noreferrer">https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/sphereface</a><span>目录中</span>README.md下载网络模型文件（sphereface.prototxt）及其对应的权重文件（sphereface.caffemodel）。</p>
    </td>
    </tr>
    </tbody>
    </table>          


3. <a name="zh-cn_topic_0219108795_li2074865610364"></a>获取此应用中所需要的facedetection网络模型。 
 
     -  facedetection网络模型及权重文件按如下方式下载。

        **mkdir -p $HOME/models** 

        **wget -P $HOME/models https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection/face_detection.caffemodel** 
 
        **wget -P $HOME/models https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection/face_detection.prototxt**
    
        >![](public_sys-resources/icon-note.gif) **说明：**   
        >- facedetection原始模型网络： https://github.com/opencv/opencv/blob/master/samples/dnn/face_detector/deploy.prototxt。
        >- facedetection原始网络LICENSE地址： https://github.com/opencv/opencv/blob/master/LICENSE
        >- C7x对prototxt文件有修改要求，按照[SSD网络模型prototxt修改](https://support.huaweicloud.com/usermanual-mindstudioc73/atlasmindstudio_02_0114.html)文档对prototxt文件进行修改。这里已经修改完成，直接执行以上命令下载即可。

4.  将原始网络模型转换为适配昇腾AI处理器的模型。
    -   通过Mind Studio工具进行模型转换。
    1.  在Mind Studio操作界面的顶部菜单栏中选择**Ascend \> Model Convert**，进入模型转换界面。
    2.  在弹出的**Model Conversion**操作界面中，进行模型转换配置。
    3.  face_detection参照以下图片进行参数配置。    
        -   Model File选择[步骤2](#zh-cn_topic_0219108795_li2074865610364)中下载的模型文件，此时会自动匹配到权重文件并填写在Weight File中。  
        -   模型输入选择fp32。  
        -   Input Image Resolution填写为300\*304。   
        -   Model Image Format选择BGR。   
        -   打开Crop。

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/203114_239ed3c2_7985487.png "屏幕截图.png")  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/203126_c08119ac_7985487.png "屏幕截图.png")  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/203139_595983cf_7985487.png "屏幕截图.png")   
    4.  vanillacnn参照以下图片进行参数配置。    
![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/173302_cee66fa8_7985487.png "屏幕截图.png")   
![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/173408_48009a23_7985487.png "屏幕截图.png")   
![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/205902_5cbf2f3b_7985487.png "屏幕截图.png")     
    5.  sphereface参照以下图片进行参数配置。    
![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/205918_11d242bb_7985487.png "屏幕截图.png")      
![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/205931_f80c10a2_7985487.png "屏幕截图.png")   
![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/205944_0cd5a166_7985487.png "屏幕截图.png")       
 
5.将转换好的模型文件（.om文件）上传到步骤1中源码所在路径的“facerecognition/model”目录下。     
**cp \\$HOME/modelzoo/face_detection/device/face_detection.om \\$HOME/AscendProjects/facerecognition/model/**     
**cp \\$HOME/modelzoo/vanillacnn/device/vanillacnn.om \\$HOME/AscendProjects/facerecognition/model/**       
**cp \\$HOME/modelzoo/sphereface/device/sphereface.om \\$HOME/AscendProjects/facerecognition/model/**       

## 环境配置   

**注：服务器上已安装交叉编译工具和PresenterAgent可跳过此步骤。**   
      
- 安装编译工具  
  **sudo apt-get install -y g++\-aarch64-linux-gnu g++\-5-aarch64-linux-gnu** 

- 安装PresenterAgent    
  请参考 https://gitee.com/ascend/samples/tree/master/common/install_presenteragent/for_atlas200dk
  
## 编译<a name="zh-cn_topic_0219108795_section3723145213347"></a>

1.  打开对应的工程。

    以Mind Studio安装用户在命令行中进入安装包解压后的“MindStudio-ubuntu/bin”目录，如：$HOME/MindStudio-ubuntu/bin。执行如下命令启动Mind Studio

    **./MindStudio.sh**

    启动成功后，打开**facerecognition**工程，如[图7](#zh-cn_topic_0203223340_fig28591855104218)所示。

    **图 7**  打开 **facerecognition** 工程<a name="zh-cn_topic_0203223340_fig28591855104218"></a>  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/181636_2ef0a430_7985487.png "屏幕截图.png")     

2.  在data/param.conf 修改presenter_server_ip     
![输入图片说明](https://images.gitee.com/uploads/images/2020/1120/201644_9456e521_7985487.png "屏幕截图.png")
2.  在**src/param\_configure.conf**文件中配置相关工程信息。

    如[图8](#zh-cn_topic_0203223340_fig1338571124515)所示。

    **图 8**  配置文件路径<a name="zh-cn_topic_0203223340_fig1338571124515"></a>  
    

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/181456_f6ae1375_7985487.png "屏幕截图.png")

    该配置文件默认配置内容如下：

    ```
    remote_host=192.168.1.2
    data_source=Channel-1
    presenter_view_app_name=video
    ```

    -   remote\_host：配置为Atlas 200 DK开发者板的IP地址。
    -   data\_source: 配置为摄像头所属Channel，取值为Channel-1或者Channel-2，查询摄像头所属Channel的方法请参考[Atlas 200 DK用户手册](https://ascend.huawei.com/doc/Atlas200DK/)中的“如何查看摄像头所属Channel”。
    -   presenter\_view\_app\_name: 用户自定义的在PresenterServer界面展示的View Name，此View Name需要在Presenter Server展示界面唯一，只能为大小写字母、数字、“\_”的组合，位数3\~20。

    >![](public_sys-resources/icon-note.gif) **说明：**   
    >-   三个参数必须全部填写，否则无法通过build。  
    >-   注意参数填写时不需要使用“”符号。  
    >-   当前已经按照配置示例配置默认值，请按照配置情况自行修改。  



4.  开始编译，打开Mind Studio工具，在工具栏中点击**Build \> Build \> Build-Configuration**。如[图10](#zh-cn_topic_0203223340_fig1629455494718)所示，会在目录下生成build和run文件夹。

    **图 10**  编译操作及生成文件<a name="zh-cn_topic_0203223340_fig1629455494718"></a>     
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/182322_31e44ff5_7985487.png "屏幕截图.png")
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/182309_15b83b07_7985487.png "屏幕截图.png")

    >![](public_sys-resources/icon-notice.gif) **须知：**   
    >首次编译工程时，**Build \> Build**为灰色不可点击状态。需要点击**Build \> Edit Build Configuration**，配置编译参数后再进行编译。  

5.  <a name="zh-cn_topic_0203223340_li1364788188"></a>启动Presenter Server

    打开Mind Studio工具的Terminal，在应用代码存放路径下，执行如下命令在后台启动facerecognition应用的Presenter Server主程序。如[图11](#zh-cn_topic_0203223340_fig156364995016)所示。

    **bash run_present_server.sh**

    **图 11**  启动PresenterServer<a name="zh-cn_topic_0203223340_fig156364995016"></a>  
    

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/182759_a03af4da_7985487.png "屏幕截图.png")

    -   当提示“Please choose one to show the presenter in browser\(default: 127.0.0.1\):“时，请输入在浏览器中访问Presenter Server服务所使用的IP地址（一般为访问Mind Studio的IP地址）。
    -   当提示“Please input a absolute path to storage facerecognition data:“时，请输入Mind Studio中存储人脸注册数据及解析数据，此路径Mind Studio用户需要有读写权限，如果此路径不存在，脚本会自动创建。

    如[图12](#zh-cn_topic_0203223340_fig157571218181018)所示，请在“Current environment valid ip list“中选择通过浏览器访问Presenter Server服务使用的IP地址，并输入存储人脸识别解析数据的路径。

    **图 12**  工程部署示意图<a name="zh-cn_topic_0203223340_fig157571218181018"></a>  
    

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/182959_fb072204_7985487.png "屏幕截图.png")

    如[图13](#zh-cn_topic_0203223340_fig123741843161320)所示，表示presenter\_server的服务启动成功。

    **图 13**  Presenter Server进程启动<a name="zh-cn_topic_0203223340_fig123741843161320"></a>  
    

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/183033_164846ff_7985487.png "屏幕截图.png")

    使用上图提示的URL登录Presenter Server，IP地址为[图12](#zh-cn_topic_0203223340_fig157571218181018)中输入的IP地址，端口号默为7009，如下图所示，表示Presenter Server启动成功。

    **图 14**  主页显示<a name="zh-cn_topic_0203223340_fig98461795813"></a>  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/183112_49f848d1_7985487.png "屏幕截图.png")

    Presenter Server、Mind Studio与Atlas 200 DK之间通信使用的IP地址示例如下图所示：

    **图 15**  IP地址示例<a name="zh-cn_topic_0203223340_fig1627210116351"></a>  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/183122_56274f00_7985487.png "屏幕截图.png")

    其中：

    -   Atlas 200 DK开发者板使用的IP地址为192.168.1.2（USB方式连接）。
    -   Presenter Server与Atlas 200 DK通信的IP地址为UI Host服务器中与Atlas 200 DK在同一网段的IP地址，例如：192.168.1.223。
    -   通过浏览器访问Presenter Server的IP地址本示例为：10.10.0.1，由于Presenter Server与Mind Studio部署在同一服务器，此IP地址也为通过浏览器访问Mind Studio的IP。


## 运行<a name="zh-cn_topic_0203223340_section1676879104"></a>

1.  运行人脸识别应用程序。

    在Mind Studio工具的工具栏中找到Run按钮，点击**Run \> Run 'facerecognition'**，如[图16](#zh-cn_topic_0203223340_fig182957429910)所示，可执行程序已经在开发者板执行。

    **图 16**  程序已执行示意图<a name="zh-cn_topic_0203223340_fig182957429910"></a>  
    

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/183727_05d7d1b0_7985487.png "屏幕截图.png")

2.  使用启动Presenter Server服务时提示的URL登录 Presenter Server 网站，详细可参考[启动Presenter Server](#zh-cn_topic_0203223340_li1364788188)  。

    Presenter Server展示界面如[图17](#zh-cn_topic_0203223340_fig1189774382115)所示。

    **图 17**  Presenter Server界面<a name="zh-cn_topic_0203223340_fig1189774382115"></a>  
    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/183820_56192295_7985487.png "屏幕截图.png")

    >![](public_sys-resources/icon-note.gif) **说明：**   
    >-   facerecognition的Presenter Server最多支持2路Channel同时显示，每个  _presenter\_view\_app\_name_  对应一路Channel。  
    >-   由于硬件的限制，每一路支持的最大帧率是20fps，受限于网络带宽的影响，帧率会自动适配较低的帧率进行显示。  

3.  进行人脸注册。
    1.  点击“Face Library“页签，在界面中输入“Username“。

        **图 18**  人脸注册界面<a name="zh-cn_topic_0203223340_fig12445181112163"></a>  
        ![输入图片说明](https://images.gitee.com/uploads/images/2020/1118/183930_01fcddf5_7985487.png "屏幕截图.png")

    2.  单击“Browse“按钮，上传人脸图像，人脸图像裁剪时尽量按照“Example Photo“的比例设置。

    1.  点击Submit按钮上传若上传失败，可以更改裁剪比例。

4.  人脸识别以及比对。

    进入“App List“页签，在界面中点击对应的“App Name“，例如  _video_  ，若有人脸出现在摄像头中，且与已注册人脸匹配一致，则会出现对应人员姓名及相似度的标注。


## 后续处理<a name="zh-cn_topic_0203223340_section1092612277429"></a>

-   **停止人脸识别应用**

    Facial Recognition应用执行后会处于持续运行状态，若要停止Facial Recognition应用程序，可执行如下操作。

    单击[图19 停止Facial Recognition应用](#zh-cn_topic_0203223340_fig12461162791610)所示的停止按钮停止Facial Recognition应用程序。

    **图 19**  停止Facial Recognition应用<a name="zh-cn_topic_0203223340_fig12461162791610"></a>  
    

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1119/204132_0680b016_7985487.png "屏幕截图.png")

    如[图20](#zh-cn_topic_0203223340_fig5786125319165)所示应用程序已停止运行

    **图 20**  Facial Recognition应用已停止<a name="zh-cn_topic_0203223340_fig5786125319165"></a>  
    

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1119/204153_557cd8cf_7985487.png "屏幕截图.png")


-   **停止Presenter Server服务**

    Presenter Server服务启动后会一直处于运行状态，若想停止人脸识别应用对应的Presenter Server服务，可执行如下操作。

    以Mind Studio安装用户在Mind Studio所在服务器中执行如下命令查看人脸识别应用对应的Presenter Server服务的进程。

    **ps -ef | grep presenter | grep facerecognition**

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1119/205326_0c7998d7_7985487.png "屏幕截图.png")

    如上所示  _36806_  即为人脸识别应用对应的Presenter Server服务的进程ID。

    若想停止此服务，执行如下命令：

    **kill -9** _36806_


