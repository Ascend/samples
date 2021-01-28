**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**
# MindSpore+Ascend310 从MindSpore到线下部署，开发垃圾分类AI应用（图片输入+图片输出）

## 案例内容
此案例将带领开发者体验端云协同开发，首先使用MindSpore训练垃圾分类模型，然后，使用Atlas200 DK/Atlas300(ai1s)部署模型并进行垃圾分类，端到端掌握AI业务全流程开发实践技能。开发技能的流程如图所示：



## 案例目标
- 掌握使用ModelArts训练垃圾分类AI模型。
- 掌握使用Atlas200 DK/Atlas300(ai1s)部署模型并跑通垃圾分类样例。

## 物料准备
- Liunx环境（虚拟机或Liunx系统）。
- [Atlas200 DK开发套件](https://www.vmall.com/product/10086085080100.html?78119)/[Atlas300(ai1s)](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)

## 环境准备
体验垃圾分类AI应用的开发，需要完成以下准备工作。
1. **ModelArts训练准备工作**

    参考[ModelArts准备工作wiki](https://gitee.com/ascend/samples/wikis/ModelArts%E5%87%86%E5%A4%87%E5%B7%A5%E4%BD%9C?sort_id=3466403)，完成ModelArts准备工作。包括注册华为云账号、ModelArts全局配置和OBS相关操作。

2. <a name="1"></a>**Atlas推理准备工作**

    - Atlas200 DK    
      （1）参考[制卡文档](https://support.huaweicloud.com/dedg-A200dk_3000_c75/atlased_04_0013.html)进行SD卡制作，制卡成功后等待开发者板四个灯常亮即可。
    
      （2）参考[连接文档](https://support.huaweicloud.com/dedg-A200dk_3000_c75/atlased_04_0015.html)中的**使用网线通过路由器连接Ubuntu服务器**步骤，完成开发者板和本地机器的连接及开发者板上网配置。

      （3）配置完成后，参考[环境准备和依赖安装](https://gitee.com/ascend/samples/blob/master/python/environment)准备好环境。

    - Atlas300（ai1s）
      （1）参考[购买并登录Linux弹性云服务器指南](https://support.huaweicloud.com/qs-ecs/zh-cn_topic_0132727313.html)购买AI加速型（ai1s）ECS弹性云服务器，并参考[卸载驱动固件和CANN软件文档](https://support.huaweicloud.com/instg-cli-cann/atlascli_03_0100.html)卸载预安装的老版本。

      （2）参考[CANN安装指南](https://support.huaweicloud.com/instg-cli-cann/atlascli_03_0017.html）配置ai1s的推理开发和运行环境。
 
      （3）配置完成后，参考[环境准备和依赖安装](https://gitee.com/ascend/samples/blob/master/python/environment)准备好环境。

## 数据集准备
按照如下步骤在modelarts上部署数据集。
1. **下载数据集**
    
    本案例使用垃圾分类数据集，点击[此链接](https://modelarts-labs.obs.cn-north-1.myhuaweicloud.com/codelab/mask_detection/mask_detection_500.tar.gz)，下载压缩包至本地，然后解压。

    解压后，可以看到mask_detection_500文件夹下有train和test两个文件夹。train文件夹中存放的训练集，共500张图片，均已标注。test文件夹下存放的是测试图片。

    **注意：该垃圾分类数据集只能用于学习用途，不得用于商业用途。**

2. **上传数据至OBS**

    windows环境中在OBS Browser+中，进入刚刚创建的“华为北京四”区域的OBS桶，然后点击上传按钮，上传本地文件夹mask_detection_500至OBS桶。

    ![](https://images.gitee.com/uploads/images/2021/0127/151112_1ab34d4a_5400693.png "uploadfolder.png")

    如果没有下载OBS Browser+，可以直接在网页上的OBS桶中直接上传文件（每次最多上传100个文件）。
    
    ![](https://images.gitee.com/uploads/images/2021/0127/151157_837b2d9f_5400693.png "chromuploader.png")

3. **创建数据集**

    点击[此链接](https://console.huaweicloud.com/modelarts/?region=cn-north-4#/manage/dataLabel_Beta)，进入ModelArts数据集。请确保区域在“华北-北京四”，本案例所有操作在“华北-北京四”。

    点击页面上的“创建数据集”按钮， 创建数据集页面填写示例如下：
    
    ![](https://images.gitee.com/uploads/images/2021/0127/151226_5a1c8ce7_5400693.png "createdatasets1.png")
    ![](https://images.gitee.com/uploads/images/2021/0127/151239_e04266f2_5400693.png "createdatasets2.png")

    - 数据集名称：自定义
    - 数据集输入位置：train文件夹所在的OBS路径
    - 数据集输出位置：标注数据的输出OBS路径。需要在OBS中创建这个路径，可以是使用OBS Browser+创建。
    - 标注场景：图片
    - 标注类型：物体检测
    
    填写完毕上述字段后，点击创建按钮。

    训练集中已经包含了标注文件，ModelArts数据集会自动加载标注文件。

    创建成功后，点击“发布”按钮，发布数据集

    注意：训练验证集比例中的训练集比例需要填写比例值（如：0.8）。

    ![](https://images.gitee.com/uploads/images/2021/0127/151258_350307a3_5400693.png "release.png")    

    ![](https://images.gitee.com/uploads/images/2021/0127/151314_23b5d95b_5400693.png "proportion.png")

4. **数据标注格式解读**

    **这里是对图片标注的介绍，当前的训练集中已经包含了标注文件，无需标注。**

    数据集发布成功后，点击进入数据集，然后点击“开始标注”按钮，观察数据标注详情。其中一张样例图片的标注详情如下：

    ![](https://images.gitee.com/uploads/images/2021/0127/151335_b8ce4b42_5400693.png "calloutformat.png")

    数据集共有三种类型的标注框，person（包含头部和肩部）、face和mask。判断一个人有没有戴口罩的方法是，脸部的检测框里面是否有口罩的检测框。person物体的作用是对人做目标跟踪。

## 模型训练
我们在ModelArts中训练模型，模型训练完成后转换成Atlas200 DK中可用的om模型。

1. **创建训练作业**

    接下来将通过ModelArts训练作业训练AI模型，使用ModelArts的yolov3预置算法训练一个垃圾分类模型。

    进入[ModelArts管理控制台](https://console.huaweicloud.com/modelarts/?region=cn-north-4#/manage/trainingjobs)，进入ModelArts“训练作业”页面。

    单击“ **创建** ”按钮，进入“ **创建训练作业** ”页面。

     在“创建训练作业”页面，按照如下指导填写训练作业相关参数。

    “计费模式”和“版本”为系统自动生成，不需修改。

    - 名称：自定义。
    - 描述：描述信息，可选。

    ![](https://images.gitee.com/uploads/images/2021/0127/151412_7e57ce05_5400693.png "create_train1.png")

    算法来源：单击“ **选择** ”，从“ **预置算法** ”列表中，选择“ **yolov3_resnet18** ”算法。

    - 数据来源：数据集
    - 选择数据集和选择版本：选择刚刚创建的口罩数据集和版本。
    - 训练输出位置：选择一个空的OBS路径，用来存储训练输出的模型。如/modelarts-course/mask_detection_500/output/，该路径需要自己创建。
    - 运行参数：列表中会自动增加train_url和data_url两个参数。需要添加一个运行参数max_epochs=400，max_epochs值越大训练时间越长。
    - 作业日志路径：选择一个空的OBS路径，用来存储作业训练日志。如/modelarts-course/mask_detection_500/log/，该路径需要自己创建。

    ![](https://images.gitee.com/uploads/images/2021/0127/151424_5879396e_5400693.png "create_train2.png")

    - 资源池：公共资源池
    - 类型：GPU
    - 规格：CPU：8 核 64GiB GPU：1 * nvidia-p100 16GiB。也可以选择V100，V100比P100的算力更强，但是更贵。
    - 计算节点：1

    ![](https://images.gitee.com/uploads/images/2021/0127/151605_3486a5ad_5400693.png "create_train4.png")

    完成信息填写，单击“下一步”。

    在“规格确认”页面，确认填写信息无误后，单击“ **立即创建** ”。

    在“训练作业”管理页面，可以查看新建训练作业的状态。

    如果设置 **max_epochs=400** ，训练过程需要4小时30分钟左右。当状态变更为“运行成功”时，表示训练作业运行完成。 您可以单击训练作业的名称，可进入此作业详情页面，了解训练作业的“配置信息”、“日志”、“资源占用情况”和“评估结果”等信息。
    
2. **模型转换**

    进入ModelArts管理控制台，在左侧导航栏中选择 **“模型管理”> “压缩/转换”** ，进入模型转换列表页面。

    单击左上角的 **“创建任务”** ，进入任务创建任务页面。

    在“创建任务”页面，填写相关信息。

    - 名称：输入“ **convert-mask-detection** ”。
    - 描述：口罩识别。
    - 输入框架：选择 **“TensorFlow”**。
    - 转换输入目录：训练作业的训练输出目录下的frozen_graph OBS目录，本案例中是/modelarts-course/mask_detection_500/output/frozen_graph/。
    - 输出框架：选择 **“MindSpore”**。
    - 转换输出目录：训练作业的训练输出目录下的om/model OBS目录，本案例中是/modelarts-course/mask_detection_500/output/om/model/。
    - 转换模板：选择 **“TensorFlow frozen graph 转 Ascend”** 。就是将TensorFlow的frozen graph格式的模型转换成可在昇腾芯片上推理的格式。
    - 输入张量形状：填写为 **“images:1,352,640,3”**。   

    任务信息填写完成后，单击右下角 **“立即创建”** 按钮。等待模型转换任务完成。

    模型转换完成后，在OBS的对应目录中下载模型，在本案例中模型路径如下：

    **/modelarts-course/mask_detection_500/output/om/model/convert-mask-detection.om**

## 模型推理
**详细案例部署步骤可以参考[YOLOV3_mask_detection_picture样例](../../2_object_detection/YOLOV3_mask_detection_picture)中Readme进行相关部署和运行，其中模型替换为自己训练出来的模型即可。**

### 案例部署

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

        开发环境，非root用户命令行中执行以下命令下载源码仓。

       **cd $HOME**

       **git clone https://gitee.com/ascend/samples.git**

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

        2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

        3. 开发环境中，执行以下命令，解压zip包。

            **cd $HOME**

            **unzip ascend-samples-master.zip**         

2. 模型移动到样例的model文件夹中。

   1. 将转换好的模型放置到Liunx下普通用户的$HOME目录中。

   2. 执行以下命令将模型复制到样例中model文件夹中。

        **cp ./mask_detection.om $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection/model/**

   3. 获取样例需要的测试图片。

        执行以下命令，进入样例的data文件夹中，下载对应的测试图片（或者自己找一些测试图片，放置到data目录中即可）。

        **cd $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection/data**

        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_mask_detection_picture-python/mask.jpg**

### 案例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **YOLOV3_mask_detection** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection/src**     
      **python3.6 mask_detect.py**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/YOLOV3_mask_detection/src**

      **python3.6 mask_detect.py** 


### 查看结果

运行完成后，会在out目录下生成带推理结果的jpg图片。
​       