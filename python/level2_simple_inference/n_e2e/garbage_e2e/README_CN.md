**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**
# MindSpore+Ascend310 从MindSpore到线下部署，开发垃圾分类AI应用（图片输入+图片输出）

## 案例内容
此案例将带领开发者体验端云协同开发，首先使用MindSpore训练垃圾分类模型，然后，使用Atlas200 DK/Atlas300(ai1s)部署模型并进行垃圾分类，端到端掌握AI业务全流程开发实践技能。开发技能的流程如图所示：

![](https://images.gitee.com/uploads/images/2021/0128/170026_bdfe13e3_5400693.png "garbage_flow.png")

## 案例目标
- 掌握使用MindSpore训练垃圾分类AI模型（基于modelarts）。
- 掌握使用Atlas200 DK/Atlas300(ai1s)部署模型并跑通垃圾分类样例。

## 物料准备
- Liunx环境（虚拟机或Liunx系统）。
- [Atlas200 DK开发套件](https://www.vmall.com/product/10086085080100.html?78119)/[Atlas300(ai1s)](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)

## 环境准备
体验垃圾分类AI应用的开发，需要完成以下准备工作。
1. **ModelArts训练准备工作**

    参考[ModelArts准备工作wiki](https://github.com/Ascend/samples/wikis/ModelArts%E5%87%86%E5%A4%87%E5%B7%A5%E4%BD%9C?sort_id=3466403)，完成ModelArts准备工作。包括注册华为云账号、ModelArts全局配置和OBS相关操作。

2. <a name="1"></a>**Atlas推理准备工作**

    - Atlas200 DK    
      （1）参考[制卡文档](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1013/environment/atlased_04_0009.html)进行SD卡制作，制卡成功后等待开发者板四个灯常亮即可。
    
      （2）参考[连接文档](https://www.hiascend.com/document/detail/zh/Atlas200DKDeveloperKit/1013/environment/atlased_04_0012.html)中的**使用网线通过路由器连接Ubuntu服务器**步骤，完成开发者板和本地机器的连接及开发者板上网配置。

      （3）配置完成后，参考[环境准备和依赖安装](https://github.com/Ascend/samples/blob/master/python/environment)准备好环境。

    - Atlas300（ai1s）    
      （1）参考[购买并登录Linux弹性云服务器指南](https://support.huaweicloud.com/qs-ecs/zh-cn_topic_0132727313.html)购买AI加速型（ai1s）ECS弹性云服务器，并参考[卸载驱动固件和CANN软件文档](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/softwareinstall/instg/atlasdeploy_03_0070.html)卸载预安装的老版本。

      （2）参考[CANN安装指南](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/softwareinstall/instg/atlasdeploy_03_0002.html)配置ai1s的推理开发和运行环境。
 
      （3）配置完成后，参考[环境准备和依赖安装](https://github.com/Ascend/samples/blob/master/python/environment)准备好环境。

## 模型训练
我们在ModelArts中训练模型，模型训练完成后转换成Atlas中可用的om模型。

1. 下载训练代码和数据集并上传至obs。

    - 训练代码及数据集文件：mobilenetv2_garbage[点击下载](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com:443/003_Atc_Models/AE/ATC%20Model/garbage/mobilenetv2_garbage.zip)
    
    - 下载后解压至本地并且上传到obs桶中。     
      ![](https://images.gitee.com/uploads/images/2021/0129/113113_d163802f_5400693.png "上传obs.png")
      

2. **创建训练作业**

    接下来将通过ModelArts的Notebook训练AI模型，使用ModelArts的MindSpore框架训练一个垃圾分类模型。

    进入[ModelArts管理控制台](https://console.huaweicloud.com/modelarts/?region=cn-north-4#/manage/trainingjobs)，进入ModelArts“开发环境/Notebook”页面，默认进入的是新版的界面，这里点击左上角，选择返回旧版（以下以旧版本界面为例）。

    单击“ **创建** ”按钮，进入“ **创建Notebook** ”页面。

     在“创建Notebook”页面，按照如下指导填写训练作业相关参数。

    “计费模式”为系统自动生成，不需修改。

    - 名称：自定义。
    - 描述：描述信息，可选。
    - 自动停止：选择“工作环境“后弹出，可自行选择时长。

    ![](https://images.gitee.com/uploads/images/2021/0129/114727_aeda310d_5400693.png "notebook1.png")

    - 工作环境：展开“公共镜像”，选择 **Ascend-Powered-Engine 1.0（Python3）**。    
    - 资源池：默认选择“公共资源池”即可。
    - 类型：默认选择“Ascend”即可。
    - 规格：默认选择“Ascend：1*Ascend 910 cpu：24核 96 GIB”即可。。
    - 存储配置：默认选择“对象存储服务（OBS）”即可。
    - 存储位置：选择一个上传代码的路径，如/train-moderlarts/mobilenetv2_garbage/。

    ![](https://images.gitee.com/uploads/images/2021/0129/115439_462389e9_5400693.png "notebook2.png")

    完成信息填写，单击“下一步”,规格确认无误后点击提交即可。

3. **配置训练作业**

    如果Notebook没有启动，则在Notebook页签中启动创建的任务。

    ![](https://images.gitee.com/uploads/images/2021/0129/141119_8c8f5300_5400693.png "启动.png")

    如果Notebook已经启动，则在Notebook页签中打开训练任务。

    ![](https://images.gitee.com/uploads/images/2021/0129/141412_80014968_5400693.png "open.png")

    打开后，进入到Jupyter页面，将所有文件同步到Modelarts中（当前展示的这些文件都是OBS上的数据，训练加载时需要在Modelarts的Notebook创建的环境中同步这些文件）。

    ![](https://images.gitee.com/uploads/images/2021/0129/141949_538edff6_5400693.png "sync.png")

    **注意：同步时会报错每次限制同步文件数为1024个，此时需要分批同步，保证将所有文件都同步到Modelarts中。**

    同步完成后，在juputer中直接打开main.ipynb文件。

    ![](https://images.gitee.com/uploads/images/2021/0129/142438_14700518_5400693.png "openmain.png")
    
    打开后会提示Kernel not found，此时选择为Mindspore-1.0.0-python3.7-aarch64后点击Set Kernel即可。

    ![](https://images.gitee.com/uploads/images/2021/0129/142755_70778456_5400693.png "setkernel.png")

4. **模型训练**

    源代码中设置了device_id为3，此处设置会有报错，所以需要删除准备工作中context.set_context函数的参数device_id=3。

    ![](https://images.gitee.com/uploads/images/2021/0129/143536_bccaa3d7_5400693.png "delparam.png")

    将超参中的"pretrained_ckpt"中的参数改成 "./mobilenetV2-200_1067.ckpt"
  
    ![](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/200dk/pictures/garbage_pr_use.png)

    依次单步执行即可，可以查看每一步的输出结果和描述来了解代码。

    ![](https://images.gitee.com/uploads/images/2021/0129/144050_5cd2e87d_5400693.png "viewres.png")

    执行完成后会生成后缀为air的模型，该模型为Mindspore框架的模型，所以需要增加两行，将生成的air木星复制到OBS中再下载到本地准备推理。

    **import moxing as mox**    
    **mox.file.copy('/home/ma-user/work/obs_file.txt', 'obs://bucket_name/obs_file.txt')**    
    本次训练修改如下。    
    ![](https://images.gitee.com/uploads/images/2021/0129/150116_ccb57e2f_5400693.png "copydata.png")

    执行成功后，可以在对应文件夹中检查模型是否保存到OBS中。

## 模型推理
**详细案例部署步骤可以参考[garbage_picture样例](../../../contrib/garbage_picture)中Readme进行相关部署和运行，其中模型替换为自己训练出来的模型即可。**

### 案例部署

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

        开发环境，非root用户命令行中执行以下命令下载源码仓。

       **cd $HOME**

       **git clone https://github.com/Ascend/samples.git**

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

        2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

        3. 开发环境中，执行以下命令，解压zip包。

            **cd $HOME**

            **unzip ascend-samples-master.zip**         

2. 从OBS下载air模型到开发环境普通用户的$HOME目录下的任意文件夹，然后进行模型转换。例如：$HOME/models/googlenet_imagenet_picture。

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export LD_LIBRARY_PATH=\\${install_path}/compiler/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/googlenet_imagenet_picture**  

        **wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/garbage_picture/insert_op_yuv.cfg**

        **atc --model=./mobilenetv2.air --framework=1 --output=garbage_yuv --soc_version=Ascend310 --insert_op_conf=./insert_op_yuv.cfg --input_shape="data:1,3,224,224" --input_format=NCHW**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./garbage_yuv.om $HOME/samples/python/contrib/garbage_picture/model/**

    4. 获取样例需要的测试图片。

        执行以下命令，进入样例的data文件夹中，下载对应的测试图片（或者自己找一些测试图片，放置到data目录中即可）。

        **cd $HOME/samples/python/contrib/garbage_picture/data**

        **wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/garbage_picture/newspaper.jpg**

        **wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/garbage_picture/bottle.jpg**

        **wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/garbage_picture/dirtycloth.jpg**

### 案例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **YOLOV3_mask_detection** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，切换目录并运行。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/python/contrib/garbage_picture/**     

      **python3.6 src/classify_test.py ./data/**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录并运行。
    
      **cd $HOME/garbage_picture/**

      **python3.6 src/classify_test.py ./data/** 


### 查看结果

运行完成后，会在outputs目录下生成带推理结果的jpg图片。
​       
