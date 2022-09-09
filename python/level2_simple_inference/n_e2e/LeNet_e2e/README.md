

# LeNet图像分类应用全流程开发（MindSpore训练+AscendCL推理）

* [1. 案例内容](#1-案例内容)
* [2. 案例目标](#2-案例目标)
* [3. 物料准备](#3-物料准备)
* [4. 环境准备](#4-环境准备)
* [5. 模型训练](#5-模型训练)
* [6. 应用开发](#6-应用开发)

## 1 案例内容

首先使用ModelArts训练图片分类LeNet模型，然后，使用Atlas200 DK/Atlas300(Ai1S)部署模型并进行图片分类，端到端掌握AI业务全流程开发实践技能。开发的流程如图所示：

![img](https://images.gitee.com/uploads/images/2021/0128/170026_bdfe13e3_5400693.png)

## 2 案例目标
- 目标是使用MindSpore训练LeNet模型（选用ModelArts平台） 
- 使用AscendCL基于LeNet模型编写推理应用（推理选用Atlas200DK/Atlas300(Ai1S)平台）

## 3 物料准备
- Liunx环境（Liunx系统）。
- [Atlas200 DK开发套件](https://e.huawei.com/cn/products/cloud-computing-dc/atlas/atlas-200/)/[Atlas300(ai1s)](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)

## 4 环境准备
体验LeNet图片分类AI应用的开发，需要完成以下准备工作。
1. **ModelArts训练准备工作**

    参考[ModelArts准备工作wiki](https://github.com/Ascend/samples/wikis/ModelArts%E5%87%86%E5%A4%87%E5%B7%A5%E4%BD%9C?sort_id=3466403)，完成ModelArts准备工作。包括注册华为云账号、ModelArts全局配置和OBS相关操作。

2. **Atlas推理准备工作（两种产品二选一即可）**

    - Atlas200 DK    
      （1）参考[制卡文档](https://support.huaweicloud.com/environment-deployment-Atlas200DK202/atlased_04_0001.html)进行SD卡制作，制卡成功后等待开发者板四个灯常亮即可。
    
      （2）参考[连接文档](https://support.huaweicloud.com/environment-deployment-Atlas200DK202/atlased_04_0014.html)中的**使用网线通过路由器连接Linux服务器**步骤，完成开发者板和本地机器的连接及开发者板上网配置。

      （3）配置完成后，参考[环境准备和依赖安装](https://github.com/Ascend/samples/blob/master/python/environment)准备好环境。

    - Atlas300（ai1s）
      （1）参考[购买并登录Linux弹性云服务器指南](https://support.huaweicloud.com/qs-ecs/zh-cn_topic_0132727313.html)购买AI加速型（ai1s）ECS弹性云服务器，选择镜像的时候选择公共的linux镜像即可。

      （2）参考[环境准备和依赖安装](https://github.com/Ascend/samples/blob/master/python/environment)准备好环境。


## 5 模型训练
这里我们选用AI开发平台ModelArts来进行训练，ModelArts是一个一站式的开发平台，能够支撑开发者从数据到AI应用的全流程开发过程。包含数据处理、模型训练、模型管理、模型部署等操作，并且提供AI Gallery功能，能够在市场内与其他开发者分享模型。

**ModelArts架构图：**

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/090401_9929617a_5403304.png "未命名1622118500.png")
​                                                                                                              
                               


我们在ModelArts中训练模型，模型训练完成后转换成昇腾芯片中可用的om模型。

**1、按照如下步骤在modelarts上部署数据集。**

- 下载数据集

本案例使用[MINST](<http://yann.lecun.com/exdb/mnist/>) 数据集，下载压缩包至本地，然后解压。里面包含了训练集60000张图片和测试集10000张图片。

- 下载训练代码

可以使用以下两种方式下载，请选择其中一种进行源码准备。

- 命令行方式下载。
  命令行中执行以下命令下载源码仓。
  **git clone https://gitee.com/mindspore/models.git**
- 压缩包方式下载。
  1. [mindspore/models仓](https://gitee.com/mindspore/models)右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。
  2. 解压zip包，进入models/official/cv/lenet目录，准备上传到OBS。

- 上传数据至OBS

windows环境中在OBS Browser+中，进入刚刚创建的“华为北京四”区域的OBS桶，然后点击上传按钮，上传本地文件夹**Date和models/official/cv/lenet**至OBS桶的一个文件目录下。

![](https://images.gitee.com/uploads/images/2021/0127/151112_1ab34d4a_5400693.png "uploadfolder.png")

如果没有下载OBS Browser+，可以直接在网页上的OBS桶中直接上传文件（每次最多上传100个文件）。

![](https://images.gitee.com/uploads/images/2021/0127/151157_837b2d9f_5400693.png "chromuploader.png")

**2、创建Notebook**

接下来将通过ModelArts的Notebook训练AI模型，使用ModelArts的MindSpore框架训练一个图片分类模型。

进入[ModelArts管理控制台](https://console.huaweicloud.com/modelarts/?region=cn-north-4#/manage/trainingjobs)，进入ModelArts“开发环境/Notebook”页面。

单击“ **创建** ”按钮，进入“ **创建Notebook** ”页面。

在“创建Notebook”页面，按照如下指导填写训练作业相关参数。

“计费模式”为系统自动生成，不需修改。

- 名称：自定义。
- 描述：描述信息，可选。
- 自动停止：选择“工作环境“后弹出，可自行选择时长。



![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/143823_4ddb9860_5578318.png "屏幕截图.png")


- 工作环境：展开“公共镜像”，选择 **Ascend-Powered-Engine 1.0（Python3）**。
- 资源池：默认选择“公共资源池”即可。
- 类型：默认选择“Ascend”即可。
- 规格：默认选择“Ascend：1*Ascend 910 cpu：24核 96 GIB”即可。。
- 存储配置：默认选择“对象存储服务（OBS）”即可。
- **存储位置：选择一个上传代码的路径，即OBS上训练代码和数据集所在的地方，如train-moderlarts/wy/，方便后续上传到modelarts。**
- 完成信息填写，单击“下一步”,规格确认无误后点击提交即可。



**3、配置训练作业**

如果Notebook没有启动，则在Notebook页签中启动创建的任务。

![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144721_3c542f7a_5578318.png "屏幕截图.png")


如果Notebook已经启动，则在Notebook页签中打开训练任务。

![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144636_dde5d6d1_5578318.png "屏幕截图.png")

打开后，进入到Jupyter页面，将所有文件同步到Modelarts中（当前展示的这些文件都是OBS上的数据，训练加载时需要在Modelarts的Notebook创建的环境中同步这些文件）。

![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144039_29ad0c80_5578318.png "屏幕截图.png")

![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144057_a5bd1737_5578318.png "屏幕截图.png")

![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144142_1d016461_5578318.png "屏幕截图.png")

同步完成后如下：

![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144157_137ee154_5578318.png "屏幕截图.png")


**4、训练步骤**

- **执行训练**

点击右上角的Open JupyterLab


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093046_23629234_5403304.png "444.png")

进入后
![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144223_38dcb321_5578318.png "屏幕截图.png")

点击Other->Terminal，使用命令行进入MindSpore的训练环境，执行

```
cat /home/ma-user/README
```

可以看到进入Mindspore的环境命令，执行

```
source /home/ma-user/miniconda3/bin/activate Mindspore-python3.7-aarch64
```

进入/home/ma-user/work目录，即可看到刚才同步OBS的文件目录。

由于部分代码经过windows上传会出现格式变化，这里可以执行以下命令做一些格式处理

```
sed -i 's/\r//g' train.py
```

执行以下命令开始训练

```
python train.py
```


执行流程如图

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/142147_9f9fae0c_5578318.png "屏幕截图.png")



当看到epoch = 10时，说明训练完成了。

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/142226_e69767ae_5578318.png "屏幕截图.png")



- **导出AIR格式模型**

这时候我们看到当前output目录下生成了ckpt文件，取生成的第一个即checkpoint_lenet-10_1875.ckpt，通过网络定义和CheckPoint生成AIR格式模型文件。

导出代码参考如下：

```
import sys 
from mindspore.train.serialization import load_checkpoint, save_checkpoint, export
sys.path.append("./src")
from lenet import LeNet5
import numpy as np
from mindspore import Tensor

network = LeNet5()
load_checkpoint("/home/ma-user/work/output/checkpoint_lenet-10_1875.ckpt", network)
input_data = np.random.uniform(0.0, 1.0, size = [1, 1, 32, 32]).astype(np.float32)
export(network, Tensor(input_data), file_name = './lenet', file_format = 'AIR') 

```

点击左上角，新建Mindspore的Notebook，

![输入图片说明](https://images.gitee.com/uploads/images/2021/0601/144358_331175a5_5578318.png "屏幕截图.png")

通过执行!pwd，我们看到当前路径是在/home/ma-user/work，网络定义文件所在路径是你自己的目录下的LeNet_for_MindSpore/src中。

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/142946_7fe31bd4_5578318.png "屏幕截图.png")


执行代码块，完成后会在/home/ma-user/work下生成.air文件。

- **上传模型到OBS**

这时候再把模型上传到OBS（关于[如何在Notebook中读写OBS文件？](https://support.huaweicloud.com/modelarts_faq/modelarts_05_0024.html)），我们通过moxing库上传，代码如下：

```
import moxing as mox
mox.file.copy('/home/ma-user/work/lenet.air', 'obs://train-moderlarts/wy/lenet/lenet.air')
```

执行成功后就可以在OBS的路径下看到自己的模型文件了，在OBS界面获取这个.air文件的链接，下载模型文件，准备做离线模型转换。



## 6 应用开发

### 6.1 实验原理



本实验是基于Atlas 200DK的图像分类项目，基于lenet图像分类网络编写的示例代码，该示例代码部署在Atlas 200DK上 ，通过读取本地图像数据作为输入，对图像中的数字进行识别分类，并将识别的结果展示出来

在本实验中，主要聚焦在Atlas 200 DK开发板上的应用案例移植环节，因此读者需要重点关注图片数据预处理及数据推理、检测结果后处理环节的操作。

完整的实验流程涉及到的模块介绍如下：

1. 预处理模块读取本地data目录下的jpg格式的图片，读取图片之后调用OpenCV的cvtColor函数将图片转为灰度图，然后对图像进行减均值和标准化操作后调用OpenCV的resize函数将图片缩放至模型需要的尺寸。
2. 推理模块接收经过预处理之后的图片数据，调用ACL库中模型推理接口进行模型推理。将推理得到的图片类别的置信度集合作为输出传给后处理模块。
3. 后处理模块接收推理结果，选取其中置信度最高的类别，作为图片分类的分类结果并将分类结果写入文件中。

### 6.2 实验流程

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/150427_93ebc289_5578318.png "屏幕截图.png")


 **图 6.2 LeNet图片分类应用案例移植流程图** 

在本实验中，默认已完成硬件环境和软件环境的准备工作，在此基础上进行LeNet图片分类应用项目的实验操作，由上图可知，本实验需要分别在Ubuntu主机PC端完成基于Python的LeNet图片分类应用代码的编写工作，以及LeNet图片分类模型转换，最后在Atlas 200 DK开发板上进行项目部署执行工作。

本案例移植的源代码编写及运行以链接
（[https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/lenet_mindspore_picture](https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/lenet_mindspore_picture)
）里的源码为例进行说明，实验任务及步骤将围绕图6.2所示四个方面分别展开介绍。

### 6.3 实验任务及步骤

 **任务一 实验准备** 

本实验使用Python进行开发，并使用命令行操作进行应用的部署和使用，因此我们选用官方提供的图像分类应用案例作为接下来开发的模板工程。图像分类应用案例可在[https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/lenet_mindspore_picture](https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/lenet_mindspore_picture)中进行下载。

参考该案例的README.md进行软件准备、部署、运行等步骤。确保环境配置无误，并能够得到正确的结果，即可进行下一步的开发。

 **任务二 模型转换** 

在完成LeNet图片模型的训练得到mindspore的LeNet.air算法模型之后，首先需要进行离线模型转换这一步骤，将mindspore的LeNet.air模型转换为Ascend 310芯片支持的模型（Davinci架构模型），才可进一步将其部署在Atlas 200 DK开发板上。

通过ATC命令对训练得到的mindspore的模型进行转化。

步骤 1 设置环境变量


```
export install_path=$HOME/Ascend/ascend-toolkit/latest
export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
export PYTHONPATH=${install_path}/atc/python/site-packages:${install_path}/atc/python/site-packages/auto_tune.egg/auto_tune:${install_path}/atc/python/site-packages/schedule_search.egg:$PYTHONPATH
export LD_LIBRARY_PATH=${install_path}/atc/lib64:$LD_LIBRARY_PATH
export ASCEND_OPP_PATH=${install_path}/opp
```

步骤 2   ATC转化

```
atc --framework=1 --model=lenet.air --output=mnist --soc_version=Ascend310
```

执行完之后会在当前执行ATC命令的目录下生成mnist.om文件

 **任务三 应用代码修改** 

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/144856_8fc0af32_5578318.png "屏幕截图.png")


**图6.1 LeNet图像分类实验原理图** 

完成以上步骤后，我们得到了所需要的网络模型。我们基于任务一获取的Python模板工程进行修改和补充，构建LeNet图片分类算法应用。接下来我们将对预处理模块、推理模块以及后处理模块的更新和补充进行介绍。

步骤 1  预处理模块

预处理模块读取本地data目录下的jpg格式的图片，读取图片之后调用OpenCV的cvtColor函数将图片转为灰度图，然后对图像进行减均值和标准化操作后调用OpenCV的resize函数将图片缩放至模型需要的尺寸。
该部分的代码如清单7.1所示，更详细的代码请查看项目代码。

清单7.1 预处理模块代码

```
    def preprocess(bgr_img):
        """
        preprocess
        """
        #get img shape
        orig_shape = bgr_img.shape[:2]

        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        #normalization
        gray_img = gray_img / 255.0
        gray_img = gray_img / 0.3081
        gray_img = gray_img - 1 * 0.1307 / 0.3081
        #resize img
        gray_img = cv2.resize(gray_img, (MODEL_WIDTH, MODEL_HEIGHT)).astype(np.float32)
        print(gray_img.shape)
        
        # save memory C_CONTIGUOUS mode
        if not gray_img.flags['C_CONTIGUOUS']:
            gray_img = np.ascontiguousarray(gray_img)

        return orig_shape, gray_img

```

步骤 2  推理模块

推理模块对应的函数为classify.py中的model.execute():，完成推理过程后，即可得到推理结果。

该部分的代码如清单7.2所示，更详细的代码请查看项目代码。

清单7.2推理模块

```
    result_list = model.execute([test_img, ])  
```

步骤 3  后处理模块
在得到推理模块输出的结果后，我们需要对其进行后处理，首先提取模型第一路输出得到置信度最高的类别并保存到本地文件。

该部分的代码如清单7.3所示，更详细的代码请查看项目代码。

清单7.3后处理模块

```
   def postprocess(infer_output, image_file):
        """
        post_process
        """
        print("post process")
        data = infer_output[0]
        vals = data.flatten()
        max_val=np.max(vals)
        vals = np.exp(vals - max_val)
        sum_val = np.sum(vals) 
        vals /= sum_val

        top_k = vals.argsort()[-1:-6:-1]
        print("images:{}".format(image_file))
        print("======== top5 inference results: =============")
        for n in top_k:
            object_class = image_net_classes.get_image_net_class(n)
            print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
        
        (filepath, tempfilename) = os.path.split(image_file)
        (filename, extension) = os.path.splitext(tempfilename)
        output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), filename + ".txt")	
        with open(output_path, "w", encoding="utf-8") as fp:
            fp.write(image_net_classes.get_image_net_class(top_k[0]))
     
```

 **任务四 应用运行** 
本应用的运行过程是在开发板上执行，需要将工程文件拷贝到开发板上。

我们在如下链接[lenet_mindspore_picture应用](https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/lenet_mindspore_picture)的readme中详细提供了运行本案例部署和运行步骤、脚本使用方法与各参数的意义供读者阅读与实验。

步骤 1 准备开发板运行环境

本次实验使用USB直连的方式连接Ubuntu服务器与开发板，开发板的IP地址为192.168.1.2，下文中涉及开发板IP地址的操作请替换为实际IP地址。

1)创建开发板工程主目录

如果已经创建过开发板工程主目录，则此步骤可跳过。

如果首次使用开发板，则需要使用如下命令创建开发板工程主目录：


```
ssh HwHiAiUser@192.168.1.2 "mkdir HIAI_PROJECTS"
```

提示password时输入开发板密码，开发板默认密码为Mind@123。

2)将应用代码（含转换后的离线模型）拷贝至开发板，由于代码中使用了公共接口库，所以需要把公共库文件也拷贝到开发板，这里我们直接把samples整个目录拷贝过去即可。

```
scp -r ~/AscendProjects/samples HwHiAiUser@192.168.1.2:/home/HwHiAiUser/HIAI_PROJECTS
```

提示password时输入开发板密码，开发板默认密码为Mind@123。

3)配置开发板环境变量

使用如下命令检查开发板是否已配置环境变量：

```
ssh HwHiAiUser@192.168.1.2 "cat ~/.bashrc | grep PATH"
```

如下图所示，如果打印输出包含如下红框中的内容则跳过此步骤：

![输入图片说明](https://images.gitee.com/uploads/images/2021/0127/094612_eae60264_8018002.png "屏幕截图.png")

如果上述命令打印输出不包含上图中红框的内容，则需要执行如下命令更新开发板环境变量配置：

```
ssh HwHiAiUser@192.168.1.2 "echo 'export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:\${LD_LIBRARY_PATH}' >> .bashrc ; echo 'export PYTHONPATH=/home/HwHiAiUser/Ascend/pyACL/python/site-packages/acl:\${PYTHONPATH}' >> .bashrc"
```

使用如下命令确认环境变量，下图中红框中的内容为更新的内容：

```
ssh HwHiAiUser@192.168.1.2 "tail -n8  .bashrc"
```

![输入图片说明](https://images.gitee.com/uploads/images/2021/0127/094702_63ca658c_8018002.png "屏幕截图.png")

步骤 2 准备推理输入数据

本实验的输入图片需要自行下载放到工程目录下的./data目录下。

```
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/lenet_mindspore/test_image/test1.png
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/lenet_mindspore/test_image/test2.png
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/lenet_mindspore/test_image/test3.png

```

用户可将要推理的图片存放于此目录作为推理输入数据。

步骤 3  登录开发板运行工程

1)使用如下命令登录开发板

```
ssh HwHiAiUser@192.168.1.2
```

2)进入拷贝至开发板中的工程目录，执行如下命令运行工程

```
cd HIAI_PROJECTS/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/src


python3.6 src/classify.py ./data/
```

3)查看工程运行完成后的推理结果，如下图

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/160829_98f3bc72_5578318.png "屏幕截图.png")

4)查看推理结果

推理产生的结果保存在outputs文件夹下的三个txt文件里，分别点开即可查看推理结果：

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/162948_3a5111b2_5578318.png "屏幕截图.png")