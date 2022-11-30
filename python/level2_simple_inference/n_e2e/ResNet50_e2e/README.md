

# ResNet50图片分类应用全流程开发（MindSpore训练+AscendCL推理）

* [1. 案例内容](#1-案例内容)
* [2. 案例目标](#2-案例目标)
* [3. 物料准备](#3-物料准备)
* [4. 环境准备](#4-环境准备)
* [5. 模型训练](#5-模型训练)
* [6. 应用开发](#6-应用开发)

## 1 案例内容

首先使用ModelArts训练图片分类resnet50模型，然后，使用Atlas200 DK/Atlas300(Ai1S)部署模型并进行图片分类，端到端掌握AI业务全流程开发实践技能。开发的流程如图所示：

![img](https://images.gitee.com/uploads/images/2021/0128/170026_bdfe13e3_5400693.png)

## 2 案例目标
- 目标是使用MindSpore训练ResNet50模型（选用ModelArts平台） 
- 使用AscendCL基于ResNet50模型编写推理应用（推理选用Atlas200DK/Atlas300(Ai1S)平台）

## 3 物料准备
- Liunx环境（Liunx系统）。
- [Atlas200 DK开发套件](https://e.huawei.com/cn/products/cloud-computing-dc/atlas/atlas-200/)/[Atlas300(ai1s)](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)

## 4 环境准备
体验ResNet50图片分类AI应用的开发，需要完成以下准备工作。
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

## 5 模型训练
这里我们选用AI开发平台ModelArts来进行训练，ModelArts是一个一站式的开发平台，能够支撑开发者从数据到AI应用的全流程开发过程。包含数据处理、模型训练、模型管理、模型部署等操作，并且提供AI Gallery功能，能够在市场内与其他开发者分享模型。

**ModelArts架构图：**

![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/090401_9929617a_5403304.png "未命名1622118500.png")
​                                                                                                              
                               


我们在ModelArts中训练模型，模型训练完成后转换成昇腾芯片中可用的om模型。

**1、按照如下步骤在modelarts上部署数据集。**

- 下载数据集

本案例使用[CIFAR-10](http://www.cs.toronto.edu/~kriz/cifar.html)数据集，点击[此链接](http://www.cs.toronto.edu/~kriz/cifar-10-binary.tar.gz)，下载压缩包至本地，然后解压。

解压后，可以看到一些bin文件，里面包含了训练集50000张图片和测试集10000张图片。

- 下载训练代码

可以使用以下两种方式下载，请选择其中一种进行源码准备。

- 命令行方式下载。
  命令行中执行以下命令下载源码仓。
  **git clone https://github.com/Ascend/modelzoo.git**
- 压缩包方式下载。
  1. [modelzoo仓](https://github.com/Ascend/modelzoo)右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。
  2. 解压zip包，进入modelzoo/built-in/MindSpore/Official/cv/image_classification/ResNet50_for_MindSpore目录，准备上传到OBS。

- 上传数据至OBS

windows环境中在OBS Browser+中，进入刚刚创建的“华为北京四”区域的OBS桶，然后点击上传按钮，上传本地文件夹**cifar-10-batches-bin和modelzoo/built-in/MindSpore/Official/cv/image_classification/ResNet50_for_MindSpore**至OBS桶的一个文件目录下。

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



![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/091204_a3c692c3_5403304.png "image-20210525153814870.png")


- 工作环境：展开“公共镜像”，选择 **Ascend-Powered-Engine 1.0（Python3）**。
- 资源池：默认选择“公共资源池”即可。
- 类型：默认选择“Ascend”即可。
- 规格：默认选择“Ascend：1*Ascend 910 cpu：24核 96 GIB”即可。。
- 存储配置：默认选择“对象存储服务（OBS）”即可。
- **存储位置：选择一个上传代码的路径，即OBS上训练代码和数据集所在的地方，如/modelart-xuyetao/resnet50/，方便后续上传到modelarts。**
- 完成信息填写，单击“下一步”,规格确认无误后点击提交即可。



**3、配置训练作业**

如果Notebook没有启动，则在Notebook页签中启动创建的任务。



![启动.png](https://images.gitee.com/uploads/images/2021/0129/141119_8c8f5300_5400693.png)

如果Notebook已经启动，则在Notebook页签中打开训练任务。

![open.png](https://images.gitee.com/uploads/images/2021/0129/141412_80014968_5400693.png)

打开后，进入到Jupyter页面，将所有文件同步到Modelarts中（当前展示的这些文件都是OBS上的数据，训练加载时需要在Modelarts的Notebook创建的环境中同步这些文件）。


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/091253_f8113b27_5403304.png "01.png")


同步完成后如下：



![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/091315_cd547436_5403304.png "02.png")


**4、训练步骤**

- **执行训练**

点击右上角的Open JupyterLab


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093046_23629234_5403304.png "444.png")

进入后


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093112_2183c352_5403304.png "3.png")


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
cd ResNet50_for_MindSpore/scripts/
sed -i 's/\r//g' run_standalone_train.sh
```

执行以下命令开始训练

```
bash run_standalone_train.sh resnet50 cifar10 /home/ma-user/work/cifar-10-batches-bin/
```

可以在当前路径下的train目录执行命令看到log

```
cd train
tail -f log
```

执行流程如图


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093200_80a88229_5403304.png "05.png")


当看到epoch = 90时，说明训练完成了。


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093224_2c0d183a_5403304.png "8.png")


- **导出AIR格式模型**

这时候我们看到当前train目录下生成了ckpt文件，取生成的最后一个即resnet-90_1875.ckpt，通过网络定义和CheckPoint生成AIR格式模型文件。

导出代码参考如下：

```
import numpy as np
from mindspore import Tensor, export, load_checkpoint, load_param_into_net
import sys
sys.path.append("./resnet/ResNet50_for_MindSpore/scripts/train/src")
from resnet import *

resnet = resnet50()
# load the parameter into net
load_checkpoint("/home/ma-user/work/resnet/ResNet50_for_MindSpore/scripts/train/resnet-90_1875.ckpt", net=resnet)
input = np.random.uniform(0.0, 1.0, size=[1, 3, 224, 224]).astype(np.float32)
export(resnet, Tensor(input), file_name='resnet50-90_1875', file_format='AIR')
```

点击左上角，新建Mindspre的Notebook，


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093707_450db74a_5403304.png "01.png")


通过执行!pwd，我们看到当前路径是在/home/ma-user/work，网络定义文件所在路径是你自己的目录下的ResNet50_for_MindSpore/scripts/train/src中。


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093739_6357a520_5403304.png "02.png")


执行代码块，完成后会在/home/ma-user/work下生成.air文件。

- **上传模型到OBS**

这时候再把模型上传到OBS（关于[如何在Notebook中读写OBS文件？](https://support.huaweicloud.com/modelarts_faq/modelarts_05_0024.html)），我们通过moxing库上传，代码如下：

```
import moxing as mox
mox.file.copy('/home/ma-user/work/resnet50-90_1875.air', 'obs://modelart-xuyetao/resnet50/resnet-90_1875.air')
```

执行成功后就可以在OBS的路径下看到自己的模型文件了，在OBS界面获取这个.air文件的链接，下载模型文件，准备做离线模型转换。



## 6 应用开发

### 6.1 实验原理


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093824_7dd1d3c2_5403304.png "readme.png")


**图6.1 ResNet50图片分类实验原理图** 

本实验是基于Atlas 200DK的图像分类项目，基于ResNet50图片分类网络编写的示例代码，该示例代码部署在Atlas 200DK上 ，通过读取本地图像数据作为输入，对图像中的物体进行识别分类，并将分类的结果展示出来

在本实验中，主要聚焦在Atlas 200 DK开发板上的应用案例移植环节，因此读者需要重点关注图片数据预处理及数据推理、检测结果后处理环节的操作。

完整的实验流程涉及到的模块介绍如下：

1. 预处理模块读取本地data目录下的jpg格式的图片，读取图片之后调用python的pillow模块里的resize函数将图片缩放至模型需要的尺寸，然后进行图像色域转换、归一化、减均值和标准化操作后将数据排布格式转换成NCHW后得到预处理后的数据。
2. 推理模块接收经过预处理之后的图片数据，调用ACL库中模型推理接口进行模型推理。将推理得到的图片类别的置信度集合作为输出传给后处理模块。
3. 后处理模块接收推理结果，选取其中置信度最高的类别，作为图片分类的分类结果，并使用PIL将分类结果写入图片中。

### 6.2 实验流程


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093841_2aac1b34_5403304.png "readme2.png")


 **图 6.2 ResNet50图片分类应用案例移植流程图** 

在本实验中，默认已完成硬件环境和软件环境的准备工作，在此基础上进行ResNet50图片分类应用项目的实验操作，由上图可知，本实验需要分别在Ubuntu主机PC端完成基于Python的ResNet50图片分类应用代码的编写工作，以及ResNet50图片分类模型转换，最后在Atlas 200 DK开发板上进行项目部署执行工作。

本案例移植的源代码编写及运行以链接
（[https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/resnet50_mindspore_picture](https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/resnet50_mindspore_picture)
）里的源码为例进行说明，实验任务及步骤将围绕图6.2所示四个方面分别展开介绍。

### 6.3 实验任务及步骤

 **任务一 实验准备** 

本实验使用Python进行开发，并使用命令行操作进行应用的部署和使用，因此我们选用官方提供的图像分类应用案例作为接下来开发的模板工程。图像分类应用案例可在[https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/resnet50_mindspore_picture](https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/resnet50_mindspore_picture)中进行下载。

参考该案例的README.md进行软件准备、部署、运行等步骤。确保环境配置无误，并能够得到正确的结果，即可进行下一步的开发。

 **任务二 模型转换** 

在完成ResNet50图片模型的训练（训练可参考**（文档未上传）**）得到mindspore的resnet50-90_1875.air算法模型之后，首先需要进行离线模型转换这一步骤，将mindspore的resnet50-90_1875.air模型转换为Ascend 310芯片支持的模型（Davinci架构模型），才可进一步将其部署在Atlas 200 DK开发板上。

通过ATC命令对训练得到的mindspore的模型进行转化。

步骤 1 设置环境变量


```
export install_path=$HOME/Ascend/ascend-toolkit/latest
export PATH=/usr/local/python3.7.5/bin:${install_path}/compiler/ccec_compiler/bin:${install_path}/compiler/bin:$PATH
export PYTHONPATH=${install_path}/compiler/python/site-packages:${install_path}/compiler/python/site-packages/auto_tune.egg/auto_tune:${install_path}/compiler/python/site-packages/schedule_search.egg:$PYTHONPATH
export LD_LIBRARY_PATH=${install_path}/compiler/lib64:$LD_LIBRARY_PATH
export ASCEND_OPP_PATH=${install_path}/opp
```

步骤 2   ATC转化

```
atc --model=./resnet-90_1875.air --framework=1 --output=./resnet50 --soc_version=Ascend310
```

执行完之后会在当前执行ATC命令的目录下生成resnet50.om文件

 **任务三 应用代码修改** 

完成以上步骤后，我们得到了所需要的网络模型。我们基于任务一获取的Python模板工程进行修改和补充，构建ResNet50图片分类算法应用。接下来我们将对预处理模块、推理模块以及后处理模块的更新和补充进行介绍。

步骤 1  预处理模块

预处理模块读取本地data目录下的jpg格式的图片，读取图片之后调用python的pillow模块里的resize函数将图片缩放至模型需要的尺寸，然后进行图像色域转换、归一化、减均值和标准化操作后将数据排布格式转换成NCHW后得到预处理后的数据，最后输入模型进行推理。

该部分的代码如清单7.1所示，更详细的代码请查看项目代码。

清单7.1 预处理模块代码

```
    def pre_process(self, image):
        """preprocess"""
        input_image = Image.open(image)
        input_image = input_image.resize((224, 224))
        # hwc
        img = np.array(input_image)
        height = img.shape[0]
        width = img.shape[1]
        h_off = int((height - 224) / 2)
        w_off = int((width - 224) / 2)
        crop_img = img[h_off:height - h_off, w_off:width - w_off, :]
        # rgb to bgr
        print("crop shape = ", crop_img.shape)
        img = crop_img[:, :, ::-1]
        shape = img.shape
        print("img shape = ", shape)
        img = img.astype("float32")
        img[:, :, 0] *= 0.003922
        img[:, :, 1] *= 0.003922
        img[:, :, 2] *= 0.003922
        img[:, :, 0] -= 0.4914
        img[:, :, 0] = img[:, :, 0] / 0.2023
        img[:, :, 1] -= 0.4822
        img[:, :, 1] = img[:, :, 1] / 0.1994
        img[:, :, 2] -= 0.4465
        img[:, :, 2] = img[:, :, 2] / 0.2010
        img = img.reshape([1] + list(shape))
        # nhwc -> nchw
        result = img.transpose([0, 3, 1, 2]).copy()
        return result
```

步骤 2  推理模块

推理模块对应的函数为classify_test.py中的 inference(self, resized_image):，完成推理过程后，即可得到推理结果。

该部分的代码如清单7.2所示，更详细的代码请查看项目代码。

清单7.2推理模块

```
    def inference(self, resized_image):
        return self._model.execute([resized_image, ])
```

步骤 3  后处理模块
在得到推理模块输出的结果后，我们需要对其进行后处理，首先提取模型第一路输出得到置信度最高的类别，使用pillow将置信度最高的类别写在图片上，并保存到本地文件。

该部分的代码如清单7.3所示，更详细的代码请查看项目代码。

清单7.3后处理模块

```
    def post_process(self, infer_output, image_file):
        """postprocess"""
        print("post process")
        data = infer_output[0]
        print("data shape = ", data.shape)
        vals = data.flatten()
        max = 0
        sum = 0
        for i in range(0, 10):
            if vals[i] > max:
                max = vals[i] 
        for i in range(0, 10):
            vals[i] = np.exp(vals[i] - max)
            sum += vals[i]
        for i in range(0, 10):
            vals[i] /= sum
        print("vals shape = ", vals.shape)
        top_k = vals.argsort()[-1:-6:-1]
        print("images:{}".format(image_file))
        print("======== top5 inference results: =============")
        for n in top_k:
            object_class = get_resnet50_class(n)
            print("label:%d  confidence: %f, class: %s" % (n, vals[n], object_class))
        
        #using pillow, the category with the highest confidence is written on the image and saved locally
        if len(top_k):
            object_class = get_resnet50_class(top_k[0])
            output_path = os.path.join(os.path.join(SRC_PATH, "../outputs"), os.path.basename(image_file))
            origin_img = Image.open(image_file)
            draw = ImageDraw.Draw(origin_img)
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size=20)
            draw.text((10, 50), object_class, font=font, fill=255)
            origin_img.save(output_path)
```

 **任务四 应用运行** 
本应用的运行过程是在开发板上执行，需要将工程文件拷贝到开发板上。

我们在如下链接[https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/resnet50_mindspore_picture](https://github.com/Ascend/samples/tree/master/python/level2_simple_inference/1_classification/resnet50_mindspore_picture)的readme中详细提供了运行本案例部署和运行步骤、脚本使用方法与各参数的意义供读者阅读与实验。

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
ssh HwHiAiUser@192.168.1.2 "echo 'export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/runtime/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib:\${LD_LIBRARY_PATH}' >> .bashrc ; echo 'export PYTHONPATH=/home/HwHiAiUser/Ascend/pyACL/python/site-packages/acl:\${PYTHONPATH}' >> .bashrc"
```

使用如下命令确认环境变量，下图中红框中的内容为更新的内容：

```
ssh HwHiAiUser@192.168.1.2 "tail -n8  .bashrc"
```

![输入图片说明](https://images.gitee.com/uploads/images/2021/0127/094702_63ca658c_8018002.png "屏幕截图.png")

步骤 2 准备推理输入数据

本实验的输入图片需要自行下载放到工程目录下的./data目录下。

```
wget https://modelart-xuyetao.obs.cn-north-4.myhuaweicloud.com/resnet50/airplane.jpg 
wget https://modelart-xuyetao.obs.cn-north-4.myhuaweicloud.com/resnet50/car.jpg
```

用户可将要推理的图片存放于此目录作为推理输入数据。

步骤 3  登录开发板运行工程

1)使用如下命令登录开发板

```
ssh HwHiAiUser@192.168.1.2
```

2)进入拷贝至开发板中的工程目录，执行如下命令运行工程

```
cd HIAI_PROJECTS/samples/python/level2_simple_inference/1_classification/resnet50_mindspore_picture/src
python3 src/classify_test.py ./data/
```

3)查看工程运行完成后的推理结果，如下图


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/093932_6ad8dfd5_5403304.png "03.png")


4)查看推理图片

推理产生的结果图片保存在outputs文件夹


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/094006_ba2c31ab_5403304.png "04.png")


将推理结果图片从Atlas200dk拷贝至本地Ubuntu的家目录中查看。在本地Ubuntu执行如下命令进行拷贝：

```
scp -r HwHiAiUser@192.168.1.2:~/HIAI_PROJECTS/samples/python/level2_simple_inference/1_classification/resnet50_mindspore_picture/outputs ~
```

在本地Ubuntu中查看拷贝后的推理结果图片，如下：


![输入图片说明](https://images.gitee.com/uploads/images/2021/0528/094033_0c4c6793_5403304.png "05.png")

到这里，我们就完成了整个ResNet50图片分类应用全流程开发（MindSpore训练+AscendCL推理）的这个实验。













