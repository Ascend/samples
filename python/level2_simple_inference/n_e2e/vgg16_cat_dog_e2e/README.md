**该案例仅仅用于学习，打通流程，不对效果负责，不支持商用。**

# ModelArts- jupyter +Ascend310 从ModelArts到线下部署，开发猫狗识别AI应用（图片输入+图片输出）

## 案例内容
此案例将带领开发者体验端云协同开发，首先使用ModelArts训练猫狗分类模型，然后，使用Atlas200 DK/Atlas300(ai1s)部署模型并进行猫狗识别，端到端掌握AI业务全流程开发实践技能。开发技能的流程如图所示：

![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/151245_103873e0_8113712.png "1.png")

## 案例目标
- 掌握使用ModelArts-训练猫狗分类模型。
- 掌握使用Atlas200 DK/Atlas300(ai1s)部署模型并跑通口罩识别样例。

## 物料准备
- Liunx环境（虚拟机或Liunx系统）。
- [Atlas200 DK开发套件](https://www.vmall.com/product/10086085080100.html?78119)/[Atlas300(ai1s)](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)

## 环境准备
体验猫狗大战AI应用的开发，需要完成以下准备工作。
1. **ModelArts训练准备工作**

    参考[ModelArts准备工作wiki](https://github.com/Ascend/samples/wikis/ModelArts%E5%87%86%E5%A4%87%E5%B7%A5%E4%BD%9C?sort_id=3466403)，完成ModelArts准备工作。包括注册华为云账号、ModelArts全局配置和OBS相关操作。

2. **Atlas推理准备工作**

    - Atlas200 DK 

      （1）参考[制卡文档](https://support.huaweicloud.com/dedg-A200dk_3000_c75/atlased_04_0013.html)进行SD卡制作，制卡成功后等待开发者板四个灯常亮即可。
    
      （2）参考[连接文档](https://support.huaweicloud.com/dedg-A200dk_3000_c75/atlased_04_0015.html)中的**使用网线通过路由器连接Ubuntu服务器**步骤，完成开发者板和本地机器的连接及开发者板上网配置。

      （3）配置完成后，参考[环境准备和依赖安装](https://github.com/Ascend/samples/blob/master/python/environment)准备好环境。

    - Atlas300（ai1s）

      （1）参考[购买并登录Linux弹性云服务器指南](https://support.huaweicloud.com/qs-ecs/zh-cn_topic_0132727313.html)购买AI加速型（ai1s）ECS弹性云服务器，并参考[卸载驱动固件和CANN软件文档](https://support.huaweicloud.com/instg-cli-cann/atlascli_03_0100.html)卸载预安装的老版本。

      （2）参考[CANN安装指南](https://support.huaweicloud.com/instg-cli-cann/atlascli_03_0017.html)配置ai1s的推理开发和运行环境。

      （3）配置完成后，参考[环境准备和依赖安装](https://github.com/Ascend/samples/blob/master/python/environment)准备好环境。

## 模型训练
使用ModelArts-Notebook完成模型训练。

### 数据集

猫狗大战数据集包含共计12500张猫和狗的图片，图片名称中含有类别名称（cat和dog），数据集已经存放到obs桶中，后期运行教程中的代码，即可下载到自己的notebook环境中。

### 进入ModelArts

   首先登录[ModelArts管理控制台](https://console.huaweicloud.com/modelarts/?region=cn-north-4#/manage/trainingjobs)，进入ModelArts控制台。

### 创建ModelArts Notebook

   ModelArts的Notebook提供网页版的Python开发环境，可以方便编写、运行代码，并查看运行结果。

   首先点击左侧“**开发环境**”-->“**Notebook**”，进入开发环境，然后单击“**创建**”按钮，进入“**创建Notebook**”页面。

   在“创建Notebook”页面，按照如下指导填写训练作业相关参数。

   - “计费模式”为系统自动生成，不需修改。

   - 名称：自定义。
   - 描述：描述信息，可选。

   ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/151628_da8e1a1d_8113712.png "1612422711569.png")

   - 工作环境：选择“ **Multi-Engine 1.0(Python3, Recommended)** ”环境    
   - 资源池：选择”**公共资源池**“
   - 类型：选择”**GPU**“
   - 规格：选择”**1*100NV32 CPU 8核 64GB**“，注意若没有选择限时免费注意设置停止时间
   - 存储配置：选择”**云硬盘（EVS）**“
   - 磁盘规格：磁盘规格按需选择，改应用”**5GB**“即可

   ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/151724_f81e5102_8113712.png "1612423079050.png")

   - 自动停止：设置自动停止时间，选择”**2小时后**“

   ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/151831_2fef4526_8113712.png "1612423764481.png")

   然后点击”**下一步**“，确认无误后点击"**提交**"，至此，ModelArts-Notebook创建完成。

### 4 在ModelArts中创建开发环境

   在开发环境主界面，待Notebook创建完毕后，创建一个实际的开发环境。

   ① 单击 **“打开”** ，进入刚刚创建的CatVSDog Notebook
   
   ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/151923_b9555c09_8113712.png "1612489770216.png")

   ② 创建一个Python-TensorFlow-1.13.1的Notebook环境
   点击右上角”**New**“-->"**TensorFlow-1.13.1**"，创建并进入开发环境

  ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/151955_57e82c01_8113712.png "1612490108785.png")
  ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/152018_1b3b75da_8113712.png "1612490476631.png")
  ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/152036_2efb21f6_8113712.png "1612490540208.png")
  
   ③ 点击左上方”**Untitled**“，修改文件名称。
   ![1612490540208](C:\Users\83395\AppData\Roaming\Typora\typora-user-images\1612490540208.png)

### 5 在ModelArts-Jupyter中编写并执行训练代码

   **① 安装Keras, Keras_applications版本配置以及数据集下载**

   In [1]  :

    !pip install --upgrade keras_applications==1.0.6 keras==2.2.4 

   In [2]:

    from modelarts.session import Session
    import os
    session = Session()
    if session.region_name == 'cn-north-1':
        bucket_path="modelarts-labs/end2end/image_recognition/dog_and_cat_25000.tar.gz"
    elif session.region_name == 'cn-north-4':
        bucket_path="modelarts-labs-bj4/end2end/image_recognition/dog_and_cat_25000.tar.gz"
    else:
        print("请更换地区到北京一或北京四")
    if not os.path.exists('./data'):
        session.download_data(
        bucket_path= bucket_path,
        path="./dog_and_cat_25000.tar.gz")
        # 使用tar命令解压资源包
        !tar xf ./dog_and_cat_25000.tar.gz
        # 清理压缩包
    !rm -f ./dog_and_cat_25000.tar.gz

   In [3]:

    !mkdir model

 **②  引入相关包**

   In [4]  :

    import numpy as np
    from keras.preprocessing import image
    from keras.models import Model
    from keras.layers import Dense, GlobalAveragePooling2D
    from keras import backend as K
    from keras.models import load_model
    from keras.preprocessing.image import ImageDataGenerator
    from keras.applications.vgg16 import VGG16

**③  读取数据**
  In [5]  :

    import os
    from PIL import Image
    def load_data():
        dirname = "./data"
        path = "./data"
    
        num_train_samples = 25000
    
        x_train = np.empty((num_train_samples, 224,224,3), dtype='uint8')
        y_train = np.empty((num_train_samples,1), dtype='uint8')
        index = 0
        for file in os.listdir("./data"):
            image = Image.open(os.path.join(dirname,file)).resize((224,224))
            image = np.array(image)
            x_train[index,:,:,:] = image
    
            if "cat" in file:
                y_train[index,0] =1
            elif "dog" in file:
                y_train[index,0] =0
    
            index += 1
        return (x_train, y_train)


In [6]  :

    (x_train, y_train) = load_data()
    print(x_train.shape)
    print(y_train.shape)

 **注：需等待约7分钟左右加载数据**

**④ 数据处理**
  In [7]  :

    from keras.utils import np_utils
    def process_data(x_train,y_train):
        x_train = x_train.astype(np.float32)
        x_train /= 255
        n_classes = 2
        y_train = np_utils.to_categorical(y_train, n_classes)
        return x_train,y_train

  In [8]  : 

    x_train,y_train= process_data(x_train,y_train)
    print(x_train.shape)
    print(y_train.shape)

   **⑤ 预训练模型准备，使用ImageNet-VGG16预训练模型**

* [下载VGG-16模型](https://github.com/fchollet/deep-learning-models/releases/download/v0.1/vgg16_weights_tf_dim_ordering_tf_kernels_notop.h5)，下载完成后，点击”**Upload**“将上传到jupty当前工作目录。

  ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/152150_6dd13d16_8113712.png "1612510992463.png")

  ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/152216_e68904f4_8113712.png "1612511009244.png")

* 点击”**New**“-->"**Terminal**"进入到终端界面，TensorFlow-1.13.1环境中。

  ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/152248_6ec0c322_8113712.png "1612511092785.png")

* 将预训练模型移动到指定位置


``` 
  source /home/ma-user/anaconda3/bin/activate TensorFlow-1.13.1
  cd $HOME
  mkdir -p .keras/models
  mv work/vgg16_weights_tf_dim_ordering_tf_kernels_notop.h5 .keras/models/
```


  执行完可在 $HOME/.keras/models目录下看到预训练模型

  ![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/152316_18bb0f0d_8113712.png "1612511505029.png")

**⑥  网络构建**

加载预训练模型，只在最后的全连接层进行调整，使用GlobalAveragePooling2D将7x7x512的卷积结果进行全局池化，减少训练参数，并加入softmax激活、output shape为2的全连接层进行二分类。

In [9]  :

  ```
def build_model(base_model):
    x = base_model.output
    x = GlobalAveragePooling2D()(x)
    predictions = Dense(2, activation='softmax')(x)
    model = Model(inputs=base_model.input, outputs=predictions)
    print(type(model))
    return model
  ```

include_top=False表示只取卷积网络结构中的参数，不包含全连接层和softmax分类层（ImageNet有1000个分类）。

  ln[10]:

  ```  
base_model = VGG16(weights='imagenet', include_top=False)
  ```

把所有卷基层的trainable设置为False，不进行训练。然后将池化层和全连接的二分类层添加到模型中，输入层不变.

ln[11]:

  ```  
for layer in base_model.layers:
    layer.trainable = False

model = build_model(base_model)
model.summary()
  ```

**⑦  设置模型的损失函数和优化器**

ln[12]:

  ```  
import keras 
opt = keras.optimizers.rmsprop(lr=0.0001, decay=1e-6)
model.compile(loss='binary_crossentropy',
              optimizer=opt,
              metrics=['accuracy'])
  ```

**⑧ 设置callback，并训练模型**

ln[13]:

  ```  
from keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau

es = EarlyStopping(monitor='val_acc', baseline=0.9, patience=15, verbose=1, mode='auto')
cp = ModelCheckpoint(filepath="./ckpt_vgg16_dog_and_cat.h5", monitor="val_acc", verbose=1, save_best_only=True, mode="auto", period=1)
lr = ReduceLROnPlateau(monitor="val_acc", factor=0.1, patience=10, verbose=1, mode="auto", min_lr=0)
callbacks = [es,cp,lr]
  ```

  ln[14]:

  ```  
history = model.fit(x=x_train, 
                    y=y_train, 
                    batch_size=16, 
                    epochs=15, 
                    verbose=1, 
                    callbacks=callbacks, 
                    validation_split=0.25, 
                    shuffle=True, 
                    initial_epoch=0)
  ```

ln[15]:

```  
save_path = "./model/"
import os
model.save(save_path + 'vgg16_cat_dog.h5')
```

**⑨ h5转成pb模型**

ln[16]:

```  
import tensorflow as tf
import keras
session = keras.backend.get_session()
min_graph = tf.graph_util.convert_variables_to_constants(session, session.graph_def, [node.op.name for node in model.outputs])
tf.train.write_graph(min_graph, "model", "vgg16_cat_dog.pb", as_text=False)
```

点击”**Download**“将训练好的模型下载到本地，以便后续进行模型转换使用

![输入图片说明](https://images.gitee.com/uploads/images/2021/0207/152349_3aeec550_8113712.png "1612513761976.png")

## 模型推理

**详细案例部署步骤可以参考[vgg16_cat_dog_picture样例](../../1_classification/vgg16_cat_dog_picture)中Readme进行相关部署和运行，其中模型替换为自己训练出来的模型即可。**

### 案例部署

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

        开发环境，非root用户命令行中执行以下命令下载源码仓。

       ```
       cd $HOME
       git clone https://github.com/Ascend/samples.git
       ```

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

        2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

        3. 开发环境中，执行以下命令，解压zip包。

       ```
       cd $HOME
       unzip ascend-samples-master.zip         
       ```
   
2. 模型转换

   1）  将从MobelArts Download的pb模型放置到开发环境普通用户下的工程目录 ，如：$HOME/samples/python/level2_simple_inference/1_classification/vgg16_cat_dog_picture/model

   2）  设置LD_LIBRARY_PATH环境变量

      ```
   export LD_LIBRARY_PATH=${install_path}/atc/lib64
      ```

   3） 执行以下命令使用ATC命令将原始模型转换为Davinci模型
   
      ```
   atc --output_type=FP32 --input_shape="input_1:1,224,224,3"  --input_format=NHWC --output="vgg16_cat_dog" --soc_version=Ascend310 --insert_op_conf=insert_op.cfg --framework=3 --model="./vgg16_cat_dog.pb"
      ```
   
4. 获取样例需要的测试图片。
   
   执行以下命令，进入样例的data文件夹中，下载对应的测试图片（或者自己找一些测试图片，放置到data目录中）。
   
      ```
      cd $HOME/samples/python/level2_simple_inference/1_classification/vgg16_cat_dog_picture/data
      wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vgg16_cat_dog/test_image/cat.jpg
      wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vgg16_cat_dog/test_image/dog.jpg
      ```

### 案例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **vgg16_cat_dog_picture** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
  ```
scp -r $HOME/samples/python/level2_simple_inference/1_classification/vgg16_cat_dog_picture/  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
scp -r $HOME/samples/python/common/atlas_utils/   HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
ssh HwHiAiUser@xxx.xxx.xxx.xxx
  ```

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
> - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. 运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      ```
      export LD_LIBRARY_PATH=
      source ~/.bashrc
      cd $HOME/samples/python/level2_simple_inference/1_classification/vgg16_cat_dog_picture/src     
      python3 main.py 
      ```
      
    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。

      ```
      cd $HOME/python/vgg16_cat_dog_picture/src
      ```
      切换目录后，执行以下命令运行样例。
      ```
      python3.6 main.py
      ```    

 ### 查看结果

运行完成后，会在out目录下生成带推理结果的jpg图片。