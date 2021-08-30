# 基于 Caffe ResNet-50 网络实现图片分类（同步推理）

## 功能描述

该样例主要是基于Caffe ResNet-50网络（单输入、单batch）实现图片分类的功能。  
将Caffe ResNet-50网络的模型文件转换为适配昇腾AI处理器的离线模型（ \*.om文件），在样例中，加载该om文件，对2张\*.jpg图片进行同步推理，分别得到推理结果后，再对推理结果进行处理，输出top5置信度的类别标识。

## 目录结构

如下为模型文件转换后的示例目录结构，model文件夹是转换后生成的。

```
resnet50_imagenet_classification
├──src
│ ├── acl_net.py //运行文件
│ └── constant.py //常量定义
├── data
│ ├── dog1_1024_683.jpg //测试图片数据
│ └── dog2_1024_683.jpg //测试图片数据
├── caffe_model
│ ├── resnet50.caffemodel //resnet50模型
│ └── resnet50.prototxt // resnet50模型的网络文件
└── model
  └── resnet50.om //转换后的模型文件
```

## 主要接口

| 功能                | 对应ACL模块        | ACL 接口函数                      | 功能说明                                |
|--------------------|-------------------|-----------------------------------|----------------------------------------|
| 资源初始化          | 初始化             | acl.init                          | 初始化ACL配置。                         |
|                    | Device管理         | acl.rt.set_device                 | 指定用于运算的Device。                  |
|                    | Context管理        | acl.rt.create_context             | 创建Context。                          |
| 模型初始化          | 模型加载与执行      | acl.mdl.load_from_file            | 从*.om文件加载模型到device侧。          |
|                    | 数据类型及操作接口   | acl.mdl.create_desc              | 创建模型描述数据类型。                   |
|                    | 数据类型及操作接口   | acl.mdl.get_desc                 | 获取模型描述数据类型。                   |
| 模型推理            | 模型加载与执行      | acl.mdl.execute                   | 执行模型同步推理。                      |
| 数据交互            | 内存管理            | acl.rt.memcpy                    | 数据传输，Host->Device或Device->Host。  |
|                    | 内存管理            | acl.rt.malloc                    | 申请Device上的内存。                    |
|                    | 内存管理            | acl.rt.malloc_host               | 申请Host上的内存。                      |
| 公共模块            | --                 | acl.util.ptr_to_numpy            | 指针转numpy类型数据。                    |
|                    | --                 | acl.util.numpy_to_ptr            | numpy类型数据转指针。                    |
| 数据后处理          | 数据类型及操作接口   | acl.mdl.get_dataset_buffer       | 获取数据集中信息。                       |
|                    | 数据类型及操作接口   | acl.mdl.get_dataset_num_buffers  | 获取数据集中信息。                       |
| 资源释放            | 内存管理            | acl.rt.free                      | 释放Device上的内存。                     |
|                    | 内存管理            | acl.rt.free_host                 | 释放Host上的内存。                       |
|                    | 模型加载与执行       | acl.mdl.unload                   | 卸载模型。                              |
|                    | Context管理         | acl.rt.destroy_context           | 销毁Context。                           |
|                    | Device管理          | acl.rt.reset_device              | 复位当前运算的Device，回收Device上的资源。 |
|                    | 去初始化     | acl.finalize                     | 实现ACL去初始化。                        |

## <span id = "env">环境变量配置 </span>

在安装完CANN软件包之后，请务必自行配置pyACL相关的环境变量，否则，将无法正常“import acl”。

1. 若环境中安装了cann-toolkit软件包：
```
# 以root用户安装toolkit包
. /usr/local/Ascend/ascend-toolkit/set_env.sh
# 以非root用户安装toolkit包
. ${HOME}/Ascend/ascend-toolkit/set_env.sh
```
2. 若环境中安装了cann-nnrt软件包：
```
# 以root用户安装nnrt包
. /usr/local/Ascend/nnrt/set_env.sh
# 以非root用户安装nnrt包
. ${HOME}/Ascend/nnrt/set_env.sh
```
3. 若环境中安装了cann-nnae软件包：
```
# 以root用户安装nnae包
. /usr/local/Ascend/nnae/set_env.sh
# 以非root用户安装nnae包
. ${HOME}/Ascend/nnae/set_env.sh
```

-   **Atlas 500 智能小站：**
    1.  配置PYTHON环境变量，将“/home/data/miniD/driver/lib64”加入到PYTHON中。命令示例如下：
        ```
        export PYTHONPATH=/home/data/miniD/driver/lib64:$PYTHONPATH
        ```

    2. 配置LD\_LIBRARY\_PATH，将“/home/data/miniD/driver/lib64”加入到LD_LIBRARY_PATH中。命令示例如下：
        ```
        export LD_LIBRARY_PATH=/home/data/miniD/driver/lib64:$LD_LIBRARY_PATH
        ```

-   **Atlas 200 AI加速模块（ RC场景）：**
    1.  安装minirc版本的acllib包，以安装在默认路径“/usr/local/Ascend”为例：配置LD\_LIBRARY\_PATH，将“/usr/local/Ascend/acllib/lib64”加入到LD\_LIBRARY\_PATH中。命令示例如下：
        ```
        export LD_LIBRARY_PATH=/usr/local/Ascend/acllib/lib64:$LD_LIBRARY_PATH
        ```

    2.  获取aarch64架构的cann-nnrt软件包，在服务器上解压，在软件包所在目录下的解压命令如下：
        ```
        ./Ascend-cann-nnrt_xxx_linux-aarch64.run --noexec --extract="解压目标路径"
        ```
        在解压目标路径下找到“run_package/Ascend-pyACL-xxx-linux.aarch64.run”，在软件包所在目录下的解压命令如下：
        ```
        ./Ascend-pyACL-xxx-linux.aarch64.run --noexec --extract="解压目标路径"
        ```
        在解压路径下找到“python/site-packages/acl/acl.so”，将acl.so上传至200RC上，例如，上传至“ /usr/local/Ascend/pyACL/python/site-packages/acl”目录下，配置PYTHON环境变量，将“ /usr/local/Ascend/pyACL/python/site-packages/acl”加入到PYTHON中。命令示例如下：
        ```
        export PYTHONPATH=/usr/local/Ascend/pyACL/python/site-packages/acl:$PYTHONPATH
        ```

## 运行应用

1.  模型转换。
    1.  以HwHiAiUser（运行用户）登录开发环境。
    2.  参见《[CANN 开发辅助工具指南](https://support.huawei.com/enterprise/zh/doc/EDOC1100206690?idPath=23710424%7C251366513%7C22892968%7C251168373)》中的“ATC工具使用指南”章节中的ATC工具使用环境搭建，获取ATC工具并设置环境变量。

    3.  准备数据。
        从以下链接获取ResNet-50网络的权重文件（*.caffemodel）、模型文件（resnet50.prototxt），并以HwHiAiUser（运行用户）将获取的文件上传至开发环境的“resnet50_imagenet_classification/caffe_model”目录下。
        - 从gitee上获取：单击[Link](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。
        - 从GitHub上获取：单击[Link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。

    4.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（*.om文件）。
        切换到“resnet50_imagenet_classification”目录，执行如下命令。 Ascendxxx为使用的昇腾AI处理器型号，请用户自行替换。
        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --output=model/resnet50 --soc_version=Ascendxxx --input_format=NCHW --input_fp16_nodes=data --output_type=FP32 --out_nodes=prob:0
        ```
        -   --output参数：生成的resnet50.om文件存放在“resnet50_imagenet_classification/model”目录下。
        -   使用atc命令时用户需保证对resnet50_imagenet_classification目录有写权限。

    5.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境。

2.  运行应用
    1.  登录运行环境。
    2.  准备测试数据。
        请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“resnet50_imagenet_classification/data”目录下。如果目录不存在，需自行创建。  
        https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg  
        https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg  
    3.  参照[环境变量配置](#env)完成运行环境的配置。
    4.  在resnet50_imagenet_classification路径下执行如下命令：
        ```
        python3 ./src/acl_net.py
        ```