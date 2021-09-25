# 基于 Caffe ResNet-50 网络实现图片分类（图片解码+抠图缩放+图片编码+同步推理）

## 功能描述


该样例主要是基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类的功能。  
根据运行应用的入参，该样例可实现以下功能：
-  将*.jpg格式的解码成YUV420SP NV12格式的图片，缩放，再进行模型推理，得到推理结果后，处理推理结果，输出top5置信度。
-  将YUV420SP NV12格式的输入图片，抠图，再进行模型推理，得到推理结果后，处理推理结果，输出top5置信度。
-  将YUV420SP NV12格式的输入图片，抠图贴图，再进行模型推理，得到推理结果后，处理推理结果，输出top5置信度。
-  将YUV420SP NV12格式的输入图片，编码为jpg格式的图片。
-  将YUV420SP NV12格式的图片（分辨率8192\*8192）缩放，得到4000\*4000。
-  将YUV420SP NV12格式的输入图片，进行批量抠图。
-  将YUV420SP NV12格式的输入图片，进行批量抠图粘贴。

## 目录结构

如下为模型文件转换后的示例目录结构，model文件夹是转换后生成的。

```
resnet50_imagenet_classification
├──src
│ ├── acl_dvpp.py //图片缩放实现文件
│ ├── acl_model.py //模型推理实现文件
│ ├── acl_sample.py //运行文件
│ ├── acl_util.py //工具类函数实现文件
│ ├── acl_vdec.py //视频解码实现文件
│ └── constant.py //常量定义
├── data
│ └── vdec_h265_1frame_rabbit_1280x720.h265 //用户待处理的视频文件，由用户自行获取
├── caffe_model
│ ├── aipp.cfg
│ ├── resnet50.caffemodel //resnet50模型
│ └── resnet50.prototxt // resnet50模型的网络文件
└── model
  └── resnet50_aipp.om //推理模型
```

## 主要接口

| 功能                | 对应ACL模块        | ACL 接口函数                      | 功能说明                                |
|--------------------|-------------------|-----------------------------------|----------------------------------------|
| 资源初始化          | 初始化             | acl.init                          | 初始化ACL配置。                         |
|                    | Device管理         | acl.rt.set_device                 | 指定用于运算的Device。                  |
|                    | Context管理        | acl.rt.create_context             | 创建Context。                          |
|                    | Stream管理         | acl.rt.create_stream              | 创建Stream。                           |
|                    | 算子加载与执行      | acl.op.set_model_dir              | 加载模型文件的目录。                    |
| 模型初始化          | 模型加载与执行      | acl.mdl.load_from_file            | 从*.om文件加载模型到device侧。          |
|                    | 数据类型及操作接口   | acl.mdl.create_desc              | 创建模型描述数据类型。                   |
|                    | 数据类型及操作接口   | acl.mdl.get_desc                 | 获取模型描述数据类型。                   |
| 数据预处理          | 媒体数据模块-JPEG   | acl.media.dvpp_jpeg_decode_async  | 图形解码。                              |
|                    |                    | acl.media.dvpp_jpeg_encode_async  | 图形编码。                              |
|                    | 媒体数据模块-VPC    | acl.media.dvpp_vpc_crop_async     | 抠一张子图。                            |
|                    |                    | acl.media.dvpp_vpc_resize_async               | 将输入图片缩放到输出图片大小。 |
|                    |                    | acl.media.dvpp_vpc_crop_and_paste_async       | 抠图并粘贴到输出图片。        |
|                    |                    | acl.media.dvpp_vpc_batch_crop_async           | 抠多张子图。                 |
|                    |                    | acl.media.dvpp_vpc_batch_crop_and_paste_async | 抠多张子图并粘贴到输出图片。  |
|                    | 数据类型及操作接口  | acl.media.dvpp_create_roi_config  | 创建图片框区域配置。                     |
|                    |                    | acl.media.dvpp_set_roi_config系列接口 | 设置图片框区域配置参数。              |
|                    |                    | acl.media.dvpp_set_pic_desc系列接口   | 设置图片描述相关参数。                |
| 模型推理           | 模型加载与执行       | acl.mdl.execute                    | 执行模型同步推理。                      |
| 数据后处理         | 数据类型及操作接口   | acl.op.create_attr                 | 创建aclopAttr类型的数据。               |
|                    | 数据类型及操作接口  | acl.create_tensor_desc             | 创建aclTensorDesc类型的数据。           |
|                    | 数据类型及操作接口  | acl.get_tensor_desc_size           | 获取tensor描述占用的空间大小。           |
|                    | 数据类型及操作接口  | acl.create_data_buffer             | 创建aclDataBuffer类型的数据。           |
| 数据交互            | 内存管理           | acl.rt.memcpy                     | 数据传输，Host->Device或Device->Host。  |
|                    | 内存管理           | acl.media.dvpp_malloc             | 分配内存给Device侧媒体数据处理时使用。    |
|                    | 内存管理           | acl.rt.malloc                     | 申请Device上的内存。                    |
|                    | 内存管理           | acl.rt.malloc_host                | 申请Host上的内存。                      |
| 单算子推理          | 算子加载与执行      | acl.op.execute                    | 异步加载并执行指定的算子。              |
| 公共模块            | --                 | acl.util.ptr_to_numpy            | 指针转numpy类型数据。                    |
|                    | --                 | acl.util.numpy_to_ptr            | numpy类型数据转指针。                    |
| 资源释放            | 内存管理            | acl.rt.free                      | 释放Device上的内存。                     |
|                    | 内存管理            | acl.media.dvpp_free              | 通过acl.media.dvpp_malloc接口申请的内存。 |
|                    | 内存管理            | acl.rt.free_host                 | 释放Host上的内存。                       |
|                    | 模型加载与执行       | acl.mdl.unload                   | 卸载模型。                              |
|                    | Stream管理         | acl.rt.destroy_stream             | 销毁Stream。                           |
|                    | Context管理         | acl.rt.destroy_context           | 销毁Context。                           |
|                    | Device管理          | acl.rt.reset_device              | 复位当前运算的Device，回收Device上的资源。 |
|                    | 去初始化            | acl.finalize                     | 实现ACL去初始化。                        |

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
        从以下链接获取ResNet-50网络的权重文件（*.caffemodel）、模型文件（resnet50.prototxt），并以HwHiAiUser（运行用户）将获取的文件上传至开发环境的“vpc_jpeg_resnet50_imagenet_classification/caffe_model”目录下。
        - 从gitee上获取：单击[Link](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。
        - 从GitHub上获取：单击[Link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。

    4.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（*.om文件），转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片。
        切换到“vpc_jpeg_resnet50_imagenet_classification”目录，执行如下命令。 Ascendxxx为使用的昇腾AI处理器型号，请用户自行替换。
        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --output=model/resnet50_aipp --soc_version=Ascendxxx --insert_op_conf=caffe_model/aipp.cfg
        ```
        -   --output参数：生成的resnet50_aipp.om文件存放在“vpc_jpeg_resnet50_imagenet_classification/model”目录下。
        -   使用atc命令时用户需保证对vpc_jpeg_resnet50_imagenet_classification目录有写权限。

    5.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境。

2.  运行应用
    1.  登录运行环境。
    2.  准备输入图片。
        请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“vpc_jpeg_resnet50_imagenet_classification/data”目录下。如果目录不存在，需自行创建。  
        https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_vpc_8192x8192_nv12.yuv  
        https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/persian_cat_1024_1536_283.jpg  
        https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1061_330.jpg  
        https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1068_nv12.yuv  
    3.  参照[环境变量配置](#env)完成运行环境的配置。
    4.  在vpc_jpeg_resnet50_imagenet_classification路径下请用户根据场景选择对应的命令执行：  
        执行解码+缩放+推理：
        ```
        python3 ./src/main.py --images_path="./data/wood_rabbit_1024_1061_330.jpg" --dvpp_type=0 --image_type='jpg'
        ```
        执行抠图：
        ```
        python3 ./src/main.py --images_path="./data/wood_rabbit_1024_1068_nv12.yuv" --dvpp_type=1 --image_type='yuv'
        ```
        执行抠图粘贴：
        ```
        python3 ./src/main.py --images_path="./data/wood_rabbit_1024_1068_nv12.yuv" --dvpp_type=2 --image_type='yuv'
        ```
        执行编码：
        ```
        python3 ./src/main.py --images_path="./data/wood_rabbit_1024_1068_nv12.yuv" --dvpp_type=3 --image_type='yuv'
        ```
        8k缩放：  
        将data路径下dvpp_vpc_8192x8192_nv12.yuv文件名修改为dvpp_vpc_8192_8192_nv12.yuv
        ```
        python3 ./src/main.py --images_path="./data/dvpp_vpc_8192_8192_nv12.yuv" --dvpp_type=4 --image_type='yuv'
        ```
        执行批量抠图：
        ```
        python3 ./src/main.py --images_path="./data/wood_rabbit_1024_1068_nv12.yuv" --dvpp_type=5 --image_type='yuv' --in_batch_size=1 --out_batch_size=8
        ```
        执行批量抠图粘贴：
        ```
        python3 ./src/main.py --images_path="./data/wood_rabbit_1024_1068_nv12.yuv" --dvpp_type=6 --image_type='yuv' --in_batch_size=1 --out_batch_size=8
        ```

