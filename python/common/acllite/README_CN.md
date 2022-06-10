中文|[English](README.md)
# python acllite使用说明

## 使用说明

1.本库仅供当前社区开源样例使用，不覆盖ascend平台应用开发的所有场景，不作为用户应用开发的标准库；

2.本库仅在Atlas200DK和Atlas300（x86）服务器上做了验证。

## 使用约束

| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | pyav、numpy、Pillow等| 请参考[第三方依赖安装指导（python样例）](../../environment)完成对应依赖安装 |


## 使用步骤

#### 2.公共库部署

运行应用前需要先将公共库部署到运行环境，具体安装步骤参考[第三方依赖安装指导（python样例）](../../environment)。
完成后，将该目录路径加入PYTHONPATH环境变量即可。例如在以上文档中，公共库被拷贝到```$THIRDPART_PATH```，在~/.bashrc文件中添加：

```
export PYTHONPATH=$THIRDPART_PATH/acllite:$PYTHONPATH
```

保存配置好的环境变量，执行以下命令使环境变量生效

```
source ~/.bashrc
```

完成以上步骤后，即可在应用的代码中调用acllite库的接口，例如：

```
import presenteragent.presenter_channel as presenter_channel

chan = presenter_channel.open_channel(config_file)
```

## 接口说明

### AclLiteImage类

AclLiteImage类为python-acllite公共库，针对atlas200dk摄像头、jpg图片、输入的图片数据，所提供的一套统一的数据封装结构，便于后续公共库接口对其进行处理。

#### \_\_init\_\_

方法：\_\_init\_\_ (image, width=0, height=0, size=0, memory_type=const.MEMORY_NORMAL):

说明：

根据初始化参数列表，生成对应的AclLiteImage结构的数据。

输入参数：

image ：图片数据，支持numpy array、jpeg/png图片路径、内存数据作为参数输入；不填/填写不支持类型输入会报错。

width ：图片宽；如果image为jpeg/png图片，该参数可不填；不填则填入默认参数0。

height ：图片高；如果image为jpeg/png图片，该参数可不填；不填则填入默认参数0。

size ：图片数据大小，如果image为jpeg/png图片，该参数可不填；不填则填入默认参数0。

memory_type ：图片数据存储类型，即该图片数据是存储在一般内存、device、host或是dvpp内存；如果image为jpeg/png图片，该参数可不填；不填则填入默认参数 MEMORY_NORMAL。

返回值：

AclLiteImage结构的数据

约束：

无

#### save

方法：save(filename):

说明：

将AclLiteImage数据转换为np array后。保存为二进制文件。

输入参数：

filename ： 保存后的文件名

返回值：

无

约束：

无

### Camera类

Camera类为Atlas200DK板载摄像头解码提供python接口。

#### is_opened

方法：is_opened（）

说明：

根据初始化的cameracapture类对象的摄像头id，判断Atlas200DK板载摄像头是否已打开。

输入参数：

无

返回值：

1.TRUE，摄像头已打开

2.FALSE，摄像头未打开

约束：

无

#### read

方法：read（）

说明：

根据cameracapture类对象初始化时的id，从该id表示的摄像头读取图片，并将图片保存为AclLiteImage结构的数据。

输入参数：

无

返回值：

AclLiteImage类型的图片数据

约束：

无

#### close

方法：close（）

说明：

关闭打开的摄像头

输入参数：

无

返回值：

无，正常执行会打印 "Close camera" 字段。

约束：

无

### AclLiteModel类

AclLiteModel类为python-acllite对acl模型推理相关接口的封装，包括但不限于模型加载与初始化，模型输入输出dataset的创建，模型推理执行及资源释放等功能。

#### __init__

方法：AclLiteModel(model_path, load_type)

说明：

模型推理类初始化

输入数据：

model_path：模型的路径。

load_type：加载模型的方式，可选0和1，默认值为0。（0：从文件加载离线模型数据；1：从内存加载离线模型数据）

返回值：

无

约束：

无

#### execute

方法：execute (input_list):

说明：

模型推理接口，将输入数据转变为acl dataset类型数据后送给模型做推理，推理结果以numpy array表示

输入参数：

input_list：模型输入数据，支持AclLiteImage、numpy array 和{'data': ,'size':} dict 结构数据。

返回值：

numpy array，用来表示模型推理结果。

约束：

无


### AclLiteImageProc类

AclLiteImageProc类为python-acllite对CANN媒体数据处理相关接口的封装，包括但不限于图片解码编码，视频解码编码，抠图缩放等功能。

#### jpegd

方法：jpegd(image):

说明：

图片解码接口，将jpeg格式图片转换为yuv格式

输入参数：

image：原始jpeg图片数据，以AclLiteImage结构存储的数据。

返回值：

AclLiteImage，用来存放yuv图片数据。

约束：

无

#### jpege

方法：jpege(image):

说明：

图片解码接口，将yuv格式图片转换为jpeg格式

输入参数：

image：原始yuv图片数据，以AclLiteImage结构存储的数据。

返回值：

AclLiteImage，用来存放jpeg图片数据。

约束：

无

#### crop_and_paste

方法：crop_and_paste(image, width, height, crop_and_paste_width, crop_and_paste_height)

说明：

图片VPC（vision preprocessing core）功能相接口，将原始图片扣出再贴到目标大小

输入参数：

image：原始图片数据，以AclLiteImage结构存储的数据。

width：原始图片宽。

height：原始图片高。

crop_and_paste_width：VPC后目标图片的宽

crop_and_paste_height：VPC后目标图片的高

返回值：

AclLiteImage，用来存放vpc后的图片数据。

约束：

无

#### resize

方法：resize(image, resize_width, resize_height)

说明：

将输入图片resize到指定大小。

输入参数：

image：原始图片数据，以AclLiteImage结构存储的数据。

resize_width：缩放后图片的宽。

resize_height：缩放后图片的高。

返回值：

AclLiteImage，用来存放resize后的图片数据。

约束：

无

### Dvpp_Vdec类

Dvpp_Vdec类为python-acllite对视频流的解码相关接口的封装。包括了对视频流的切帧等。

#### read

方法：read (no_wait):

说明：

视频帧读接口，异步接口，负责从队列中读取数据并送去解码。

输入参数：

no_wait：布尔变量，为真则不断从队列中读取数据，需要使用is_finished（）接口来判断该帧数据是否解码完毕；为否则会按照READ_TIMEOUT设置的时间间隔从队列里读取数据，为空则会报错；默认为否。

返回值：

ret ：接口执行结果；SUCCESS为正常；FAILED表示失败，有数据未解码，但是接口未从队列中读取到数据。

image ：读到的视频帧

约束：

视频流必须为以下格式之一：

h264 ：main, baselineor high level，且为 annex-b格式

h265 ：main level

#### process

方法：process (input_data, input_size, user_data)

说明：

视频解码接口，将需要解码的视频帧数据送给解码器做处理。

输入参数：

input_data ：输入数据。

input_size ：输入数据大小。

user_data ：python对象，用户自定义数据。如果用户需要获取解码的帧序号，则可以在user_data参数处定义，然后解码的帧序号可以通过user_data参数传递给VDEC的回调函数，用于确定回调函数中处理的是第几帧数据。

返回值：

ret ：接口执行结果；SUCCESS为正常；FAILED表示失败


约束：

无

