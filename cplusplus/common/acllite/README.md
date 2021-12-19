# AclLite库
## 使用说明

AclLite库是对当前开源社区样例中

1.Atlas200DK板载摄像头

2.acl dvpp图像和视频处理

3.acl设备资源、模型推理接口

等重复代码进行封装，为用户提供的一组简易公共接口。

注意：

1.本库仅供当前社区开源样例使用，不覆盖ascend平台应用开发的所有场景，不作为用户应用开发的标准库；

2.本库仅在Atlas200DK和Atlas300（x86）服务器上做了验证。

## 编译方法

#### 编译依赖

| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | CANN>=5.0.4 | 请到[CANN社区版](https://www.hiascend.com/software/cann/community)下载对应版本软件包 |
| 设备形态 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | presentagent, ffmpeg | 请参考[第三方依赖安装指导（C++样例）](../../environment)完成对应安装 |

#### 编译步骤

1. 获取源码包

   可以使用以下两种方式下载，请选择其中一种即可   
    - 命令行下载
       ```    
       # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```   
    - 压缩包下载   
       ``` 
        # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
        # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip
        ```

2.进入acllite目录
```
cd ${HOME}/samples/cplusplus/common/acllite
```
3.执行编译安装命令。
```
make 
make install
```
4.确认安装

  若安装正常完成，编译生成的libacllite.so将被拷贝到`${THIRDPART_PATH}/lib`路径下；头文件被拷贝到`${THIRDPART_PATH}/include/acllite`路径。  


## 部署方法

1. 昇腾AI设备安装开发环境，同时作为运行环境场景：

   不需要另外部署。

2. 非昇腾AI设备上安装开发环境场景：

    - 将libacllite.so拷贝到运行环境的`${THIRDPART_PATH}/lib`路径。
    - 在运行环境下切换到 root用户，打开`/etc/ld.so.conf.d/mind_so.conf` ，将`${THIRDPART_PATH}/lib`追加到文件末尾，保存后退出，执行命令ldconfig。

注：开发环境是指编译应用代码的环境；运行环境是指运行应用的昇腾AI设备；两者可以在同一个硬件设备上，也可以是分离的。

## 接口说明

### AclLiteCapture类

AclLiteCapture类为Atlas200DK板载摄像头、RTSP视频流、mp4文件和H264/H265裸流文件解码提供统一接口。

#### AclLiteVideoProc

方法：AclLiteVideoProc(uint32_t width = 1280, uint32_t height = 720, uint32_t fps = 15)

说明：在Atlas200DK上打开一个可用的摄像头。如果0槽位摄像头可用，则选择0槽位；否则选择槽位1;如果两个摄像头都不可用，只生成实例，不会打开任何摄像头。

输入参数：
    
width:摄像头分辨率宽  

height:摄像头分辨率高  

fps:帧率, 若使用的RASPBERRY PI V2.1型号的摄像头，摄像头支持设置的帧率范围为：[1-20]；若使用的RASPBERRY PI V1.3型号的摄像头，摄像头支持设置的帧率范围为：[1-15]。

返回值：无

约束：

1.  只支持Atlas200dk;  

2. 摄像头默认分辨率参数设置需要符合驱动要求，当前支持5种分辨率：1920x1080，1280x720，704x576，704x288，352x288。

#### AclLiteVideoProc

方法: AclLiteVideoProc(uint32_t cameraId, uint32_t width = 1280, uint32_t height = 720, uint32_t fps = 15)

说明: 在Atlas200DK上打开指定槽位的摄像头。如果该摄像头不可用，只生成实例，不会打开摄像头

输入参数:

cameraId：摄像头id，0 表示CAMERA0槽位的摄像头，1 表示CAMERA1槽位的摄像头

width:摄像头分辨率宽

height:摄像头分辨率高

fps:帧率, 若使用的RASPBERRY PI V2.1型号的摄像头，摄像头支持设置的帧率范围为：[1-20]；若使用的RASPBERRY PI V1.3型号的摄像头，摄像头支持设置的帧率范围为：[1-15]。

返回值：无

约束： 

1. 只支持Atlas200dk;  

2. 摄像头默认分辨率参数设置需要符合驱动要求，当前支持5种分辨率：1920x1080，1280x720，704x576，704x288，352x288。

#### AclLiteVideoProc

方法：AclLiteVideoProc(const string& videoPath, aclrtContext context = nullptr)

说明：解码视频videoPath

输入参数：

videoPath：视频文件或者rtsp地址；

context：解码器使用dvpp vdec解码时使用的acl context。默认情况下使用当前线程的context

返回值：无

约束：	解码器使用ffmpeg+vdec解码视频，在创建实例前需要初始化acl(aclInit)和设置device(aclrtSetDevice)

注意事项：无

#### IsOpened

方法: bool IsOpened()

说明: 判断摄像头或者视频流是否已经打开

输入参数: 无

返回值: 

true: 已经打开摄像头，或者可以解码视频流；

false: 摄像头不可用，或者视频流无法解码。

约束: 无

注意: 无

#### Get

方法: uint32_t Get(StreamProperty key)

说明: 获取摄像头或者视频解码属性

输入参数: key属性，定义如下

​                 enum StreamProperty {

​                          FRAME_WIDTH = 1, ///视频分辨率宽

​                          FRAME_HEIGHT = 2,//视频分辨率高

​                          VIDEO_FPS = 3,//频解码帧率
                          
                          OUTPUT_IMAGE_FORMAT = 4,//输出图片格式

                          RTSP_TRANSPORT = 5，//RTSP流传输协议（UDP/TCP）

​                 };

返回值: 属性值

约束: 只有视频流解码支持Get获取属性；

注意: 无

#### Set

方法: AclLiteError Set(StreamProperty key, uint32_t value)

说明: 设置解码属性

输入参数: 

1.key属性：

​                  enum StreamProperty {   

​                            ......

​                            OUTPUT_IMAGE_FORMAT = 4,//输出图像格式

​                            RTSP_TRANSPORT = 5 //rtsp传输方式
​                   };

​                 2.value 属性值。图像格式支持PIXEL_FORMAT_YUV_SEMIPLANAR_420和PIXEL_FORMAT_YVU_SEMIPLANAR_420；

​                 传输属性支持RTSP_TRANS_UDP(值为0)和RTSP_TRANS_TCP(值为1)

返回值: 是否成功

约束: 

1.只有视频流支持Set设置属性；

​2.Mp4文件和rtsp都支持设置输出图像格式；

3.只有RTSP流支持设置传输方式

注意: 无

#### Read

方法: AclLiteError Read(ImageData& frame)

说明: 读取摄像头或者视频流解码后的一帧视频图像

输入: Frame：视频图像数据和属性

返回值: ACLLITE_OK：读取成功

其他: 读取失败。当前解码异常或者已经解码完毕

约束说明: 获取的数据内存为DVPP内存，因为内存不能在不同的context间传递，所以创建解码器时传入的context和调用Read接口线程的context必须相同，否则图像数据不可用

注意事项: 果返回值不为ACLLITE_OK，表示当前解码失败建议不再读取

#### Close

方法: AclLiteError Close()

说明: 停止解码或者关闭摄像头

输入参数: 无

返回值: ACLLITE_OK：关闭/停止成功

其他: 关闭/停止失败

约束: 无

注意事项

### AclLiteImageProc类

#### AclLiteImageProc

方法: AclLiteImageProc()

说明: 图片处理实例，包括但不限于图片编码、图片解码、缩放、crop抠图等方法

输入参数: 无

返回值: 无

约束: 无

注意事项	

#### InitResource

方法: AclLiteError InitResource(aclrtStream& stream)

说明: AclLiteImageProc初始化

输入参数: Stream: AclLiteImageProc实例使用的acl stream

返回值: ACLLITE_OK:初始化成功

​             其他: 初始化失败

约束: 无

注意事项: 无	

#### Resize

方法: AclLiteError Resize(ImageData& dest, ImageData& src,  uint32_t width, uint32_t height)

说明: 将图片缩放到指定大小

输入参数: 


dest: resize后的图片数据，格式为Yuv420sp NV12, 图片数据存放在dvpp内存中

src: 待压缩图片，尺寸满足dvpp对齐要求

width: 缩放目标大小的宽度

height: 缩放目标大小的高度

返回值: 

ACLLITE_OK: 缩放成功

其他: 缩放失败

约束: 输入图片内存为dvpp, 并且满足16x2对齐

注意事项: acl dvpp在缩放图片时，输出图片是16x2对齐的，所以缩放结果图片不一定和接口参数一致，例如将图片缩放参数是300x300，得到的图片将是304x300

#### JpegD

方法: AclLiteError JpegD(ImageData& destYuv, ImageData& srcJpeg)

说明: 将jpeg图片解码为yuv420sp图片

输入参数: 

destYuv: 解码后的yuv图片

srcJpeg: 待解码的jpeg图片

返回值: 

ACLLITE_OK：解码成功

其他: 解码失败

约束: 输入图片内存为dvpp，并满足dvpp jpegd约束

注意: 输出yuv图片满足16x2对齐，不一定和输入图片尺寸一致

#### JpegE

方法: AclLiteError JpegE(ImageData& destJpeg, ImageData& srcYuv)

说明: 将yuv图片编码为jpeg图片

输入参数: 

destJpeg:生成的jpeg图片

SrcYuv:输入的yuv 图片

返回值: ACLLITE_OK：编码成功

其他: 编码失败

约束: 无

注意事项: 无	

### AclLiteModel类

#### AclLiteModel

方法: AclLiteModel(const string& modelPath)

说明: Acl model的封装

输入参数: modelPath 离线模型路径

返回值: 无

约束: 无

注意事项: 无	

#### Init方法

方法: AclLiteError Init()

说明: AclLiteModel初始化

输入参数: 无

返回值说明: ACLLITE_OK: 初始化成功

​                     其他: 初始化失败

约束: 无

注意事项: 无	

#### CreateInput

方法: AclLiteError CreateInput(void *input, uint32_t inputSize)

说明: 创建模型推理输入

输入参数: input: 推理数据

​                 inputSize: 数据大小

返回值: ACLLITE_OK：创建成功

​             其他: 创建失败

约束: 输入数据input在device或者dvpp内存中

注意事项: 无	

#### CreateInput

方法: AclLiteError CreateInput(void* input1, uint32_t input1Size,  void* input2, uint32_t input2Size)

说明: 创建模型推理输入

输入参数: input1: 第一个输入

​                 input1Size: 第一个输入数据大小

​                 input2: 第二个输入

​                 input2Size: 第二个输入大小

返回值: ACLLITE_OK: 创建成功

​             其他: 创建失败

约束: 要求输入数据在device或者dvpp内存中

注意事项: 无	

#### CreateInput

方法: AclLiteError CreateInput(vector<DataInfo\>& inputData)

说明: 创建模型推理输入。用于输入有多个的模型。

输入参数: inputData：输入数据列表

​                 struct DataInfo {

​                        void* data;

​                        uint32_t size;

​                 };

返回值: ACLLITE_OK: 创建成功

​             其他: 创建失败

约束: 要求数据在device或者dvpp内存中

注意事项: 无	

#### DestroyInput

方法: void DestroyInput()

说明: 销毁创建的模型输入

输入参数: 无

返回值: 无

约束: 无

注意事项: 只释放CreateInput创建的dataset结构，不会释放输入的数据	

#### Execute方法

方法: AclLiteError Execute(vector<InferenceOutput\>& inferOutputs);

说明: 执行模型推理

输入参数: inferOutputs 输出参数，推理结果：

​                 struct InferenceOutput {

​                         shared_ptr<void\> data = nullptr;

​                        uint32_t size;

​                 };

返回值: ACLLITE_OK：推理成功

​             其他: 推理失败

约束: 无	

注意事项: 无	

### 日志

#### ACLLITE_LOG_ERROR

方法: ACLLITE_LOG_ERROR(fmt, ...)

说明: 打印acl ERROR级别日志到/var/log/npu/slog/host-0/host-xxxx.log

输入参数: fmt：格式化字符串

返回值: 无

约束: 无

注意事项: 无	

#### ACLLITE_LOG_INFO

方法: ACLLITE_LOG_INFO(fmt, ...)

说明: 打印acl INFO级别日志到/var/log/npu/slog/host-0/host-xxxx.log

输入参数: log_info：打印的日志内容，支持格式化字符串,记录的日志包括函数、文件和行号

返回值: 无

约束: 无

注意事项: 无	

#### ACLLITE_LOG_DEBUG

方法: ACLLITE_LOG_DEBUG(fmt, ...)

说明: 打印acl DEBUG级别日志到/var/log/npu/slog/host-0/host-xxxx.log

输入参数: log_info：打印的日志内容，支持格式化字符串,记录的日志包括函数、文件和行号

返回值: 无

约束: 无

注意事项: 无	

### 其他接口

#### ReadConfig

方法: bool ReadConfig(map<string, string>& config,  const char* configFile)

说明: 解析配置文件

输入参数: config：解析结果

​                 configFile：配置文件路径

返回值: true:解析成功

​              false: 解析失败

约束: 类似如下的配置文件，要求配置项名称唯一：

​         [baseconf]

​         presenter_server_ip=192.168.1.166

注意事项: 无	

#### CopyDataToHost

方法: void* CopyDataToHost(void* data, uint32_t size, aclrtRunMode curRunMode, MemoryType memType)

说明: 数据拷贝到host侧

输入参数: data:待拷贝数据

​                 size: 数据大小

​                 curRunMode:当前的runMode

​                 memType：拷贝目的内存种类

​                 enum MemoryType {

​                            MEMORY_NORMAL = 0,//使用new申请的uint8_t类型内存

​                            MEMORY_HOST,//acl接口申请的host内存

​                            MEMORY_DEVICE,//acl接口申请的device内存

​                            MEMORY_DVPP, //dvpp内存   

​                            MEMORY_INVALID_TYPE

​                 };

返回值: 拷贝后的目的内存指针

约束: 不支持Atlas300服务器本地内存之间的拷贝

注意事项: 无	

#### CopyDataToDevice

方法: void* CopyDataToDevice(void* data, uint32_t size, aclrtRunMode curRunMode, MemoryType memType)

说明: 数据拷贝到device侧

输入参数: data:待拷贝数据

​                 size: 数据大小

​                 curRunMode:当前的runMode

​                 memType：拷贝目的内存种类

​                 enum MemoryType {

​                            MEMORY_NORMAL = 0,//使用new申请的uint8_t类型内存

​                            MEMORY_HOST,//acl接口申请的host内存

​                            MEMORY_DEVICE,//acl接口申请的device内存

​                            MEMORY_DVPP, //dvpp内存   

​                            MEMORY_INVALID_TYPE

​                 };

返回值: 拷贝后的目的内存指针

约束: 无 	

注意事项: 无 	

#### SaveBinFile

方法: void SaveBinFile(const char* filename, void* data, uint32_t size)

说明: 将数据保存二进制文件

输入参数: filename:带路径的二进制文件名

​                 Data:二进制数据

​                 size:数据大小

返回值: 拷贝后的目的内存指针

约束: 无

注意事项: 无	

#### ReadBinFile

方法: AclLiteError ReadBinFile(const char* filename, void*& data, uint32_t& size)

说明: 将数据保存二进制文件

输入: filename:带路径的二进制文件名

​          Data:读取的二进制数据

​          size:数据大小

返回值: 拷贝后的目的内存指针

约束: 无

注意事项: 无	