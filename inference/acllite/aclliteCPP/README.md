# AclLite库
## 使用说明

AclLite库是对当前开源社区样例中

1.Atlas200DK板载摄像头

2.acl dvpp图像和视频处理

3.acl设备资源、模型推理接口

等重复代码进行封装，为用户提供的一组简易公共接口。

注意：

1.本库仅供当前社区开源样例使用，不覆盖ascend平台应用开发的所有场景，不作为用户应用开发的标准库；

2.本库仅在Atlas200DK和Atlas300（x86）服务器上做了验证；

3.该公共库仅作为一个参考，用户可以根据自身使用习惯和实际业务对性能的要求，根据官方接口文档和现有代码进行二次开发或重构。

## 编译方法

#### 编译依赖

| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | CANN>=5.0.4 | 请到[CANN社区版](https://www.hiascend.com/software/cann/community)下载对应版本软件包 |
| 设备形态 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | presentagent, ffmpeg | 请参考[第三方依赖安装指导（C++样例）](../../../docs/INSTALL_cn.md)完成对应安装 |

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
cd ${HOME}/samples/inference/acllite/aclliteCPP
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

## 接口设计与说明

AclLite公共库主要根据面向对象原则设计，按处理对象主要分为以下功能模块：
   
   1. 资源管理模块，负责对acl推理资源的初始化与释放；

   2. 模型推理模块，负责对模型资源的初始化，输入输出创建，推理调用，资源释放；

   3. 图片处理模块，负责对图片类型数据的处理；

   4. 视频处理模块，负责对视频类型数据的处理；

   5. 应用及多线程模块，对编码一个简单多线程样例，提供了一套单例模式的应用-线程管理类-线程类接口，简化多线程样例开发；

   6. 其他文件，主要包括AclLite库中文件的工具函数文件、封装的结构体文件及错误码文件等等。

以下将按模块介绍接口功能及约束,当前仅对用户直接调用接口进行介绍。

### 1.资源管理模块

|  说明项  | 具体描述 |
|---|---|
| 函数 | AclLiteResource() |
| 功能 | 构造函数，创建一个AclLiteResource对象 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteResource(int32_t devId, const std::string& aclConfigPath, bool useDefaultCtx = true) |
| 功能 | 构造函数，创建一个AclLiteResource对象，并指定device，config配置文件和线程使用context | 
| 参数 | devId：设备id<br>aclConfigPath：config文件路径<br>useDefaultCtx：是否使用当前线程context |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | ~AclLiteResource() |
| 功能 | 析构函数，销毁AclLiteResource对象 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Init() |
| 功能 | 初始化函数，初始化Acl相关资源device、context | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK: 初始化成功<br>其他: 初始化失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void Release() |
| 功能 | 资源释放函数，释放Acl相关资源device、context | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | aclrtRunMode GetRunMode() |
| 功能 | 获取设备run mode | 
| 参数 | 无 |
| 返回值 | ACL_DEVICE：昇腾AI软件栈运行在Device的Control CPU或板端环境<br> ACL_HOST：昇腾AI软件栈运行在Host侧 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | aclrtContext GetContext() |
| 功能 | 获取程序运行的context | 
| 参数 | 无 |
| 返回值 | Context：<br>nullptr：无效context<br>非nullptr：有效context |

### 2.模型推理模块
该模块接口原型定义在AclLiteModel.h文件中，主要负责acl推理资源的初始化与释放。

#### 2.1 AclLiteModel类

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteModel() |
| 功能 | 构造函数，创建一个AclLiteModel类对象 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteModel(const std::string& modelPath) |
| 功能 | 构造函数，创建一个AclLiteModel类对象，并提供待加载模型文件路径 | 
| 参数 | modelPath：离线模型路径 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteModel(void *modelAddr, size_t modelSize) |
| 功能 | 构造函数，创建一个AclLiteModel类对象，并提供待加载模型文件内存地址和内存大小 | 
| 参数 | modelAddr：离线模型文件内存地址<br>modelSize：离线模型文件内存大小 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | ~AclLiteModel() |
| 功能 | 析构函数，销毁一个AclLiteModel类对象 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Init() |
| 功能 | 初始化函数，需要用户提供好模型文件路径/模型文件内存地址 | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK：初始化成功 <br>其他：初始化失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Init(const std::string& modelPath) |
| 功能 | 初始化函数，提供待加载模型文件路径后再调用Init() | 
| 参数 | modelPath：离线模型路径 |
| 返回值 | ACLLITE_OK：初始化成功 <br>其他：初始化失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Init(void *modelAddr, size_t modelSize) |
| 功能 |初始化函数，提供待加载模型文件内存地址和内存大小 | 
| 参数 | modelAddr：离线模型文件内存地址<br>modelSize：离线模型文件内存大小 |
| 返回值 | ACLLITE_OK：初始化成功 <br>其他：初始化失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void DestroyResource() |
| 功能 | 资源释放函数，将AclLiteModel类对象相关资源，如模型输入输出、desc等数据成员释放置空 | 
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | isReleased_，防止资源多次释放报错而设置的标志位 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CreateInput(void *input, uint32_t inputsize) |
| 功能 | 创建模型输入(场景：有一个输入的模型) | 
| 参数 | input：模型输入数据<br>input1size：模型输入数据大小 |
| 约束 | 要求数据在device或者dvpp内存中 |
| 返回值 | ACLLITE_OK：创建成功 <br>其他：创建失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CreateInput(void *input1, uint32_t input1size, void* input2, uint32_t input2size) |
| 功能 | 创建模型输入(场景：有两个输入的模型) | 
| 参数 | input1：模型第一个输入数据<br>input1size：模型第一个输入数据大小<br>input2：模型第二个输入数据<br>input2size：模型第二个输入数据大小 |
| 约束 | 要求数据在device或者dvpp内存中 |
| 返回值 | ACLLITE_OK：创建成功 <br>其他：创建失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CreateInput(std::vector<DataInfo>& inputData) |
| 功能 | 创建模型输入(场景：有多个输入的模型) | 
| 参数 | inputData：模型输入数据vector |
| 返回值 | ACLLITE_OK：创建成功 <br>其他：创建失败 |
| 约束 | 要求数据在device或者dvpp内存中 |
| 备注 | DataInfo数据结构详见 [**DataInfo**](#DataInfo)|

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs, void *data, uint32_t size, uint32_t batchsize = 0) |
| 功能 | 执行模型推理，该接口针对模型只有一个输入的场景，会先用第二、三个参数构建模型输入，再送去推理，支持动态batch特性，默认关闭 | 
| 参数 | inferOutputs：模型推理结果<br>data：模型输入数据<br>size：模型输入数据大小<br>batchsize：动态batch模型单次推理batch数 |
| 返回值 | ACLLITE_OK：推理成功 <br>其他：推理失败 |
| 备注 | 推理后结果数据存储在本地，如果是模型串接或再使用dvpp功能场景需要进行拷贝<br>InferenceOutput数据结构详见[**InferenceOutput**](#InferenceOutput) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs) |
| 功能 | 执行模型推理 | 
| 参数 | inferOutputs：模型推理结果 |
| 返回值 | ACLLITE_OK：推理成功 <br>其他：推理失败 |
| 备注 | 推理后结果数据存储在本地，如果是模型串接或再使用dvpp功能场景需要进行拷贝<br>InferenceOutput数据结构详见[**InferenceOutput**](#InferenceOutput) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | size_t GetModelInputSize(int index) |
| 功能 | 获取模型输入数据大小 | 
| 参数 | index：索引，标记是模型的第几个输入，index从0开始 |
| 返回值 | 模型输入数据大小 |
| 备注 | size单位为Byte |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void GetModelOutputInfo(vector<ModelOutputInfo>& modelOutputInfo) |
| 功能 | 获取模型输出数据信息 | 
| 参数 | modelOutputInfo：存储模型的输出节点信息 |
| 返回值 | ACLLITE_OK：获取成功 <br> ACLLITE_ERROR：获取失败 |
| 备注 | ModelOutputInfo数据结构详见 [**ModelOutputInfo**](#ModelOutputInfo) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void DestroyInput() |
| 功能 | 销毁模型输入 | 
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | 只释放CreateInput()创建的dataset结构，不会释放输入的数据 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError LoadModelFromFile(const std::string& modelPath) |
| 功能 | 模型加载函数，从文件加载模型 | 
| 参数 | modelPath：离线模型路径 |
| 返回值 | ACLLITE_OK：加载成功 <br>其他：加载失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError LoadModelFromMem() |
| 功能 | 模型加载函数，从内存加载模型 | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK：加载成功 <br>其他：加载失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError SetDesc() |
| 功能 | 创建并设置模型desc | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK：设置成功 <br>其他：设置失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CreateOutput() |
| 功能 | 创建模型输出 | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK：创建成功 <br>其他：创建失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError AddDatasetBuffer(aclmdlDataset* dataset, void* buffer, uint32_t bufferSize) |
| 功能 | 创建databuffer并添加到dataset | 
| 参数 | dataset：待添加databuffer的dataset<br>buffer：创建databuffer的数据 <br>bufferSize：数据大小 |
| 返回值 | ACLLITE_OK：添加成功 <br>其他：添加失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError GetOutputItem(InferenceOutput& out, uint32_t idx) |
| 功能 | 按索引将推理结果取出并拷贝至本地 | 
| 参数 | out：拷贝到本地的推理结果<br>idx：索引 |
| 返回值 | ACLLITE_OK：拷贝成功 <br>其他：拷贝失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void Unload() |
| 功能 | 模型卸载函数，卸载已加载模型；如果模型是存放在内存中，则相应内存也一并释放 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void DestroyDesc() |
| 功能 | 销毁模型desc | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void DestroyOutput() |
| 功能 | 销毁模型输出 | 
| 参数 | 无 |
| 返回值 | 无 |

### 3.图片处理模块

#### 3.1 AclLiteImageProc类
该模块接口原型定义在AclLiteImageProc.h文件中，主要负责图片类型数据的处理。

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteImageProc() |
| 功能 | 构造函数，创建一个AclLiteImageProc对象，便于对图片数据做处理 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | ~AclLiteImageProc() |
| 功能 | 析构函数，销毁AclLiteImageProc对象 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Init() |
| 功能 | AclLiteImageProc对象初始化，指定后续使用dvpp功能所需的stream和channel | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Resize(ImageData& dest,ImageData& src, uint32_t width, uint32_t height) |
| 功能 | 将图片缩放到指定大小 | 
| 参数 | dest: 压缩后的图片<br>src: 待压缩图片<br>width: 缩放目标大小的宽度<br>height: 缩放目标大小的高度 |
| 返回值 | ACLLITE_OK: 缩放成功<br>其他: 缩放失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[VPC约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0159.html)”请注意选择配套的CANN版本<br> |
| 备注 | acllite resize()在内部封装了对齐操作，使用的对齐参数为16x2，而对齐可能会使输出的缩放图片size与接口参数不一致，请注意<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError PngD(ImageData& destRgb, ImageData& srcPng) |
| 功能 | 将png图片解码为rgb图片 | 
| 参数 | destRgb：解码后的rgb图片<br>srcPng：待解码的png图片 |
| 返回值 | ACLLITE_OK: 解码成功<br>其他: 解码失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[pngD功能及约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0189.html)”请注意选择配套的CANN版本 |
| 备注 | acllite PngD()在内部封装了对齐操作，当前使用的对齐参数为128x16，而对齐可能会使输出的解码图片size与原始图片宽高不一致，请注意<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError JpegD(ImageData& destYuv, ImageData& srcJpeg) |
| 功能 | 将jpeg图片解码为yuv图片 | 
| 参数 | destYuv：解码后的yuv图片<br>srcJpeg：待解码的jpeg图片 |
| 返回值 | ACLLITE_OK: 解码成功<br>其他: 解码失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[JPEGD功能及约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0177.html)”请注意选择配套的CANN版本 |
| 备注 | acllite JpegD()在内部封装了对齐操作，当前使用的对齐参数为128x16，而对齐可能会使输出的解码图片size与原始图片宽高不一致，请注意<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError JpegE(ImageData& destJpeg, ImageData& srcYuv) |
| 功能 | 将yuv图片编码为jpeg图片 | 
| 参数 | destJpeg：编码后的jpeg图片<br>srcYuv：待编码的yuv图片 |
| 返回值 | ACLLITE_OK: 编码成功<br>其他: 编码失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[JPEGE功能及约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0184.html)”请注意选择配套的CANN版本 |
| 备注 | acllite JpegE()在内部封装了对齐操作，当前使用的对齐参数为16x2，而对齐可能会使输出的编码图片size与原始图片宽高不一致，请注意，请注意<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Crop(ImageData& dest, ImageData& src, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert) |
| 功能 | 抠图贴图，从原图抠出(ltHorz, ltVert)、(rbHorz, rbVert)两点确定的矩形区域,并贴至贴图区域(0, 0)(rbHorz-ltHorz, ltVert-rbVert) | 
| 参数 | dest：抠图贴图后图片数据<br>src：待处理图片数据<br>ltHorz：左上点的X坐标<br>ltVert：左上点的Y坐标<br>rbHorz：右下点的X坐标<br>rbVert：右下点的Y坐标 |
| 返回值 | ACLLITE_OK: 处理成功<br>其他: 处理失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[VPC约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0159.html)”请注意选择配套的CANN版本<br> |
| 备注 | acllite Crop()在内部封装了对齐操作，会对传入图片的宽高及坐标偏移值做自动化处理，满足VPC功能方面约束<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CropPaste(ImageData& dest, ImageData& src, uint32_t width, uint32_t height, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert) |
| 功能 | 抠图贴图，从原图抠出(ltHorz, ltVert)、(rbHorz, rbVert)两点确定的矩形区域，并贴至贴图区域(0, 0)(width, height) | 
| 参数 | dest：抠图贴图后图片数据<br>src：待处理图片数据<br>width：贴图后图片宽<br>height:贴图后图片高<br>ltHorz：确定抠图区域的左上点的X坐标<br>ltVert：确定抠图区域的左上点的Y坐标<br>rbHorz：确定抠图区域的右下点的X坐标<br>rbVert：确定抠图区域的右下点的Y坐标 |
| 返回值 | ACLLITE_OK: 处理成功<br>其他: 处理失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[VPC约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0159.html)<br> |
| 备注 | 该接口处理完的图片数据，会对传入图片的宽高及坐标偏移值做自动化处理，满足VPC功能方面约束<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError ProportionPaste(ImageData& dest, ImageData& src, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert); |
| 功能 | 等比例贴图, 将原始图片数据在不改变宽高比的情况下，贴至区域(0, 0)(rbHorz-ltHorz, ltVert-rbVert)| 
| 参数 | dest：抠图贴图后图片数据<br>src：待处理图片数据<br>ltHorz：左上点的X坐标<br>ltVert：左上点的Y坐标<br>rbHorz：右下点的X坐标<br>rbVert：右下点的Y坐标 |
| 返回值 | ACLLITE_OK: 处理成功<br>其他: 处理失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[VPC约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0159.html)<br> |
| 备注 | 该接口处理完的图片数据，会对传入图片的宽高及坐标偏移值做自动化处理，满足VPC功能方面约束，因此可能会有绿边产生，介意者需要使用AIPP功能消除<br>由于是等比例贴图，操作后贴图区域的空白区域会被绿边填充，如原始图片区域为(0,0)(200,100)，贴图区域为(0,0)(50,50)，则(0，0)(50，25)区域为有效图片数据，(0，25)(50，50)会被绿边填充(该例中dvpp对齐约束先忽略不计，以左上角为原点，x轴往右y轴往下为正方向)<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError ProportionPasteCenter(ImageData& dest, ImageData& src, uint32_t width, uint32_t height); |
| 功能 | 中心等比例贴图, 将原始图片数据在不改变宽高比的情况下，贴至贴图区域(width,height)中央| 
| 参数 | dest：抠图贴图后图片数据<br>src：待处理图片数据<br>width：贴图区域图片宽<br>height：贴图区域图片高 |
| 返回值 | ACLLITE_OK: 处理成功<br>其他: 处理失败 |
| 约束 | 该接口对数据输入输出的相关约束请参考“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中的“[VPC约束说明](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0159.html)<br> |
| 备注 | 该接口处理完的图片数据，会对传入图片的宽高及坐标偏移值做自动化处理，满足VPC功能方面约束，因此可能会有绿边产生，介意者需要使用AIPP功能消除<br>由于是等比例贴图，操作后贴图区域的空白区域会被绿边填充<br>ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void DestroyResource() |
| 功能 | 销毁AclLiteImageProc类对象的相关资源 | 
| 参数 | 无 |
| 返回值 | 无 |

### 4.视频处理模块

该模块接口负责对视频类型数据的处理。

#### 4.1 AclLiteVideoProc类

该类为Atlas200DK板载摄像头、RTSP视频流、mp4文件和H264/H265裸流文件解码及yuv图片编码，该类的原型定义在AclLiteVideoProc.h文件中。

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteVideoProc() |
| 功能 | 构造函数，生成摄像头实例；如果0槽位摄像头可用，则选择0槽位；否则选择槽位1;如果两个摄像头都不可用，只生成实例，不会打开任何摄像头 | 
| 参数 | 无，但是正常打开后，会指定分辨率参数：宽为1280，高为720，帧率为15 |
| 返回值 | 无 |
| 约束 | 1. 只支持Atlas200dk设备形态<br>2. 若使用的RASPBERRY PI V2.1型号的摄像头，摄像头支持设置的帧率范围为：[1-20]<br>3. 若使用的RASPBERRY PI V1.3型号的摄像头，摄像头支持设置的帧率范围为：[1-15] <br>4. 摄像头默认分辨率参数设置需要符合驱动要求，当前支持5种分辨率：1920x1080，1280x720，704x576，704x288，352x288 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteVideoProc(uint32_t cameraId, uint32_t width = 1280, uint32_t height = 720, uint32_t fps = 15) |
| 功能 | 构造函数，生成摄像头实例；如果该摄像头不可用，只生成实例，不会打开摄像头 | 
| 参数 | cameraId：摄像头id，0 表示CAMERA0槽位的摄像头，1 表示CAMERA1槽位的摄像头<br>width:摄像头分辨率宽<br>height:摄像头分辨率高<br>fps:摄像头帧率 |
| 返回值 | 无 |
| 约束 | 1. 只支持Atlas200dk<br>2. 若使用的RASPBERRY PI V2.1型号的摄像头，摄像头支持设置的帧率范围为：[1-20]<br>3. 若使用的RASPBERRY PI V1.3型号的摄像头，摄像头支持设置的帧率范围为：[1-15] <br>4. 摄像头默认分辨率参数设置需要符合驱动要求，当前支持5种分辨率：1920x1080，1280x720，704x576，704x288，352x288|

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteVideoProc(const string& videoPath, aclrtContext context) |
| 功能 | 构造函数，创建等待解码的视频/rtsp流实例 | 
| 参数 | videoPath：等待解码的视频文件地址/rtsp流地址<br>context：解码器使用dvpp vdec功能解码时使用的acl context；可不填，则传入参数视为nullptr，并使用当前线程的context做解码 |
| 返回值 | 无 |
| 约束 | 1. 请参见“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中“应用开发”手册的[VDEC功能及约束](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0195.html)，请注意选择对应的CANN版本。<br>2. 在创建实例前需要初始化acl(aclInit)和设置device(aclrtSetDevice) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteVideoProc(VencConfig& vencConfig, aclrtContext context) |
| 功能 |  构造函数，创建等待编码的视频实例 | 
| 参数 | vencConfig：编码配置文件，VencConfig类型结构体，结构体详见于[**VencConfig**](#VencConfig)<br>context：解码器使用dvpp venc功能编码时使用的acl context；可不填，则传入参数视为nullptr，并使用当前线程的context做解码 |
| 返回值 | 无 |
| 约束 | 1. 请参见“[昇腾社区文档中心](https://www.hiascend.com/document?tag=community-developer)”中“应用开发”手册的[VENC功能及约束](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/60RC1alpha02/infacldevg/aclcppdevg/aclcppdevg_03_0201.html)，请注意选择对应的CANN版本。<br>2. 在创建实例前需要初始化acl(aclInit)和设置device(aclrtSetDevice) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | ~AclLiteVideoProc() |
| 功能 |  析构函数 | 
| 参数 | 无 |
| 返回值 | 无 |
| 约束 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Open() |
| 功能 | 打开摄像头/视频流 | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK：打开成功<br>非ACLLITE_OK：打开失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool IsOpened() |
| 功能 | 判断摄像头或者视频流是否已经打开 | 
| 参数 | 无 |
| 返回值 | true: 已经打开摄像头，或者可以解码视频流<br>false: 摄像头不可用，或者视频流无法解码 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | uint32_t Get(StreamProperty key) |
| 功能 | 根据key值获取对应属性实际值 | 
| 参数 | key:枚举类型StreamProperty，详见[**AclLiteVideoCapBase类**](#AclLiteVideoCapBase类) |
| 返回值 | 属性值 |
| 约束 | 需要根据各构造函数实际生成对象的get()，判断是否支持key值对应属性的获取 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Set(StreamProperty key, uint32_t value) |
| 功能 | 根据key值设置对应属性实际值 | 
| 参数 | key：枚举类型StreamProperty，详见[**AclLiteVideoCapBase类**](#AclLiteVideoCapBase类)<br> value：属性值 |
| 返回值 | ACLLITE_OK：设置成功<br>非ACLLITE_OK：设置失败|
| 约束 | 需要根据各构造函数实际生成对象的get()，判断是否支持key值对应属性的设置 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Read(ImageData& frame) |
| 功能 | 获取要被处理的一帧图像数据；<br>根据构造函数，目前有以下三种场景：<br>1.从摄像头读取的一帧数据；<br>2.从视频文件/rtsp流读入的一帧数据；<br>3.被dvpp venc处理的一帧数据 | 
| 参数 | frame：输入的图像数据和属性，ImageData结构数据，该结构详见[**ImageData**](#ImageData) |
| 返回值 | ACLLITE_OK：读入成功<br>非ACLLITE_OK：读入失败 |
| 约束 | 获取的数据内存为dvpp内存，因为该内存不能在不同的context间传递，所以创建解码器时传入的context和调用Read接口线程的context必须相同，否则图像数据不可用 |
| 备注 | 注意在预处理后处理时，是否存在数据要在dvpp、device与host之间拷贝的场景 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Close() |
| 功能 | 停止读入图片数据 | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK：关闭/停止成功<br>非ACLLITE_OK：关闭/停止失败|

<a name="AclLiteVideoCapBase类"></a>
#### 4.2 AclLiteVideoCapBase类

该类的原型定义在AclLiteVideoCapBase.h文件中；公共库通过对该类做继承，派生出CameraCapture、VideoCapture、VideoWriter这三个类并对其应有功能进行一定规定，从而使这三个类能够被AclLiteVideoProc整合调用，并提供给用户使用。
CameraCapture类：实现从摄像头读取图片帧；
VideoCapture类：实现从视频读取图片帧；
VideoWriter类：实现将一个个图片帧写成视频文件。

用户可以通过继承该类，进行二次开发，从而根据自身实际业务场景，完成对视频处理模块的功能定制与增加；甚至公共库更起到抛砖引玉的作用——用户可以参考AclLite这种思路，根据自身使用习惯和需要的功能自行对AscendCL接口进行梳理和封装，从而方便使用。

| 说明项 | 具体描述 |
|---|---|
| 枚举类型 | StreamProperty |
| 功能 | 标记可设置的属性 | 
| 参数 |  FRAME_WIDTH = 1  // 图像宽<br>FRAME_HEIGHT = 2  // 图像高<br>VIDEO_FPS = 3 // 解码帧率<br>OUTPUT_IMAGE_FORMAT = 4 // 输出图片格式<br>RTSP_TRANSPORT = 5 // RTSP流传输协议（UDP/TCP）<br>STREAM_FORMAT = 6 // 流格式（H265 MP / H264 BP/MP/HP）|

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteVideoCapBase() |
| 功能 | 构造函数，无实现，用户必须在继承类中做实现 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual ~AclLiteVideoCapBase() |
| 功能 | 析构函数，无实现，用户必须在继承类中做实现 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual bool IsOpened() |
| 功能 | 纯虚函数，无实现，用户必须在继承类中做实现，该函数旨在确认视频/摄像头等数据来源状态是否正常 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual AclLiteError Set(StreamProperty key, uint32_t value) |
| 功能 | 纯虚函数，无实现，用户必须在继承类中做实现，该函数旨在为用户提供处理数据时，设置特定属性的值的接口 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual uint32_t Get(StreamProperty key) |
| 功能 | 纯虚函数，无实现，用户必须在继承类中做实现，该函数旨在为用户提供处理数据时，获取特定属性的值的接口 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual AclLiteError Read(ImageData& frame) |
| 功能 | 纯虚函数，无实现，用户必须在继承类中做实现，该函数旨在为用户提供数据读入动作的接口 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual AclLiteError Close() |
| 功能 | 纯虚函数，无实现，用户必须在继承类中做实现。该函数旨在实现视频/摄像头等数据来源的相关资源的正确释放 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual AclLiteError Open() |
| 功能 | 纯虚函数，无实现，用户必须在继承类中做实现。该函数旨在实现视频/摄像头等数据来源初始化完全，等待读入 | 
| 参数 | 无 |
| 返回值 | 无 |

### 5. 应用及多线程模块

#### 5.1 AclLiteApp类

AclLiteApp类为应用管理类，用于创建和管理应用的线程。本类是一个单实例模式的类。该类的原型定义在AclLiteApp.h文件中。

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteApp()<br>AclLiteApp(const AclLiteApp&) = delete<br>AclLiteApp& operator=(const AclLiteApp&) = delete; |
| 功能 | 构造函数 | 
| 参数 | 无|
| 返回值 | 无 |
| 备注 | 由于AclLiteApp类是一个单实例模式的类，所以应用中创建一个实例时，使用GetInstance方法获取类的唯一实例，而不是直接使用构造函数直接创建实例。并且实例禁止拷贝 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | ~AclLiteApp() |
| 功能 | 析构函数，释放应用所有线程 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | static AclLiteApp& GetInstance() |
| 功能 | 获取全局唯一的AclLiteApp实例 | 
| 参数 | 无 |
| 返回值 | AclLiteApp实例 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError Init() |
| 功能 | AclLiteApp实例初始化，并创建第一个线程，线程号为0。这个线程用于处理整个应用的公共事务，例如和其他应用进程的通信、本应用的结束等 | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK ：初始化成功<br>非ACLLITE_OK ：初始化失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | int CreateAclLiteThread(AclLiteThread* thInst, const std::string& instName,aclrtContext context, aclrtRunMode runMode) |
| 功能 | 创建一个AclLite线程，并执行用户提供的业务对象实例Init方法，然后循环等待业务消息。线程为detach，非阻塞接口，所以在调用本接口后，必须有循环等待接口来等待线程结束 | 
| 参数 | thInst: 应用的业务线程实例，需要用户在该线程中重载Init方法，实现线程的初始化；重载Process方法，处理接收到的消息<br>instName: 线程名称。需要使用全局唯一的名称，否则创建失败。用户可以调用GetAclLiteThreadIdByName接口，以线程名为参数，获取线程ID<br>context：线程运行的acl context<br>runMode：应用的runmode，可取值：ACL_DEVICE（昇腾AI软件栈运行在Device的Control CPU或板端环境）/ ACL_HOST（昇腾AI软件栈运行在Host侧） |
| 返回值 | 线程id<br>-1 ：创建失败<br>>0 ：创建成功 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | int CreateAclLiteThreadMgr(AclLiteThread* thInst, const string& instName, aclrtContext context, aclrtRunMode runMode) |
| 功能 | 本方法创建AclLite线程对应的管理结构，并实现以下功能：<br>1.检查用户线程名是否全局唯一<br>2.设置用户线程的context、run mode、name和id，这样在用户线程中可以使用对应的接口访问这些信息<br>3.将线程管理对象加入AclLiteApp线程管理表| 
| 参数 | thInst: 应用的业务线程实例，需要用户在该线程中重载Init方法，实现线程的初始化；重载Process方法，处理接收到的消息<br>instName: 线程名称。需要使用全局唯一的名称，否则创建失败。用户可以调用GetAclLiteThreadIdByName接口，以线程名为参数，获取线程ID<br>context：线程运行的acl context<br>runMode：应用的runmode，可取值：ACL_DEVICE（昇腾AI软件栈运行在Device的Control CPU或板端环境）/ ACL_HOST（昇腾AI软件栈运行在Host侧） |
| 返回值 | 线程id<br>-1 ：创建失败<br>>0 ：创建成功 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool CheckThreadNameUnique(const std::string& threadName) |
| 功能 | 检查线程名是称否全局唯一 | 
| 参数 | threadName：线程名称 |
| 返回值 | true：线程名称唯一<br>false：线程名称不唯一 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool CheckThreadAbnormal() |
| 功能 | 检查是否有状态为不正常的线程存在 | 
| 参数 | 无 |
| 返回值 | true：存在状态不正常线程<br>false：不存在状态不正常线程 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | int GetAclLiteThreadIdByName(const std::string& threadName) |
| 功能 | 根据线程名称获取线程id | 
| 参数 | threadName：线程名称 |
| 返回值 | 线程id<br>-1 ：无效id<br>>0 ：线程名称对应id |

| 说明项 | 具体描述 |
|---|---|
| 函数 | int Start(vector<AclLiteThreadParam>& threadParamTbl) |
| 功能 | 拉起线程表中的所有线程 | 
| 参数 | threadParamTbl：线程表 |
| 返回值 | 0：启动成功<br>非0：创建失败 |
| 备注 | 1.当前，有两种创建线程的方式：<br>(1)使用CreateAclLiteThread接口，传入对应参数，拉起一个线程；<br>(2)构造一个AclLiteThreadParam表，然后传入本方法，一次拉起所有线程.<br>2.线程创建需要保证时序。例如创建了一个上游线程，需要给下游线程发消息时，需要知道下游线程的id；但是此时若下游线程还未创建，则没有分配线程id，因此消息无法发出。所以上游线程必须在下游线程创建成功后，才能获取下游线程的id。<br>3.AclLiteThreadParam结构体内容详见[**AclLiteThreadParam**]() |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void Wait(); |
| 功能 | 阻塞0号线程即主线程；当在main流程中使用CreateAclLiteThread或者start方法创建了业务线程，并且通知各线程工作后，各个线程是detach的，且不会阻塞主线程。这个时候如果主线程执行完所有函数，整个应用将直接退出，各个线程也就无法完成自己的工作。所以在主线程中需要一个阻塞点，保持主线程不会结束，整个应用一直工作 | 
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | 本方法使用sleep死循环等待，如需结束应用，必须使用ctrl+c或者kill应用进程的方法 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void Wait(AclLiteMsgProcess msgProcess, void* param) |
| 功能 | AclLiteApp创建时，会拉起一个0号线程，即主线程。该线程有消息队列，由于该线程处于忙等状态，并没有使用该消息队列接收和处理消息；用户可以定义一个消息处理函数，然后调用本wait方法，在本方法中将会轮询0号线程的消息队列，收到消息后，调用msgProcess处理。如果处理返回失败（非0）值，本函数会退出轮询（死循环） | 
| 参数 | msgProcess:用户定义的消息处理函数入口<br> param：传递给消息处理函数的参数 |
| 返回值 | 无 |
| 备注 | AclLiteMsgProcess为需要用户自行实现的主线程消息处理接口，一般旨在判断收到什么信号时，应用开始退出并释放资源。<br>当前在AclLiteApp.h中，仅定义了该类型函数指针，供用户参考：<br>typedef int (\*AclLiteMsgProcess)(uint32_t msgId, shared_ptr<void> msgData, void* userData); |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void WaitEnd() |
| 功能 | 将waitEnd_置为true，Wait接口break退出等待 | 
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | waitEnd_表示等待终止<br>AclLiteApp创建时，会拉起一个0号线程，即主线程，该线程会处于一个忙等状态，且不断查询waitEnd_的值。<br>当用户需要结束整个app时，可以通过调用该接口，waitEnd_被置为true，当主线程查询到waitEnd_的值为true时，将会给各线程发出退出信号，并释放资源结束应用。 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError SendMessage(int dest, int msgId, shared_ptr<void> data) |
| 功能 | 向目标线程发送消息 | 
| 参数 | dest：目的线程id，是线程对应的线程管理对象，在AtlasApp的线程管理表threadList_中的下标<br>msgId：消息ID,用户在自己的应用中自行定义<br>data：消息数据，数据类型由用户定义 |
| 返回值 | ACLLITE_OK：发送成功<br>非ACLLITE_OK：发送失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void Exit() |
| 功能 | 通知所有线程结束，在线程都结束后，结束应用 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void ReleaseThreads() |
| 功能 | 将线程表中线程状态修改为退出，并释放所有线程资源 | 
| 参数 | 无 |
| 返回值 | 无 |

#### 5.2 AclLiteThread类

AclLiteThread类为抽象类，用户线程对象基类，提供线程对象的公共方法和属性。在使用AclLite库创建线程时，必须以本基类为父类进行重载。本基类提供两个纯虚函数Init()和Process()，用户线程对象必须实现这两个函数。该类的原型定义在AclLiteThread.h文件中。

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteThread() |
| 功能 | 构造函数，创建一个AclLiteThread类对象 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | ~AclLiteThread() |
| 功能 | 析构函数，销毁一个AclLiteThread类对象 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | virtual int Init() |
| 功能 | 纯虚函数，需要用户继承AclLiteThread类，定义自己的业务线程对象后，做重载实现该函数，完成自己线程需要的初始化。如果不重载，默认返回ACLLITE_OK | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|------|---|
| 函数 | virtual int Process(int msgId, shared_ptr<void> msgData) |
| 功能 | 纯虚函数，继承AclLiteThread的线程对象必须对Process()进行重载，以处理收到的消息。当其他线程给本线程发送消息后，本线程的线程管理对象在AclLiteThreadMgr::ThreadEntry的消息轮询就会获取到消息，即收到消息，然后调用本方法，把消息id和消息数据传递本方法处理。用户不感知AclLiteThreadMgr::ThreadEntry方法，**不用关注消息的接收以及Process的调用**，只需要关注Process本身的实现即可 | 
| 参数 | 无 |
| 返回值 | 无 |
| 备注 | 实现后的本方法如果返回失败，会导致线程结束，所以必须根据整体业务慎重考虑返回值 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | int SelfInstanceId() |
| 功能 | 返回线程id | 
| 参数 | 无 |
| 返回值 | 线程id<br>-1 ：无效id<br>>0 ：有效id |
| 备注 | 只有当实例化一个线程对象，并且创建线程后，本接口才能返回正确的值。实例化后创建线程前，是没有线程id的，返回值为INVALID_INSTANCE_ID(-1) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | string& SelfInstanceName() |
| 功能 | 返回线程名称 | 
| 参数 | 无 |
| 返回值 | 线程名称：<br>空字符串：无效名称<br>非空字符串：有效名称 |
| 备注 | 线程相关的接口，需要线程创建后才有效 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | aclrtContext GetContext() |
| 功能 | 返回线程运行时的context | 
| 参数 | 无 |
| 返回值 | context：<br>nullptr：无效context<br>非nullptr：有效context |
| 备注 | 线程相关的接口，需要线程创建后才有效 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | aclrtRunMode GetRunMode() |
| 功能 | 返回线程运行时的runmode | 
| 参数 | 无 |
| 返回值 | run mode：<br>ACL_DEVICE（昇腾AI软件栈运行在Device的Control CPU或板端环境）<br>ACL_HOST（昇腾AI软件栈运行在Host侧）|
| 备注 | 线程相关的接口，需要线程创建后才有效 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError BaseConfig(int instanceId, const string& threadName, aclrtContext context, aclrtRunMode runMode) |
| 功能 | 设置当前线程的线程id，线程名，context和run mode | 
| 参数 | instanceId：线程id<br>const string& threadName:线程名<br>context：线程context<br>aclrtRunMode runMode: 线程run mode |
| 返回值 | ACLLITE_OK：设置成功<br>非ACLLITE_OK：设置失败 |
| 备注 | 该函数期望仅在AclLiteApp::CreateAclLiteThreadMgr中调用一次，并且禁止在运行中随意调用修改这些属性 |

| 说明项 | <a name="AclLiteThreadParam ">具体描述</a> |
|---|---|
| 结构体 | AclLiteThreadParam |
| 功能 | 封装一个线程对象相关的属性和运行资源 | 
| 数据成员 |  threadInst ： 指向AclLiteThread对象的指针<br>threadInstName ： 线程名，全局唯一<br>context ： 线程所运行在的context<br> threadInstId ： 线程编号，全局唯一|

#### 5.3 AclLiteThreadMgr类
该类的原型定义在AclLiteThreadMgr.h文件中。AclLiteThreadMgr类顾名思义是为AclLiteThread配置一个管理线程间消息队列交互的对象，从而保证各业务线程对象之间逻辑上的隔离，使得开发者可以更专注业务线程功能的实现，而不用在AclLiteThread中纠结消息数据的发送与接受以及消息队列的维护。

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteThreadMgr(AclLiteThread* userThreadInstance, const std::string& threadName) |
| 功能 | 构造函数，生成线程对应的线程管理对象 | 
| 参数 | userThreadInstance：AclLiteThread线程实例<br>threadName：线程名称 |
| 返回值 | 无 |
| 约束 | 线程名称全局唯一 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | ~AclLiteThreadMgr() |
| 功能 | 析构函数 | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | static void ThreadEntry(void* data) |
| 功能 | CreateThread方法中创建的线程入口函数。本函数会在线程启动时，根据创建线程传入的context、run mode和用户线程对象实例，设置本线程的context, 调用用户线程实例对象的Init方法，然后轮询是否收到消息，收到后调用用户线程实例的Process函数。如果Process返回失败（非0值），轮询中止，线程结束。 | 
| 参数 | arg：线程创建参数，是线程管理对象类型指针，强转为void*类型 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void CreateThread() |
| 功能 | 调用线程入口函数拉起对应线程，并将线程detach | 
| 参数 | 无 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError WaitThreadInitEnd() |
| 功能 | 判断AclLiteThread线程实例是否初始化成功 | 
| 参数 | 无 |
| 返回值 | ACLLITE_OK：初始化成功<br>非ACLLITE_OK：初始化失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError PushMsgToQueue(shared_ptr<AclLiteMessage>& pMessage) |
| 功能 | 将消息数据送入消息队列 | 
| 参数 | pMessage：消息数据 |
| 返回值 | ACLLITE_OK：送入成功<br>非ACLLITE_OK：送入失败 |
| 备注 | AclLiteMessage结构体详见[**AclLiteMessage**](#AclLiteMessage) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | shared_ptr<AclLiteMessage> PopMsgFromQueue() |
| 功能 | 从消息队列弹出消息数据 | 
| 参数 | 无 |
| 返回值 | 消息数据：<br>nullptr：无效消息数据<br>非nullptr：有效消息数据 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteThread* GetUserInstance() |
| 功能 | 获取AclLiteThread线程实例 | 
| 参数 | 无 |
| 返回值 | AclLiteThread线程实例：<br>nullptr：无效线程实例<br>非nullptr：有效线程实例 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | const std::string& GetThreadName() |
| 功能 | 获取AclLiteThread线程名称 | 
| 参数 | 无 |
| 返回值 | 线程名称：<br>空字符串：获取名称失败<br>非空字符串：实际名称 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void SetStatus(AclLiteThreadStatus status) |
| 功能 | 设置线程实例状态 | 
| 参数 | status：要被设置的线程状态 |
| 返回值 | 无 |
| 备注 | AclLiteThreadStatus：<br>THREAD_READY = 0 // 线程就绪<br>THREAD_RUNNING = 1 // 线程运行<br>THREAD_EXITING = 2 // 线程退出中<br>THREAD_EXITED = 3 // 线程退出<br>THREAD_ERROR = 4 //线程初始化失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteThreadStatus GetStatus() |
| 功能 | 获取线程实例状态 | 
| 参数 | 无 |
| 返回值 | status：<br>THREAD_READY = 0 // 线程就绪<br>THREAD_RUNNING = 1 // 线程运行<br>THREAD_EXITING = 2 // 线程退出中<br>THREAD_EXITED = 3 // 线程退出<br>THREAD_ERROR = 4 //线程初始化失败 |

#### 5.4 一个例子：HelloWorld
```
#include <unistd.h>
#include <string>

#include "acllite/AclLiteApp.h"
#include "acllite/AclLiteThread.h"
#include "AclLiteResource.h"

#include <sys/time.h>

#define MSG_APP_START 0
#define MSG_HELLO     1
#define MSG_READ_FRAME 2
#define MSG_APP_EXIT 3

// 继承AclLiteThread类，完成具体业务功能，例如预处理线程类，后处理线程类，只要实现Process逻辑即可。
class HelloThread : public AclLiteThread
{
    AclLiteError Init()
    {
        ACLLITE_LOG_INFO("Hello thread init ok.");
        return ACLLITE_OK;
    }
    AclLiteError Process(int msgId, shared_ptr<void> msgData)
    { 
      
        shared_ptr<string> str = static_pointer_cast<string>(msgData);

        switch(msgId) {
            case MSG_APP_START:                
                SendMessage(SelfInstanceId(), MSG_HELLO, make_shared<string>("hello world"));
                break;
            case MSG_HELLO:
                cout << *str << endl;
                SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
                break;
            default:
                ACLLITE_LOG_ERROR("Preprocess thread receive unknow msg %d", msgId);
                break;
        }
        return ACLLITE_OK;
    }

};

// 创建线程并将创建的线程保存再线程表中
void CreateTshreads(vector<AclLiteThreadParam>& threadTbl, AclLiteResource& aclDev) {
    ACLLITE_LOG_INFO("begin CreateThreadInstance.");
    AclLiteThreadParam param;
    param.threadInst = new HelloThread();
    threadTbl.push_back(param);
    for (int i = 0; i < threadTbl.size(); i++) {
        threadTbl[i].context = aclDev.GetContext();
        threadTbl[i].runMode = aclDev.GetRunMode();
    }

    ACLLITE_LOG_INFO("end CreateThreadInstance.");
}

// 主线程处理函数，该函数要传递给app.wait函数，在app.wait中会循环调用该函数；
// 所以该函数的逻辑：如果收到的消息是应用退出消息，则设置退出标志位！
int MainThreadProcess(uint32_t msgId, 
                      shared_ptr<void> msgData, void* userData) {

    if (msgId == MSG_APP_EXIT) {
        AclLiteApp& app = GetAclLiteAppInstance();
        app.WaitEnd();
        ACLLITE_LOG_INFO("Receive exit message, exit now");       
    }

    return ACLLITE_OK;
    
}

int main()
{

    AclLiteResource aclDev = AclLiteResource();
    AclLiteError ret = aclDev.Init();

    // 创建线程表，并且创建线程后填充到线程表，后续所有线程都是通过线程表来访问的
    vector<AclLiteThreadParam> threadTbl;
    ACLLITE_LOG_INFO("before CreateThreadInstance.");
    CreateTshreads(threadTbl, aclDev);

    // AclLiteAPP是单例模式，直接返回app实例即可
    AclLiteApp& app = GetAclLiteAppInstance();

    // 启动app，每个线程都启动起来，开始循环等待消息了。
    ret = app.Start(threadTbl);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Start app failed, error %d", ret);
        app.Exit();
        return -1;
    }
    
    ret = SendMessage(threadTbl[0].threadInstId, MSG_APP_START, nullptr);

    // 在主线程中等待结束消息，如果收到结束消息，结束！
    // 原理：主线程会一直阻塞在当前wait函数，直到收到退出消息，会在MainThreadProcess中设置退出标识；wait中检查到退出标识，结束无线循环。
    app.Wait(MainThreadProcess, nullptr);

    app.Exit();
    
    return -1 ;
}
```

### 6. 其他

#### 6.1 AclLiteUtils.h

该文件定义AclLite公共库中使用的工具函数和宏

| 说明项 | 具体描述 |
|---|---|
| 宏 | RGBU8_IMAGE_SIZE(width, height) |
| 功能 | 计算RGB 24bits图片数据大小 | 
| 参数 | width：图片宽<br>height: 图片高 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | RGBF32_IMAGE_SIZE(width, height) |
| 功能 | 计算RGB C3F32图片数据大小 | 
| 参数 | width：图片宽<br>height: 图片高 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | YUV420SP_SIZE(width, height) |
| 功能 | 计算YUVSP420图片数据大小 | 
| 参数 | width：图片宽<br>height: 图片高 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | YUV420SP_CV_MAT_HEIGHT(height) |
| 功能 | 计算YUVSP420 nv12格式图片加载到opencv的mat时,height的数值 | 
| 参数 | yuv图片的高 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | SHARED_PTR_DVPP_BUF(buf) |
| 功能 | 新建指向dvpp内存的智能指针 | 
| 参数 | buf：dvpp内存的指针 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | SHARED_PTR_DEV_BUF(buf) |
| 功能 | 新建指向device内存的智能指针 | 
| 参数 | buf：device内存的指针 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | SHARED_PTR_U8_BUF(buf) |
| 功能 | 新建指向一边拿内存的智能指针 | 
| 参数 | buf：指向内存的指针 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ALIGN_UP(num, align) |
| 功能 | 计算对齐后的数值 | 
| 参数 | num：原始数值<br>align:要对齐的数 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ALIGN_UP2(num) |
| 功能 | 将数据按2对齐 | 
| 参数 | num：原始数值 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ALIGN_UP16(num) |
| 功能 | 将数据按16对齐 | 
| 参数 | num：原始数值 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ALIGN_UP128(num) |
| 功能 | 将数据按128对齐 | 
| 参数 | num：原始数值 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | SIZEOF_ARRAY(array) |
| 功能 | 计算数组中数据的个数 | 
| 参数 | array：数组 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ACLLITE_LOG_ERROR(fmt, ...) |
| 功能 | 打印acl ERROR级别日志到/var/log/npu/slog/host-0/host-xxxx.log | 
| 参数 | fmt：打印的日志内容，支持格式化字符串,记录的日志包括函数、文件和行号 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ACLLITE_LOG_INFO(fmt, ...) |
| 功能 | 打印acl INFO级别日志到/var/log/npu/slog/host-0/host-xxxx.log | 
| 参数 | fmt：打印的日志内容，支持格式化字符串,记录的日志包括函数、文件和行号 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ACLLITE_LOG_WARNING(fmt, ...) |
| 功能 | 打印acl WARNING级别日志到/var/log/npu/slog/host-0/host-xxxx.log | 
| 参数 | fmt：打印的日志内容，支持格式化字符串,记录的日志包括函数、文件和行号 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | ACLLITE_LOG_DEBUG(fmt, ...) |
| 功能 | 打印acl DEBUG级别日志到/var/log/npu/slog/host-0/host-xxxx.log | 
| 参数 | fmt：打印的日志内容，支持格式化字符串,记录的日志包括函数、文件和行号 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_START(X) |
| 功能 | 时间戳起点 | 
| 参数 | X：待测量函数名 |
| 备注 | TIME_START(X)与TIME_END(X)需要搭配使用，以同一X为一组；再辅以TIME_XXX_SHOW(X)系列宏打印 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_END(X) |
| 功能 | 时间戳终点 | 
| 参数 | X：待测量函数名 |
| 备注 | TIME_START(X)与TIME_END(X)需要搭配使用，以同一X为一组；再辅以TIME_XXX_SHOW(X)系列宏打印 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_USEC(X) |
| 功能 | 以微秒计算耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_USEC_SHOW(X) |
| 功能 | 以微秒打印耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_MSEC(X) |
| 功能 | 以毫秒计算耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_MSEC_SHOW(X) |
| 功能 | 以毫秒打印耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_SEC(X) |
| 功能 | 以秒计算耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_SEC_SHOW(X) |
| 功能 | 以秒打印耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_MINUTE(X) |
| 功能 | 以分钟计算耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_MINUTE_SHOW(X) |
| 功能 | 以分钟打印耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_HOUR(X) |
| 功能 | 以小时计算耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 宏 | TIME_HOUR(X) |
| 功能 | 以小时打印耗时 | 
| 参数 | X：待测量函数名 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool IsDirectory(const std::string &path) |
| 功能 | 判断字符串是否为有效的文件夹路径 | 
| 参数 | path：输入的字符串 |
| 返回值 | true：有效路径<br>false：无效路径 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void* CopyDataToDevice(const void* data, uint32_t size, aclrtRunMode curRunMode, MemoryType memType) |
| 功能 | 将数据拷贝到device侧/dvpp内存 | 
| 参数 | data：待拷贝数据<br>size：待拷贝数据大小<br>curRunMode：程序runmode<br> memType：目标侧内存类型|
| 返回值 | 目标侧内存指针：<br>nullptr：拷贝失败<br>非nullptr拷贝成功 |
| 备注 | aclrtRunMode取值见官方文档；<br>MemoryType取值见[**MemoryType**](#MemoryType)，本接口中，一般取值为MEMORY_DEVICE/MEMORY_DVPP |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CopyDataToDeviceEx(void* dest, uint32_t destSize, const void* src, uint32_t srcSize, aclrtRunMode runMode) |
| 功能 | 将数据拷贝到device侧 | 
| 参数 | dest：目标内存<br>destSize：目标内存大小<br>src：待拷贝内存<br>srcSize：待拷贝内存大小<br>curRunMode：程序run mode |
| 返回值 | ACLLITE_OK：拷贝成功<br>非ACLLITE_OK：拷贝失败 |
| 备注 | aclrtRunMode取值见官方文档 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void* CopyDataToHost(const void* data, uint32_t size， aclrtRunMode curRunMode, MemoryType memType) |
| 功能 | 将数据拷贝到host侧/一般内存上 | 
| 参数 | data：待拷贝数据<br>size：待拷贝数据大小<br>curRunMode：程序runmode<br> memType：目标侧内存类型|
| 返回值 | 目标侧内存指针：<br>nullptr：拷贝失败<br>非nullptr拷贝成功 |
| 备注 | aclrtRunMode取值见官方文档；<br>MemoryType取值见[**MemoryType**](#MemoryType)，本接口中，一般取值为MEMORY_NORMAL/MEMORY_HOST |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CopyDataToHostEx(void* dest, uint32_t destSize, const void* src, uint32_t srcSize, aclrtRunMode runMode); |
| 功能 | 将数据拷贝到host侧 | 
| 参数 | dest：目标内存<br>destSize：目标内存大小<br>src：待拷贝内存<br>srcSize：待拷贝内存大小<br>curRunMode：程序run mode |
| 返回值 | ACLLITE_OK：拷贝成功<br>非ACLLITE_OK：拷贝失败 |
| 备注 | aclrtRunMode取值见官方文档 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void* CopyData(const void* data, uint32_t size, aclrtMemcpyKind policy, MemoryType memType) |
| 功能 | 数据拷贝 | 
| 参数 | data：待拷贝数据<br>size：待拷贝数据大小<br>policy：内存拷贝种类<br> memType：目标侧内存类型 |
| 返回值 | 目标侧内存指针 |
| 备注 | aclrtMemcpyKind取值见官方文档 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError ReadPng(ImageData& image, const std::string& fileName) |
| 功能 | 读取png图片 | 
| 参数 | image：存放被读取图片数据<br>fileName：被读取的图片文件路径 |
| 返回值 | ACLLITE_OK：读取成功<br>非ACLLITE_OK：读取失败 |
| 约束 | 只支持baseline不支持渐进式的Png图片 |
| 备注 | ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError ReadJpeg(ImageData& image, const std::string& fileName) |
| 功能 | 读取jpeg图片 | 
| 参数 | image：存放被读取图片数据<br>fileName：被读取的图片文件路径 |
| 返回值 | ACLLITE_OK：读取成功<br>非ACLLITE_OK：读取失败 |
| 约束 | 只支持baseline不支持渐进式的jpeg图片 |
| 备注 | ImageData数据结构详见[**ImageData**](#ImageData) |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void GetAllFiles(const std::string &pathList， std::vector<std::string> &fileVec) |
| 功能 | 获取某路径文件下所有文件的文件名 | 
| 参数 | pathList：文件路径<br>fileVec：存放文件名的向量 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void SaveBinFile(const std::string& filename, const void* data, uint32_t size) |
| 功能 | 将数据存为二进制文件 | 
| 参数 | filename：文件名<br>data：待保存数据<br>size：待保存数据大小 |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError ReadBinFile(const std::string& filename, void*& data, uint32_t& size) |
| 功能 | 读取二进制文件 | 
| 参数 | filename：文件名<br>data：待读取数据<br>size：待读取数据大小 |
| 返回值 | ACLLITE_OK：读取成功<br>非ACLLITE_OK：读取失败 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CopyImageToLocal(ImageData& destImage, ImageData& srcImage, aclrtRunMode curRunMode); |
| 功能 | 将ImageData类型数据拷贝到本地 | 
| 参数 | destImage：拷贝后的数据<br>srcImage: 待拷贝数据<br>curRunMode：程序run mode |
| 返回值 | ACLLITE_OK：拷贝成功<br>非ACLLITE_OK：拷贝失败 |
| 备注 | aclrtRunMode取值见官方文档 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | AclLiteError CopyImageToDevice(ImageData& destImage, ImageData& srcImage， aclrtRunMode curRunMode, MemoryType memType); |
| 功能 | 将ImageData类型数据拷贝到本地拷贝到device/dvpp | 
| 参数 | destImage：拷贝后的数据<br>srcImage: 待拷贝数据<br>curRunMode：程序run mode<br>memType：目标侧内存类型 |
| 返回值 | ACLLITE_OK：拷贝成功<br>非ACLLITE_OK：拷贝失败 |
| 备注 | aclrtRunMode取值见官方文档；<br>MemoryType取值见[**MemoryType**](#MemoryType)，本接口中，一般取值为MEMORY_DEVICE/MEMORY_DVPP |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool IsIpAddrWithPort(const std::string& addrStr) |
| 功能 | 判断ip是否形如：<1-255>.<0-255>.<0-255>.<0-255>:\<port> | 
| 参数 | addrStr：字符串，待判断ip |
| 返回值 | true：格式正确<br>false：格式错误 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void ParseIpAddr(std::string& ip, std::string& port, const std::string& addr) |
| 功能 | 将形如：<1-255>.<0-255>.<0-255>.<0-255>:\<port>拆成ip和port两个字符串变量 | 
| 参数 | ip：拆后的字符串变量，ip部分<br>port：拆后的字符串变量，port部分<br>addr：字符串变量，完整地址  |
| 返回值 | 无 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool IsVideoFile(const std::string& path) |
| 功能 | 判断字符串是否为MP4文件 | 
| 参数 | path：待判断字符串 |
| 返回值 | true：是MP4<br>false：不是MP4 |
| 备注 | 该函数仅校验文件名，并非真的对文件性质做判断 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool IsRtspAddr(const std::string &str) |
| 功能 | 判断字符串是否为rtsp流格式，形如：rtsp://...... | 
| 参数 | str：待判断字符串 |
| 返回值 | true：是rtsp流<br>false：不是rtsp流 |
| 备注 | 该函数仅校验文件名，并非真的对文件性质做判断 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool IsDigitStr(const std::string& str) |
| 功能 | 判断字符串是否为数字字符串 | 
| 参数 | str：待判断字符串 |
| 返回值 | true：是数字字符串<br>false：不是数字字符串 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool IsPathExist(const std::string &path) |
| 功能 | 判断文件路径是否存在  | 
| 参数 | path：待判断字符串 |
| 返回值 | true：路径存在<br>false：路径不存在 |

| 说明项 | 具体描述 |
|---|---|
| 函数 | bool ReadConfig(std::map<std::string, std::string>& config, const char* configFile) |
| 功能 | 读取配置文件，并将解析结果保存到map中 | 
| 参数 | config：保存解析结果的map<br>configFile：待读取的文件路径 |
| 返回值 | true：读取成功<br>false：读取失败 |
| 约束 | 配置项名称唯一且格式如下：<br> [baseconf]<br>presenter_server_ip=xxx.xxx.x.xxx |

| 说明项 | 具体描述 |
|---|---|
| 函数 | void PrintConfig(const std::map<std::string, std::string> & m) |
| 功能 | 打印配置项 | 
| 参数 | m：待打印配置项map |
| 返回值 | 无 |

<a name="AclLiteType.h"></a>
#### 6.2 AclLiteType.h

该文件定义AclLite公共库中使用的结构体和枚举变量

| 说明项 | <a name="MemoryType">具体描述</a> |
|---|---|
| 枚举类型 | MemoryType |
| 功能 | 标记不同类型的内存 | 
| 可选值 | MEMORY_NORMAL = 0 // 一般内存，用new/delete申请/释放<br>MEMORY_HOST // host侧内存，用aclrtMallocHost/aclrtFreeHost申请/释放<br>MEMORY_DEVICE // device侧内存，用aclrtMalloc/aclrtFree申请/释放<br>MEMORY_DVPP // dvpp内存，用acldvppMalloc/acldvppFree申请/释放<br>MEMORY_INVALID_TYPE // 无效内存类型 |

| 说明项 | 具体描述 |
|---|---|
| 枚举类型 | CopyDirection |
| 功能 | 标记拷贝方向 | 
| 可选值 | TO_DEVICE = 0 // 往device侧<br>TO_HOST // 往host侧<br>INVALID_COPY_DIRECT // 无效拷贝方向 |

| 说明项 | 具体描述 |
|---|---|
| 枚举类型 | CameraId |
| 功能 | 标记Atlas200DK摄像头 | 
| 可选值 | CAMERA_ID_0 = 0 // 0号槽位摄像头<br>CAMERA_ID_1 // 1号槽位摄像头<br>INVALID_COPY_DIRECT // 无效槽位 |

| 说明项 | 具体描述 |
|---|---|
| 枚举类型 | VencStatus |
| 功能 | venc业务线程的状态 | 
| 可选值 | STATUS_VENC_INIT = 0 // 初始化<br>STATUS_VENC_WORK // 工作中<br>STATUS_VENC_FINISH // 业务完成<br>STATUS_VENC_EXIT // 线程退出<br>STATUS_VENC_ERROR // 初始化失败/编码报错等报错状态 |

| 说明项 | <a name="VencConfig">具体描述</a> |
|---|---|
| 结构体 | VencConfig |
| 功能 | 存放VencHelper初始化所需参数 | 
| 数据成员 | maxWidth ： 宽<br>maxHeight ： 高<br>outFile ： 编码输出文件<br>format ： 待编码图片格式<br>enType ： 编码文件流格式<br>context ：线程运行context<br>runMode ： 线程运行run mode |
| 约束 | format：PIXEL_FORMAT_YUV_SEMIPLANAR_420 / PIXEL_FORMAT_YVU_SEMIPLANAR_420 <br>enType：H265_MAIN_LEVEL / H264_BASELINE_LEVEL / H264_MAIN_LEVEL / H264_HIGH_LEVEL|

| 说明项 | <a name="ImageData">具体描述</a> |
|---|---|
| 结构体 | ImageData |
| 功能 | 封装图片数据及图片参数的数据结构 | 
| 数据成员 |  format ： 图片格式<br>width ： 图片宽<br>height ： 图片高<br> alignWidth ： 对齐后宽<br>alignHeight ： 对齐后高<br>size ： 图片数据大小<br>data ： 图片数据 |

| 说明项 | 具体描述 |
|---|---|
| 结构体 | Resolution |
| 功能 | 分辨率 | 
| 数据成员 | width：宽<br>height：高 |

| 说明项 | 具体描述 |
|---|---|
| 结构体 | Rect |
| 功能 | 矩形框坐标点 | 
| 数据成员 | ltX：左上点的X坐标<br>ltY：左上点的Y坐标<br>rbX：右下点的X坐标<br>rbY：右下点的Y坐标 |

| 说明项 | 具体描述 |
|---|---|
| 结构体 | BBox |
| 功能 | BoundingBox数据 | 
| 数据成员 | rect：矩形框左边点<br>score：置信度<br>text：类别标签 |
| 备注 |  |

| 说明项 | <a name="AclLiteMessage">具体描述</a> |
|---|---|
| 结构体 | AclLiteMessage |
| 功能 | 消息数据 | 
| 数据成员 | dest：目标线程id<br>msgId：消息Id<br>data：消息数据 |

| 说明项 | <a name="DataInfo">具体描述</a> |
|---|---|
| 结构体 | DataInfo |
| 功能 | 数据信息，存储数据内容及数据大小，用来生成模型输入输出 | 
| 数据成员 | data：数据<br>size：数据大小 |
| 备注 |  |

| 说明项 | <a name="ModelOutputInfo">具体描述</a> |
|---|---|
| 结构体 | ModelOutputInfo |
| 功能 | 模型输出节点数据信息，存储模型每个输出节点的具体信息 | 
| 数据成员 | name：模型输出的输出算子名称、算子输出边下标、top名称或输出名称<br>dims：模型输出维度信息的指针<br>format：输出的Format<br>dataType：输出的数据类型 |
| 备注 |  |

| 说明项 | <a name="InferenceOutput">具体描述</a> |
|---|---|
| 结构体 | InferenceOutput |
| 功能 | 存放模型推理结果数据 | 
| 数据成员 | data：结果数据<br>size：数据大小 |

#### 6.3 AclLiteError.h

该文件定义AclLite公共库中使用的错误码，详见文件。

#### 6.4 ThreadSafeQueue.h

该文件定义了一个模板类ThreadSafeQueue，旨在为用户提供在多线程场景下，可以直接复用的数据安全队列，详见文件。
