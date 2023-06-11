中文|[English](./README.md)
# DVPP数据预处理基础说明

### 章节目标
- 了解DVPP的基本原理与使用限制。
- 学会使用VPC进行图像缩放、裁剪等操作。
- 学会使用JPEGD进行图像解码操作。

### 课程引入
受网络结构和训练方式等因素的影响，绝大多数神经网络模型对输入数据都有格式上的限制。在计算视觉领域，这个限制大多体现在图像的尺寸、色域、归一化参数等。如果源图或视频的尺寸、格式等与网络模型的要求不一致时，就需要将源图或视频处理成符合模型要求的图或视频。    
在初步学习的过程中，只需要掌握图片处理的使用场景即可；以下是图片处理的一般场景：
![图片处理场景](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E9%A2%84%E5%A4%84%E7%90%86%E5%9C%BA%E6%99%AF.jpg)  

在[图片处理]()应用中，使用的是opencv进行的输入数据处理；而昇腾CANN也提供了一整套的图片/视频的输入数据处理的方式，即DVPP(Digital Vision Pre-Processing)和AIPP(Artificial Intelligence Pre-Processing)。它们完成上述场景的一般流程如下：
![流程比对](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E6%B5%81%E7%A8%8B%E6%AF%94%E5%AF%B9.jpg)  
其中，AIPP相信开发者们都已经有所了解，和传统流程对比，使用昇腾CANN提供的AIPP，对归一化和色域转换步骤进行了加速。  
那么DVPP是什么？DVPP的流程相较于传统流程的优势在哪？请带着这样的疑问继续往下学习。  

### DVPP概述
本章的重点是DVPP，以下是DVPP的介绍：    
**DVPP**：DVPP（Digital Video Pre-Processing）是昇腾AI处理器内置的图像处理单元，通过AscendCL媒体数据处理接口提供强大的媒体处理硬加速能力，主要功能包括缩放、抠图、格式转换、图片编解码、视频编解码等。如下图所示：    
![DVPP概念图](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/DVPP%E6%A6%82%E5%BF%B5%E5%9B%BE.png)   
1. 用户通过AscendCL调用DVPP执行器提供的数据传输API，执行器将输入数据传输到DVPP内存中。
2. 用户通过AscendCL调用DVPP执行器提供的DVPP功能API，执行器通过DVPP驱动相关硬件完成计算，结果保存在DVPP内存中。
3. 完成所有DVPP计算后，用户调用数据传输接口，将DVPP输出的内存拷贝出来，并进行后续处理。    

那么，DVPP提供了哪些功能呢？如下表所示：   

| 功能  | 说明  |
|---|---|
| VPC（Vision Preprocessing Core）  |  处理YUV、RGB等格式的图片，包括缩放（resize）、抠图（crop）、图像金字塔、色域转换等。 |
| JPEGD（JPEG Decoder）  | JPEG压缩格式-->YUV格式的图片解码。  |
| JPEGE（JPEG Encoder）  | YUV格式-->JPEG压缩格式的图片编码。  |
| VDEC（Video Decoder）  | H264/H265格式-->YUV/RGB格式的视频码流解码。  |
| VENC（Video Encoder）  | YUV420SP格式-->H264/H265格式的视频码流编码。  |
| PNGD（PNG decoder）  | PNG格式-->RGB格式的图片解码。  |    

其中，本章节的重点是**VPC**和**JPEGD**。VPC负责图像处理功能，JPEGD负责完成JPEG图像的解码功能。       
综上所述，DVPP使用了专用的硬件，在昇腾设备上为相关数据处理提供了硬件加速的能力。相较于传统流程的软件处理，能够更快的处理好输入数据，保障较大输入数据量或要求实时呈现的AI推理业务。   
但是由于DVPP硬件对图片宽、高有一定的限制，所以在使用DVPP进行数据处理时，需要将图片进行宽高对齐；也就引入了**宽stride、高stride**两个概念。  
- **宽stride**：指一行图像跨距，表示输入/输出图片对齐后的宽。    
- **高stride**：指图像在内存中的行数，表示输入/输出图片对齐后的高。   

DVPP中不同的功能，对不同格式的输入或输出图片的宽、高对齐要求也不同。初步学习过程中，仅需要了解JPEGD及VPC即可。    
![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E5%AE%BD%E9%AB%98stride.png)      
如上图，进行JPEGD时，由于图片size不满足128*16的对齐要求，所以在申请JPEGD的输出内存时，需要申请对齐后大小的内存空间。此时会在原图有效数据右侧、下侧补充对齐的无效数据。    
VPC处理时，输入为JPEGD的输出，满足16\*2的要求，同时，输出224\*224满足VPC的输出对齐16\*2的要求，所以也不会补充无效数据进行补边。   
其中有些特性在改场景下没有体现，会在后续的功能详细介绍中说明。以下简单进行列举。   
1. 当芯片为昇腾310P时，JPEGD的对齐要求为64\*16，当芯片为昇腾310/昇腾910时，JPEGD的对齐要求为128\*16。
2. VPC串接JPEGD时，会同时获取JPEGD输出的宽、高stride和宽、高值。实际处理时只会对宽、高范围内的有效数据进行图像处理。如上图VPC处理的是JPEGD输出中500*300范围内的有效内容，输出时再根据对齐要求进行补边。
3. DVPP的接口除了对宽、高stride有要求外，对输入图片的宽高也有一定要求，如要求宽高2对齐的情况下，输入奇数宽高（如213\*213）的图片会报错，需要在代码中进行规避。

DVPP的概念和一般场景理解后，接下来就可以学习DVPP的功能接口了。  

### DVPP内存管理
前面章节中已经介绍了Host&Device的内存管理。但是Device侧内存管理实际上有两套接口。    
由于芯片上内置了图像处理单元DVPP(Digital Video Pre-Processor)，该单元需要独立的进行内存管理，所以提供了一套专属于DVPP的内存申请与释放接口，函数原型如下：
```
aclError acldvppMalloc(void **devPtr, size_t size)
aclError acldvppFree (void *devPtr)
```   
同理，acldvppMalloc是DVPP内存申请接口，acldvppFree是DVPP内存释放接口；调用伪码如下：
```
...
void *devInput = nullptr;
size_input = 256;
ret = acldvppMalloc(&devInput, size_input, ACL_MEM_MALLOC_HUGE_FIRST);
if (devInput != nullptr) {
  ret = acldvppFree(devInput);
}
...
```
> :notebook: **说明:**   
>- DVPP侧的内存也是Device上的内存，那么为什么要进行独立的内存管理呢？    
>  因为DVPP处理时对输入、输出的数据有更高的要求（例如，内存首地址128字节对齐）。如果直接使用Device的内存申请接口，获取的内存并不一定能满足DVPP的要求，所以需要专门的内存接口进行管理。    
>  同时，DVPP申请的内存因为有更高的要求，所以肯定能符合其它Device操作的限制（例如，模型推理）。所以，从性能角度，可以直接将DVPP处理完的输出内存作为模型推理的输入，实现内存复用。    
>  但是，DVPP可访问的内存地址空间是有限的，所以为了确保进行DVPP处理时内存足够，除DVPP处理功能外的其它功能（例如，模型加载），还是建议使用Device的内存管理接口进行管理。
>- DVPP侧的内存如何进行数据传输？
>  由于DVPP侧的内存也是Device上的内存，所以数据传输和Device一致。如果是DVPP管理内存和Device管理内存之间要进行传输，则设置aclrtMemcpyKind为ACL_MEMCPY_DEVICE_TO_DEVICE即可。

#### 通用接口
1. 通道创建与释放  
   创建通道是DVPP整个调用流程的第一步，对于本次介绍的图片处理的接口，是可以复用同一个通道进行多个数据处理的。
   创建通道前需要创建acldvppChannelDesc类型的数据，表示创建图片数据处理通道时的通道描述信息，其函数原型如下。
   ```
   acldvppChannelDesc *acldvppCreateChannelDesc()
   ```
   拥有acldvppChannelDesc类型的数据后，就可以创建通道了；其函数原型如下所示。
   ```
   aclError acldvppCreateChannel(acldvppChannelDesc *channelDesc)
   ```
   当然，当DVPP调用结束后，就必须要按顺序，先销毁图片数据处理的通道，再销毁通道描述信息。
   ```
   aclError acldvppDestroyChannel(acldvppChannelDesc *channelDesc)
   aclError acldvppDestroyChannelDesc(acldvppChannelDesc *channelDesc)
   ```
   相对来说，通道的创建与销毁过程较为固定，只需要在DVPP开始之前进行创建，DVPP结束之后进行销毁即可。可参考如下调用。
   ```
   ...
   acldvppChannelDesc dvppChannelDesc_ = acldvppCreateChannelDesc();
   aclError ret = acldvppCreateChannel(dvppChannelDesc_);
   ...
   acldvppDestroyChannel(dvppChannelDesc_);
   (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
   dvppChannelDesc_ = nullptr;
   ...  
   ```       
2. 图片描述信息   
   DVPP接口中会提及图片描述信息的概念，相当于图片各个信息的结构体，包含了图片的数据、格式、宽高、对齐后的宽高、大小等信息。再执行DVPP接口之前，需要先创建图片描述信息，再进行操作，DVPP操作完毕后需要销毁图片描述信息。调用接口如下：
   ```
   acldvppPicDesc *acldvppCreatePicDesc()
   aclError acldvppDestroyPicDesc(acldvppPicDesc *picDesc)
   ```
   创建完成后，当然需要对图片描述信息中的各项数据进行设置，接口如下：
   ```
   aclError acldvppSetPicDescData(acldvppPicDesc *picDesc, void *dataDev);
   aclError acldvppSetPicDescSize(acldvppPicDesc *picDesc, uint32_t size);
   aclError acldvppSetPicDescFormat(acldvppPicDesc *picDesc, acldvppPixelFormat format);
   aclError acldvppSetPicDescWidth(acldvppPicDesc *picDesc, uint32_t width);
   aclError acldvppSetPicDescHeight(acldvppPicDesc *picDesc, uint32_t height);
   aclError acldvppSetPicDescWidthStride(acldvppPicDesc *picDesc, uint32_t widthStride);
   aclError acldvppSetPicDescHeightStride(acldvppPicDesc *picDesc, uint32_t heightStride);
   aclError acldvppSetPicDescRetCode(acldvppPicDesc *picDesc,uint32_t retCode)
   ```
   有数据设置接口，自然有数据获取接口，可以在程序中获取图片描述信息中的各个属性值，接口如下：
   ```
   void *acldvppGetPicDescData(const acldvppPicDesc *picDesc);
   uint32_t acldvppGetPicDescSize(const acldvppPicDesc *picDesc);
   acldvppPixelFormat acldvppGetPicDescFormat(const acldvppPicDesc *picDesc);
   uint32_t acldvppGetPicDescWidth(const acldvppPicDesc *picDesc);
   uint32_t acldvppGetPicDescHeight(const acldvppPicDesc *picDesc);
   uint32_t acldvppGetPicDescWidthStride(const acldvppPicDesc *picDesc);
   uint32_t acldvppGetPicDescHeightStride(const acldvppPicDesc *picDesc);
   uint32_t acldvppGetPicDescRetCode(const acldvppPicDesc *picDesc);
   ```
   当然，图片描述信息中的Data需要通过DVPP内存管理接口进行管理。调用伪码如下：
   ```
   ...
   acldvppPicDesc *vpcDesc_ = acldvppCreatePicDesc();
   acldvppSetPicDescData(vpcDesc_ , BufferDev);
   acldvppSetPicDescFormat(vpcDesc_ , PIXEL_FORMAT_YUV_SEMIPLANAR_420);
   acldvppSetPicDescWidth(vpcDesc_ , Width);
   acldvppSetPicDescHeight(vpcDesc_ , Height);
   acldvppSetPicDescWidthStride(vpcDesc_ , WidthStride);
   acldvppSetPicDescHeightStride(vpcDesc_ , HeightStride);
   acldvppSetPicDescSize(vpcDesc_ , BufferSize);
   ...
   ```

### JPEGD图片解码
JPEGD图片解码的功能和限制已经在前面进行过介绍了，下面来说明下JPEGD图片解码的流程和API。

1. JPEGD图片解码流程：    
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/JPEGD%E5%8A%9F%E8%83%BD%E6%9E%B6%E6%9E%84.jpeg)      
   如上图，典型的调用顺序如下所示：    
   - 创建通道描述对象：调用acldvppCreateChannelDesc创建acldvppChannelDesc对象
   - 创建通道：调用acldvppCreateChannel（ acldvppChannelDesc ）创建通道
   - 创建图片描述：调用acldvppCreatePicDesc创建acldvppPicDesc对象
   - 执行操作……
   - 销毁图片描述：调用acldvppDestroyPicDesc（ acldvppPicDesc ）销毁图片描述
   - 创建图片描述：调用acldvppCreatePicDesc创建acldvppPicDesc对象
   - 执行操作……
   - 销毁图片描述：调用acldvppDestroyPicDesc（ acldvppPicDesc ）销毁图片描述
   - 销毁通道：调用acldvppDestroyChannel（ acldvppChannelDesc ）销毁通道
   - 销毁通道描述对象：调用acldvppDestroyChannelDesc（ acldvppChannelDesc ）销毁通道描述对象

2. JPEGD接口介绍：   
   结合上图，通道和图片描述已经在前面介绍过，接下来就是具体业务流程，如对Jpeg图片进行解码。函数原型如下：
   ```
   aclError acldvppJpegDecodeAsync(acldvppChannelDesc *channelDesc, const void *data,uint32_t size, acldvppPicDesc *outputDesc, aclrtStream stream)
   ```
   由接口可以看到，进行JpegD图片解码时，输出图片需要创建图片描述，对于此处的图片描述参数有以下限制。
   - 宽高对齐一般为128*16对齐，如图片实际宽为127，那么宽取值为127，宽对齐取值为128。只有310P芯片中对齐为64*16。
   - pegD可输出多种格式的数据，本指导中仅要求输出YUV420SP类型即可。也就是输出格式固定写为PIXEL_FORMAT_YUV_SEMIPLANAR_420。

   解码过程中需要获取输入图片的宽高信息，可以手动输入，也可以通过以下函数获取。
   ```
   aclError acldvppJpegGetImageInfo(const void *data,uint32_t size,uint32_t *width,uint32_t *height,int32_t *components)
   ```
   如果不知道如何获取JpegD输出内存大小，可以使用下面这个函数计算。
   ```
   aclError acldvppJpegPredictDecSize(const void *data,uint32_t dataSize,acldvppPixelFormat outputPixelFormat,uint32_t *decSize)
   ```

3. JPEGD代码示例：    
   根据如上信息，我们可以完成如下JpegD图片解码的伪码。
   ```
   ...
   //先加载图片及获取图片占用数据大小
   FILE *fp = fopen("XXX.JPEG", "rb");
   fseek(fp, 0, SEEK_END);
   uint32_t fileLen = ftell(fp);
   fseek(fp, 0, SEEK_SET);
   //待获取数据
   void *inBufferDev = nullptr;
   int image_width = 0, image_height = 0, jpegDecodeSize = 0;
   char* inputBuff = new(std::nothrow) char[fileLen];
   size_t readSize = fread(inputBuff, sizeof(char), fileLen, fp);
   //输入数据拷贝到DVPP device侧
   acldvppMalloc(&inBufferDev, inBufferSize);
   aclrtMemcpy(inBufferDev, inBufferSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
   //获取输入图片宽高
   acldvppJpegGetImageInfo(inputBuff, inputBuffSize, &image_width, &image_height, nullptr);
   //获取JPEGD输出内存预估大小
   acldvppJpegPredictDecSize(inputBuff, inputBuffSize, PIXEL_FORMAT_YUV_SEMIPLANAR_420, &jpegDecodeSize);
   //310芯片宽对齐为128，如下所示；310P芯片宽对齐为64。
   uint32_t decodeOutWidthStride = (image_width + 127) / 128 * 128; 
   uint32_t decodeOutHeightStride = (image_height + 15) / 16 * 16; 
   //申请输出DVPP内存  
   void* decodeOutDevBuffer；
   aclError ret = acldvppMalloc(&decodeOutDevBuffer, jpegDecodeSize);
   //输出图片描述创建及设置
   decodeOutputDesc = acldvppCreatePicDesc();
   acldvppSetPicDescData(decodeOutputDesc, decodeOutDevBuffer);
   acldvppSetPicDescFormat(decodeOutputDesc, PIXEL_FORMAT_YUV_SEMIPLANAR_420); 
   acldvppSetPicDescWidth(decodeOutputDesc, image_width);
   acldvppSetPicDescHeight(decodeOutputDesc, image_height);
   acldvppSetPicDescWidthStride(decodeOutputDesc, decodeOutWidthStride);
   acldvppSetPicDescHeightStride(decodeOutputDesc, decodeOutHeightStride);
   acldvppSetPicDescSize(decodeOutputDesc, jpegDecodeSize);
   //进行图片解码，其中inDevBuffer_为输入图片数据
   acldvppJpegDecodeAsync(dvppChannelDesc, inputBuffDev, fileLen, decodeOutputDesc, stream_);
   //异步接口执行完毕后，调用同步等待接口等待执行完成
   aclrtSynchronizeStream(stream_);
   ...
   ```

### VPC图片处理
1. VPC图片处理流程：    
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/VPC%E5%8A%9F%E8%83%BD%E6%9E%B6%E6%9E%84.jpeg)      
   如上图，典型的调用顺序如下所示：    
   - 创建通道描述对象：调用acldvppCreateChannelDesc创建acldvppChannelDesc对象
   - 创建通道：调用acldvppCreateChannel（ acldvppChannelDesc ）创建通道
   - 创建抠图区域位置配置：调用acldvppCreateRoiConfig创建acldvppRoiConfig对象
   - 创建缩放配置：调用acldvppCreateResizeConfig创建acldvppResizeConfig对象
   - 申请输入输出内存：调用acldvppMalloc申请Device内存存放输入输出数据
   - 执行抠图、缩放操作……
   - 释放输入输出内存：调用acldvppFree接口释放输入、输出内存
   - 申请输入输出内存：调用acldvppMalloc申请Device内存存放输入输出数据
   - 执行抠图、缩放操作……
   - 释放输入输出内存：调用acldvppFree接口释放输入、输出内存
   - 销毁缩放配置：调用acldvppDestroyResizeConfig销毁缩放配置
   - 销毁抠图区域位置配置：调用acldvppDestroyRoiConfig销毁抠图区域位置配置
   - 销毁通道：调用acldvppDestroyChannel（ acldvppChannelDesc ）销毁通道
   - 销毁通道描述对象：调用acldvppDestroyChannelDesc（ acldvppChannelDesc ）销毁通道描述对象

2. VPC接口介绍：    
   结合上图，通道和图片描述已经在前面介绍过，接下来介绍VPC处理的业务流程。     
   对于VPC处理，主要介绍Resize和CropandPaste两个接口，如下所示：
   ```
   aclError acldvppVpcResizeAsync(acldvppChannelDesc *channelDesc,acldvppPicDesc *inputDesc,acldvppPicDesc *outputDesc,acldvppResizeConfig *resizeConfig,aclrtStream stream)
   aclError acldvppVpcCropAndPasteAsync(acldvppChannelDesc *channelDesc,acldvppPicDesc *inputDesc,acldvppPicDesc *outputDesc,acldvppRoiConfig *cropArea,acldvppRoiConfig *pasteArea,aclrtStream stream)
   ```
   由接口可以分析出。
   - VPC接口都需要输入和输出的图片描述
   - Resize接口需要图片缩放配置数据
   - cropandpaste接口需要抠图区域位置

   设置图片缩放配置数据的接口如下所示:
   ```
   acldvppResizeConfig *acldvppCreateResizeConfig()
   aclError acldvppDestroyResizeConfig(acldvppResizeConfig *resizeConfig)
   ```
   设置抠图区域位置的接口如下所示：
   ```
   acldvppRoiConfig *acldvppCreateRoiConfig(uint32_t left, uint32_t right, uint32_t top, uint32_t bottom)
   aclError acldvppDestroyRoiConfig(acldvppRoiConfig *roiConfig)
   ```
   VPC的输入可以是JPEGD的输出数据，也可以是直接传入的二进制文件，以下伪码中，我们直接使用JPEGD的输出数据为例说明。
 
3. VPC处理代码示例：     
   根据如上信息，我们可以完成如下VPC的伪码。
   ```
   //设置模型宽高（也就是VPC的目标宽高）
   int modelInputWidth = 224;
   int modelInputHeight = 224;
   //设置对齐，输入图片的对齐需要根据来源进行填写，如VPC的输入为JPEGD的输出，那么VPC输入的对齐宽高则为JPEGD输出图片的对齐宽高
   uint32_t inputWidthStride = decodeOutWidthStride;
   uint32_t inputHeightStride = decodeOutHeightStride ;
   uint32_t inputBufferSize = jpegDecodeSize;
   //创建VPC输入图片描述
   acldvppPicDesc *vpcInputDesc = acldvppCreatePicDesc();
   acldvppSetPicDescData(vpcInputDesc, reinterpret_cast<char *>(decodeOutDevBuffer));
   acldvppSetPicDescFormat(vpcInputDesc, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
   acldvppSetPicDescWidth(vpcInputDesc, inputWidth);
   acldvppSetPicDescHeight(vpcInputDesc, inputHeight);
   acldvppSetPicDescWidthStride(vpcInputDesc, inputWidthStride);
   acldvppSetPicDescHeightStride(vpcInputDesc, inputHeightStride);
   acldvppSetPicDescSize(vpcInputDesc, inputBufferSize);
   //设置输出对齐，输出图片的对齐为缩放到结果的16*2对齐，一般缩放到模型宽高
   int resizeOutWidthStride = (modelInputWidth + 15) / 16 * 16;
   int resizeOutHeightStride = (modelInputHeight + 1) / 2 * 2;
   //这里默认使用PIXEL_FORMAT_YUV_SEMIPLANAR_420，也就是YUV420SP，所以内存计算为宽对齐*高对齐*3/2
   uint32_t vpcOutBufferSize = resizeOutWidthStride * resizeOutHeightStride * 3 / 2;
   void *vpcOutBufferDev = nullptr;
   acldvppMalloc(&vpcOutBufferDev, vpcOutBufferSize);
   //创建VPC输出图片描述
   acldvppPicDesc *vpcOutputDesc = acldvppCreatePicDesc();
   acldvppSetPicDescData(vpcOutputDesc, vpcOutBufferDev);
   acldvppSetPicDescFormat(vpcOutputDesc, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
   acldvppSetPicDescWidth(vpcOutputDesc, modelInputWidth);
   acldvppSetPicDescHeight(vpcOutputDesc, modelInputHeight);
   acldvppSetPicDescWidthStride(vpcOutputDesc, resizeOutWidthStride);
   acldvppSetPicDescHeightStride(vpcOutputDesc, resizeOutHeightStride);
   acldvppSetPicDescSize(vpcOutputDesc, vpcOutBufferSize);
   //设置图片缩放配置数据，执行resize
   acldvppResizeConfig *resizeConfig = acldvppCreateResizeConfig();
   acldvppVpcResizeAsync(dvppChannelDesc, vpcInputDesc, vpcOutputDesc, resizeConfig, stream_);
   aclrtSynchronizeStream(stream_);
   (void)acldvppDestroyResizeConfig(resizeConfig);
   resizeConfig = nullptr;
   //参数值为左、右、上、下偏移(将原始图片左上角为0，0做的偏移)，左偏移和上偏移必须为偶数，右偏移和下偏移必须为奇数；310P上没有奇偶限制。
   uint32_t oddNum = 1, cropSizeWidth = 200, cropSizeHeight = 200, cropLeftOffset = 512, cropTopOffset = 512;
   uint32_t cropRightOffset = cropLeftOffset + cropSizeWidth - oddNum;
   uint32_t cropBottomOffset = cropTopOffset + cropSizeHeight - oddNum;
   acldvppRoiConfig  *cropArea = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset, cropTopOffset, cropBottomOffset);
   uint32_t pasteLeftOffset = 0, pasteTopOffset = 0;
   uint32_t pasteRightOffset = pasteLeftOffset + cropSizeWidth - oddNum;
   uint32_t pasteBottomOffset = pasteTopOffset + cropSizeHeight - oddNum;
   acldvppRoiConfig *pasteArea = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset, pasteTopOffset, pasteBottomOffset);
   acldvppVpcCropAndPasteAsync(dvppChannelDesc, vpcInputDesc, vpcOutputDesc, cropArea, pasteArea, stream_);
   aclrtSynchronizeStream(stream_);
   (void)acldvppDestroyRoiConfig(cropArea_);
   cropArea_ = nullptr;
   (void)acldvppDestroyRoiConfig(pasteArea_);
   pasteArea_ = nullptr;
   ```
  
### <a name="step_5"></a>样例及场景介绍
为了帮助大家更好的理解数据传输的使用场景，这里提供了两个场景的样例，开发者可以根据以下样例中的readme进行调测运行，再根据源码理解本专题的知识点。
| 目录  | 场景  |
|---|---|
| [jpegd](https://github.com/Ascend/samples/tree/master/cplusplus/level2_simple_inference/0_data_process/jpegd)  | jpegd解码样例 |
| [resize](https://github.com/Ascend/samples/tree/master/cplusplus/level2_simple_inference/0_data_process/resize)    | resize缩放样例  |
| [cropandpaste](https://github.com/Ascend/samples/tree/master/cplusplus/level2_simple_inference/0_data_process/cropandpaste)    | 抠图贴图样例  |