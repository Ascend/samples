中文|[English](./README.md)
# 样例开发说明

### 本章学习目标
- 了解DVPP+推理端到端样例的执行流程和原理
- 学会如何分析模型及CV样例如何开发
    
### 开发说明
对于CV类的应用开发，一般而言都有如下几个步骤：    
**样例结构设计**：设计样例的流程编排，为后续步骤做准备。    
**原始模型及输入输出分析**：分析模型的输入输出，将模型转换为离线om模型。     
**样例编写**：根据编排好的流程及转换完成的模型进行样例的编写。     
**样例编译运行**：编写CMakeList，并进行样例编译与运行。
  
本章将结合[sampleResnetDVPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetDVPP)的样例对流程进行介绍，样例效果如下：    
![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/Resnet50样例展示.png)     

### 样例结构设计
sampleResnetDVPP样例是一个图片类的CV样例，一般而言，这种样例的流程都可以编排如下：
![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/ACL_sample_analysis.png)    
- **模型转换**：使用ATC工具将Resnet50.onnx模型转换为离线om模型。
- **资源初始化**：初始化所有资源，包括DVPP初始化、模型初始化等。
- **输入数据处理**：使用DVPP将数据处理到模型需要的大小。
- **模型推理**：使用om模型进行推理。
- **输出数据处理**：对模型推理结果进行处理，该样例是将输出前五置信度的标签进行打印。
- **资源释放**：释放所有资源，包括DVPP及模型资源等。   
- **数据传输**：样例运行过程中进行的数据传输动作

### 原始模型及输入输出分析
原始模型及输入输出分析是应用开发中极为关键的一环，也是最难的一环，模型分析的正确与否直接决定了样例是否可以正确编写，以及编写后是否能正常输出。   
主要包含：原始模型获取、原始模型输入分析、预处理分析、原始模型输出分析及后处理分析。

1. **原始模型获取**           
   [Resnet50模型](https://www.hiascend.com/zh/software/modelzoo/models/detail/2/3de5052e2775e650a6f3048cf82a5d22/1)是在华为提供的模型库[modelzoo](https://www.hiascend.com/software/modelzoo/)上下载的。在开发一个新的推理样例时，也可以在modelzoo上寻找需要的模型。     
   由于模型原始结构为pth，需要转换成onnx模型后才能进行下一步操作。可以根据ModelZoo的说明，在[ModelZoo-PyTorch](https://github.com/Ascend/ModelZoo-PyTorch/tree/master/ACL_PyTorch/built-in/cv/Resnet50_Pytorch_Infer)仓中获取到pth2onnx模型转换脚本，将pth模型转换为onnx模型。
       
2. **原始模型输入分析**  
   在模型介绍中，可以获取到输入数据：     
   | 输入数据 | 数据类型 | 大小                      | 数据排布格式 |
   | -------- | -------- | ------------------------- | ------------ |
   | input    | RGB_FP32 | batchsize x 3 x 256 x 256 | NCHW         |
   
   也可以使用netron打开原始模型，获取到模型的输入数据，如下所示：   
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/netron%E8%BE%93%E5%85%A5%E6%95%B0%E6%8D%AE%E5%A4%84%E7%90%86.png)
  
   可以得知模型需要输入的数据为RGB且数据格式为1,3,256,256;     
   除此之外还需要分析模型输入数据是否需要进行归一化操作；可以将[Resnet50模型](https://www.hiascend.com/zh/software/modelzoo/models/detail/2/3de5052e2775e650a6f3048cf82a5d22/1)源码下载下来，分析dataloaders.py文件：
   ```
   img_transforms = transforms.Compose(
     [transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor()]
    )

    img = img_transforms(Image.open(path))
    with torch.no_grad():
        # mean and std are not multiplied by 255 as they are in training script
        # torch dataloader reads data into bytes whereas loading directly
        # through PIL creates a tensor with floats in [0,1] range
        mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
        std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)

        if cuda:
            mean = mean.cuda()
            std = std.cuda()
            img = img.cuda()
        if fp16:
            mean = mean.half()
            std = std.half()
            img = img.half()
        else:
            img = img.float()

        input = img.unsqueeze(0).sub_(mean).div_(std)
   ```   
   可以看到，该模型的归一化操作包含了减均值和除去系数的操作，具体操作如下：
   - 通过transforms.ToTensor()将数据处理为CHW格式并将每一个数值归一化到[0,1]；      
   - 通过torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)构造NCHW格式的减均值数据，方便后续计算；
   - 通过torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)构造NCHW格式的乘系数数据，方便后续计算；
   - 通过img.unsqueeze(0)将CHW格式的数据扩展为NCHW格式；
   - 通过.sub_(mean).div_(std)，将[0,1]的数据按通道先减去均值，再除去系数。    
     R通道最后取值为[0,(1-0.485)*0.229]、G通道最后取值为[0,(1-0.456)*0.224]、B通道最后取值为[0,(1-0.406)*0.225]。

3. **预处理分析**     
   为了加快图片处理速度，所以预处理需要使用DVPP，根据前面得到的输入数据信息，预处理需要经过如下步骤：       
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/preprocess_pic.png)    
   其中AIPP的操作需要在模型转换的过程中操作。预处理如图所示进行JpegD和Resize即可。

4. 模型输出分析     
   在模型介绍中，可以获取到输出数据：   
   | 输出数据 | 大小     | 数据类型 | 数据排布格式 |
   | -------- | -------- | -------- | ------------ |
   | output1  | 1 x 1000 | FLOAT32  | ND           |       
 
   同样，也可以使用netron打开原始模型，获取到模型的输出数据，如下所示：     
   ![输入图片说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/netron%E8%BE%93%E5%87%BA%E6%95%B0%E6%8D%AE%E5%A4%84%E7%90%86.png)

   可以通过源码中的utils.py文件看到训练时判断准确率的方法：    
   ```
   def accuracy(output, target, topk=(1, )):
    """Computes the precision@k for the specified values of k"""
    # topk=(1,)取top1准确率，topk=(1,5)取top1和top5准确率
    maxk = max(topk)
    batch_size = target.size(0)
    # topk参数中，maxk取得是top1准确率，dim=1是按行取值， largest=1是取最大值
    _, pred = output.topk(maxk, 1, True, True)
    # 转置
    pred = pred.t()
    # 比较是否相等
    correct = pred.eq(target.view(1, -1).expand_as(pred))

    res = []
    for k in topk:
        correct_k = correct[:k].view(-1).float().sum(0, keepdim=True)
        res.append(correct_k.mul_(100.0 / batch_size))
    return res
   ```     
   首先将 output 中每个样本的预测概率值排名计算出来，然后判断排名前 topk 个中是否包含了正确答案的位置。如果包含，该样本就被认为是预测正确的。最后将所有正确预测的样本数除以总样本数，即可得到准确率。


4. 数据后处理分析   
   由模型输出分析可知，输出结果为1000个类别的置信度。所以数据后处理操作如下：     
   - 将1000个置信度信息保存下来。
   - 将1000个置信度结果相加。
   - 取出topk的置信度结果并除以总置信度取值（相当于做一次softmax，将置信度归一化到[0,1]之间，且保证所有置信度相加结果为1）。
   - 输出topk的标签及置信度。
  
### 样例编写  
1. 模型转换    
   分析完模型的输入输出，预处理及后处理后，接下来就要使用昇腾CANN提供的ATC工具进行模型转换了。      
   由前面的分析可以知道，ATC需要进行色域转换、减均值和归一化的操作，同时由于数据并未像pytorch数据处理一样，归一化到[0,1]之间，所以实际数据内容还是[0,255]的，所以AIPP整体需要进行如下处理：      
   - 数据需要从YUV转换成RGB格式；
   - 减去的均值mean应该是[0.485*255, 0.456*255, 0.406*255];
   - 除去的系数std应该是[0.229*255, 0.224*255, 0.225*255];
   - 由于原本的计算公式可抽象为(data-mean)/std，而在AIPP中计算公式可抽象为(data-mean)\*std，所以系数std应该为[1/(0.229*255), 1/(0.224*255), 1/(0.225*255)];
   
   配置文件aipp.cfg如下所示：
   ```
   aipp_op{
     aipp_mode:static
     input_format : YUV420SP_U8
     src_image_size_w : 224
     src_image_size_h : 224

     csc_switch : true
     rbuv_swap_switch : false
     matrix_r0c0 : 256
     matrix_r0c1 : 0
     matrix_r0c2 : 359
     matrix_r1c0 : 256
     matrix_r1c1 : -88
     matrix_r1c2 : -183
     matrix_r2c0 : 256
     matrix_r2c1 : 454
     matrix_r2c2 : 0
     input_bias_0 : 0
     input_bias_1 : 128
     input_bias_2 : 128

     crop: true
     load_start_pos_h : 0
     load_start_pos_w : 0

     min_chn_0 : 123.675
     min_chn_1 : 116.28
     min_chn_2 : 103.53
     var_reci_chn_0: 0.0171247538316637
     var_reci_chn_1: 0.0175070028011204
     var_reci_chn_2: 0.0174291938997821
   }
   ```
   AIPP文件准备好之后，就可以进行模型转换了，转换命令开发者应该很熟悉了，如下所示：
   ```
   atc --model=resnet50.onnx --framework=5 --output=resnet50 --input_shape="actual_input_1:1,3,224,224"  --soc_version=Ascend310  --insert_op_conf=aipp.cfg
   ```

2. 资源初始化     
   资源初始化模块，主要负责对程序运行期间可复用的资源进行一次统一的初始化。一般包含以下资源的初始化：      
   - AscendCL初始化       
     使用AscendCL接口开发应用时，必须先初始化AscendCL ，否则可能会导致后续系统内部资源初始化出错，进而导致其它业务异常。     
     ```
     const char *aclConfigPath = "";
     aclError ret = aclInit(aclConfigPath);
     ```
   - 运行管理资源申请与释放     
     初始化AscendCL后，需要按照固定流程申请运行管理资源。
     ```
     ret = aclrtSetDevice(deviceId_);
     ret = aclrtCreateContext(&context_, deviceId_);
     ret = aclrtCreateStream(&stream_);
     ```
   - DVPP初始化     
     代码需要使用DVPP进行JpegD和Resize操作，由于这些操作中包含一些可复用的资源，所以在初始化中直接申请，避免程序运行过程中反复申请。
     ```
     // DVPP通道描述申请
     dvppChannelDesc_ = acldvppCreateChannelDesc();
     // DVPP通道创建
     ret = acldvppCreateChannel(dvppChannelDesc_);
     // 创建图片缩放配置数据
     resizeConfig_ = acldvppCreateResizeConfig();
     // VPC操作输入输出图片描述创建
     vpcOutputDesc_ = acldvppCreatePicDesc();
     vpcInputDesc_ = acldvppCreatePicDesc();
     // JPEGD操作输出图片描述创建
     decodeOutputDesc_ = acldvppCreatePicDesc();
     ```
   - 模型初始化    
     模型操作中存在可复用的操作，可以在初始化过程中进行操作。
     ```
     // 模型加载
     ret = aclmdlLoadFromFile(modelPath_, &modelId_);
     // 创建模型描述
     modelDesc_ = aclmdlCreateDesc();
     
     // 创建输入Dataset
     int32_t index = 0;
     devBufferSize_ = aclmdlGetInputSizeByIndex(modelDesc_, index);
     aclrtMalloc(&picDevBuffer_, devBufferSize_, ACL_MEM_MALLOC_NORMAL_ONLY);
     inputDataset_ = aclmdlCreateDataset();
     aclDataBuffer *inputData = aclCreateDataBuffer(picDevBuffer_, devBufferSize_);
     aclmdlAddDatasetBuffer(inputDataset_, inputData);

     // 创建输出Dataset
     outputDataset_ = aclmdlCreateDataset();
     size_t outputSize = aclmdlGetNumOutputs(modelDesc_);
     for (size_t i = 0; i < outputSize; ++i) {
         size_t modelOutputSize = aclmdlGetOutputSizeByIndex(modelDesc_, i);
         void *outputBuffer = nullptr;
         aclrtMalloc(&outputBuffer, modelOutputSize, ACL_MEM_MALLOC_NORMAL_ONLY);
         aclDataBuffer *outputData = aclCreateDataBuffer(outputBuffer, modelOutputSize);
         ret = aclmdlAddDatasetBuffer(outputDataset_, outputData);
     }
     ```

3. 输入数据处理       
   输入数据处理使用的是DVPP，主要为图片数据加载->JPEGD解码->VPC处理。伪码如下：
   ```
   // 图片加载
   ImageData image;
   std::ifstream binFile(imageFile, std::ifstream::binary);
   ...
   image.data.reset(binFileBufferData, [](uint8_t* p) { delete[](p); });
   image.size = binFileBufferLen;

   // 将Host的图片数据拷贝到Device
   acldvppMalloc(&buffer, image.size);
   aclError ret = aclrtMemcpy(...);
   imageDvpp.size = image.size;
   imageDvpp.data = SHARED_PTR_DVPP_BUF(buffer);

   // 获取图片宽高信息
   acldvppJpegGetImageInfo(...);

   // 根据不同芯片的对齐要求进行判断及对齐
   auto socVersion = aclrtGetSocName();
   if (strncmp(socVersion, "Ascend310P3", sizeof("Ascend310P3") - 1) == 0) {
       imageDvpp.width = ALIGN_UP2(image.width);
       imageDvpp.height = ALIGN_UP2(image.height);
       decodeOutWidthStride = ALIGN_UP64(image.width); // 64-byte alignment
       decodeOutHeightStride = ALIGN_UP16(image.height); // 16-byte alignment
   } else {
       ...
   }

   // 计算输出大小
   uint32_t decodeOutBufferSize = YUV420SP_SIZE(decodeOutWidthStride, decodeOutHeightStride);

   // 创建解码输出内存，并准备好输出图片描述中的数据
   acldvppMalloc(&decodeOutBufferDev_, decodeOutBufferSize);
   acldvppSetPicDescData(decodeOutputDesc_, decodeOutBufferDev_);
   ...

   // 执行解码
   acldvppJpegDecodeAsync(dvppChannelDesc_, reinterpret_cast<void *>(imageDvpp.data.get()),
                          imageDvpp.size, decodeOutputDesc_, stream_);
   ret = aclrtSynchronizeStream(stream_);

   // VPC（resize）输入数据准备
   yuvImage.width = ALIGN_UP2(imageDvpp.width);
   ...

   // 设置VPC输入图片描述
   acldvppSetPicDescData(vpcInputDesc_, yuvImage.data.get());  // set input desc
   ...

   // VPC（resize）输出数据准备
   int resizeOutWidth = ALIGN_UP2(modelWidth_);
   ...

   // 设置VPC输出图片描述
   acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev);   // set output desc
   ...

   // 执行VPC（resize）操作
   acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
                          vpcOutputDesc_, resizeConfig_, stream_);
   ret = aclrtSynchronizeStream(stream_);

   // 将预处理数据拷贝到device上准备推理
   std::shared_ptr<uint8_t> data = SHARED_PTR_DVPP_BUF(vpcOutBufferDev);
   ret = aclrtMemcpy(picDevBuffer_, devBufferSize_,
                      data.get(), devBufferSize_,
    ACL_MEMCPY_DEVICE_TO_DEVICE);
   ```


4. 模型推理      
   预处理结束后直接执行模型推理即可。
   ```
    // 执行推理
    aclError ret = aclmdlExecute(modelId_, inputDataset_, outputDataset_);
   ```

5. 数据后处理      
   数据后处理，是将模型推理的结果进行分析。
   ```
   // 从输出Dataset中获取实际数据
   void *outHostData = nullptr;
   float *outData = nullptr;
   size_t outputIndex = 0;
   aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(outputDataset_, outputIndex);
   void* data = aclGetDataBufferAddr(dataBuffer);
   uint32_t len = aclGetDataBufferSizeV2(dataBuffer);

   // 将数据拷贝到Host侧
   aclrtMallocHost(&outHostData, len);
   aclError ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
   outData = reinterpret_cast<float*>(outHostData);

   // 将输出数据报错到map对象中
   map<float, unsigned int, greater<float> > resultMap;
   for (unsigned int j = 0; j < len / sizeof(float); ++j) {
       resultMap[*outData] = j;
       outData++;
   }

   // 计算所有置信度的和
   double totalValue=0.0;
   for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
       totalValue += exp(it->first);
   }

   // 打印前5置信度的类别信息和归一化到[0,1]的置信度值
   int cnt = 0;
   for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
       // print top 5
       if (++cnt > 5) {
           break;
       }
       INFO_LOG("top %d: index[%d] value[%lf] class[%s]", cnt, it->second,
                exp(it->first) / totalValue, label[it->second].c_str());
   }
   ret = aclrtFreeHost(outHostData);
   outHostData = nullptr;
   outData = nullptr;
   if (ret != ACL_SUCCESS) {
       ERROR_LOG("aclrtFreeHost failed, errorCode is %d", ret);
       return FAILED;
   }
   ```

6. 资源释放      
   当程序运行结束后，需要将初始化的资源全部释放。
   ```
   // 释放图片描述
   ret = acldvppDestroyResizeConfig(resizeConfig_);
   ret = acldvppDestroyPicDesc(decodeOutputDesc_);
   ret = acldvppDestroyPicDesc(vpcInputDesc_);
   ret = acldvppDestroyPicDesc(vpcOutputDesc_);

   // 销毁通道及通道描述
   ret = acldvppDestroyChannel(dvppChannelDesc_);
   ret = acldvppDestroyChannelDesc(dvppChannelDesc_);

   // 销毁输入DataBuffer及Dataset
   for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(inputDataset_); ++i) {
       aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(inputDataset_, i);
       (void)aclDestroyDataBuffer(dataBuffer);
   }
   (void)aclmdlDestroyDataset(inputDataset_);
   inputDataset_ = nullptr;

   // 销毁输出DataBuffer及Dataset
   for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(outputDataset_); ++i) {
       aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(outputDataset_, i);
       void* data = aclGetDataBufferAddr(dataBuffer);
       (void)aclrtFree(data);
       (void)aclDestroyDataBuffer(dataBuffer);
   }
   (void)aclmdlDestroyDataset(outputDataset_);
   outputDataset_ = nullptr;

   // 销毁运行管理资源
   aclmdlDestroyDesc(modelDesc_);
   if (stream_ != nullptr) {
       ret = aclrtDestroyStream(stream_);
       if (ret != ACL_SUCCESS) {
           ERROR_LOG("aclrtDestroyStream failed, errorCode is %d", ret);
       }
       stream_ = nullptr;
   }
   if (context_ != nullptr) {
       ret = aclrtDestroyContext(context_);
       if (ret != ACL_SUCCESS) {
           ERROR_LOG("aclrtDestroyContext faild, errorCode is %d", ret);
       }
       context_ = nullptr;
   }
   ret = aclrtResetDevice(deviceId_);
   if (ret != ACL_SUCCESS) {
       ERROR_LOG("aclrtResetDevice failed, errorCode is %d", ret);
   }

   // AscendCL去初始化
   ret = aclFinalize();
   if (ret != ACL_SUCCESS) {
       ERROR_LOG("aclFinalize failed, errorCode is %d", ret);
   }
   ```

### 样例编译运行
1. CMakeList配置      
   样例编写完成后，就可以编写CMakeList并进行编译运行了。
   ```
   // 常规参数设置
   cmake_minimum_required(VERSION 3.5.1)
   project(sampleResnetDVPP)
   add_compile_options(-std=c++11)
   add_definitions(-DENABLE_DVPP_INTERFACE)
   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
   set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
   set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

   // AscendCL所需头文件环境变量DDK_PATH设置
   set(INC_PATH $ENV{DDK_PATH})
   ...
   endif ()

   // AscendCL所需库文件环境变量NPU_HOST_LIB设置
   set(LIB_PATH $ENV{NPU_HOST_LIB})
   ...
   endif ()

   // 头文件目录设置
   include_directories(
      ${INC_PATH}/runtime/include/
   )

   // 库文件目录设置
   link_directories(
       ${LIB_PATH}
   )

   // 应用代码链接
   add_executable(main
           sampleResnetDVPP.cpp)

   // 链接库文件
   ...
   target_link_libraries(main ascendcl acl_dvpp stdc++ dl rt)

   // 编译可执行文件生成
   install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
   ```
2. 编译脚本设置     
   编译脚本为scripts目录下的sample_build脚本。主要用于代码编译。
   ```
   ...
   // 判断om模型是否存在，更换模型时需要修改此处
   ret=`find ${ModelPath} -maxdepth 1 -name resnet50.om 2> /dev/null`
   ...
   // 执行build函数，主要使用make进行代码编译，无需修改
   build
   ...
   ```
3. 运行脚本设置     
   运行脚本为scripts目录下的sample_run脚本。主要用于代码运行。
   ```
   ...
   // 执行样例命令设置，样例变化时需要修改此处
   running_command="./main"
   ...
   // 运行
   ${running_command}
   ...
   ```
### <a name="step_5"></a>样例及场景介绍
以下提供了本章节介绍的样例，开发者可以根据以下样例中的readme进行调测运行，再根据源码理解本专题的知识点。
| 目录  | 场景  |
|---|---|
| [sampleResnetDVPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetDVPP)  | 使用DVPP加速预处理网络输入，并通过模型转换使能静态AIPP功能，使能AIPP功能后，YUV420SP_U8格式图片转化为RGB，然后减均值和归一化操作，并将该信息固化到转换后的离线模型中，对ResNet50网络执行推理，最终对输入的图片进行分类并且给出TOP5类别置信度和相应的类别信息。  |