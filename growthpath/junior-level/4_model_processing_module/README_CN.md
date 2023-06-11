中文|[English](./README.md)
# 模型离线推理

### 章节目标
- 了解模型离线推理的步骤
- 学会调用AscendCL相关接口进行模型推理

### 总体说明
本章节将根据[图片分类应用](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetQuickStart)中的模型离线推理介绍实际使用场景。   
模型离线推理主要是使用已经转好的om对输入图片进行推理，主要步骤如下图所示：     
![模型离线推理步骤](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E6%A8%A1%E5%9E%8B%E7%A6%BB%E7%BA%BF%E6%8E%A8%E7%90%86%E6%AD%A5%E9%AA%A4.png)     
各步骤解析如下：    
- **Host&Device内存管理与数据传输**：Host&Device上的内存申请与释放，内存间的相互拷贝；在整个样例执行过程中都会涉及。    
- **模型加载**：将离线的om文件加载到Device上；在样例的资源初始化模块中进行。
- **模型输入输出准备**：根据离线om的输入输出，在Device上申请好模型的输入输出内存；在样例的资源初始化模块中进行。
- **执行推理**：当模型的输入内存获取到有效数据后，便可以调用AscendCL接口执行模型推理，推理完成后结果生成到输出内存中；在样例的模型推理模块中进行。
- **输出解析**：使用AscendCL接口，将模型输出数据从特定格式中解析出来；在输出数据处理模块中进行。
下面将按步骤进行说明。   

### Host&Device内存管理与数据传输
代码中加载输入数据时，需要申请Host内存进行存储，当输入数据处理完毕后，需要将处理完成的数据从Host内存拷贝到Device的模型输入内存中，以便于Device进行模型推理的专用计算。          
以上就是Host&Device内存管理与数据传输的典型场景。对于Host&Device内存管理与数据传输来说，实际上就是：       
- **Host内存管理**：Host侧内存申请与释放。    
- **Device内存管理**：Device侧内存申请与释放。    
- **Host&Device数据传输**：Host和Device上的内存拷贝。  
 
除此以外，内存管理中还提供了一些其它功能。   
- **内存初始化**：对申请出来的Host或Device内存进行初始化。   
- **Device内存查询**：查询Deivce上有多少内存。  

接下来分别讲解这些功能：   
- Host侧内存申请与释放。  
  Host侧内存申请与释放接口的使用方式比较简单，函数原型如下：    
  ```
  aclError aclrtMallocHost(void **hostPtr, size_t size);
  aclError aclrtFreeHost(void *hostPtr);
  ```   
  显然，aclrtMallocHost是内存申请接口，aclrtFreeHost是其对应的释放接口；调用伪代码如下。
  ```
  ...
  void *hostInput = nullptr;
  int64_t size_input = 256;
  ret = aclrtMallocHost(&hostInput, size_input);
  if (hostInput != nullptr) {
    ret = aclrtFreeHost(hostInput);
  }
  ...
  ```
- Device侧内存申请与释放。  
  Device侧内存申请与释放接口和Host侧的很相似，函数原型如下：   
  ```
  aclError aclrtMalloc(void **devPtr, size_t size, aclrtMemMallocPolicy policy);
  aclError aclrtFree(void *devPtr);
  ```    
  申请内存的接口多了个参数：policy，指明申请内存的策略。当前一共有三种策略可选：
  - ACL_MEM_MALLOC_HUGE_FIRST：当申请的内存小于等于1M时，即使使用该内存分配规则，也是申请普通页的内存。当申请的内存大于1M时，优先申请大页内存，如果大页内存不够，则使用普通页的内存
  - ACL_MEM_MALLOC_HUGE_ONLY：仅申请大页，如果大页内存不够，则返回错误
  - ACL_MEM_MALLOC_NORMAL_ONLY：仅申请普通页
  
  调用伪代码如下：   
  ```
  ...
  void *devInput = nullptr;
  size_input = 256;
  ret = aclrtMalloc(&devInput, size_input, ACL_MEM_MALLOC_HUGE_FIRST);
  if (devInput != nullptr) {
    ret = aclrtFree(devInput);
  }
  ...
  ```  
  **注意**：申请和释放两个接口的配套关系和Host类似，aclrtMalloc和aclrtFree要成对出现;用aclrtMalloc申请出来的内存也是对齐过的，对齐方式和Host侧一致。  

- 内存初始化             
  刚申请出来的内存，里边的数据是随机的，有时需要对其进行统一的初始化，此时可以使用这个接口：
  ```
  aclError aclrtMemset(void *devPtr, size_t maxCount, int32_t value, size_t count);
  ```
  其参数如下所示；
  - devPtr：内存的起始地址，Host或者Device上的都可以，系统会根据地址自动判断内存位置
  - maxCount：内存的最大长度，单位byte
  - value：设置的值 需要设置为指定值的内存长度，单位Byte

  以Device侧内存初始化的调用伪码如下：
  ```
  ...
  void *devInput = nullptr;
  size_input = 256;
  ret = aclrtMalloc(&devInput, size_input, ACL_MEM_MALLOC_HUGE_FIRST);
  if (devInput != nullptr) {
    ret = aclrtMemset(devInput, size_input, 1, size_input);
    ret = aclrtFree(devInput);
  }
  ...
  ```
- 数据传输    
  数据传输所使用的内存拷贝函数原型如下：
  ```
  aclError aclrtMemcpy(void *dst, size_t destMax, const void *src, size_t count, aclrtMemcpyKind kind);
  ```
  其参数如下所示；       
  - dst：目的地址
  - destMax：目的内存地址的最大内存长度，单位Byte
  - src：源地址
  - count：内存复制的长度，单位Byte
  - kind：内存复制的类型，预留参数，配置枚举值中的值无效，系统内部会根据源内存地址指针、目的内存地址指针判断是否可以将源地址的数据复制到目的地址，如果不可以，则系统会返回报错。

  其中的关键为kind参数，这里的kind实际上是一组枚举值，根据上述的拷贝方向填写对应的枚举即可，虽然该参数仅为预留参数，但是还是建议大家在开发应用的过程中还是需要将传输方向参数填写正确。这样既有利于代码的可读性，也有利于开发者在开发过程中理清数据流向。   
  枚举定义如下所示：    
  ```
  typedef enum aclrtMemcpyKind {
  ACL_MEMCPY_HOST_TO_HOST, // Host -> Host
  ACL_MEMCPY_HOST_TO_DEVICE, // Host -> Device
  ACL_MEMCPY_DEVICE_TO_HOST, // Device -> Host
  ACL_MEMCPY_DEVICE_TO_DEVICE, // Device -> Device
  } aclrtMemcpyKind;
  ```
  掌握了如上知识点后，再用一小段Host->Device的传输伪码来看下具体调用过程：
  ```
  ...
  hostInput = nullptr;
  devInput = nullptr;
  size_input = 256;
  ret = aclrtMallocHost(&hostInput, size_input);
  ret = aclrtMalloc(&devInput, size_input, ACL_MEM_MALLOC_HUGE_FIRST);
  ret = aclrtMemset(hostInput, size_input, 1, size_input);
  aclrtMemcpy(devInput, size_input, hostInput, size_input, ACL_MEMCPY_HOST_TO_DEVICE);
  ret = aclrtFreeHost(hostInput);
  ret = aclrtFree(devInput);
  ...
  ```
- Device内存查询              
  程序运行过程中，如何实时获取Device上有多少内存，以及多少可用内存呢？那不妨试试下面这个接口。
  ```
  aclError aclrtGetMemInfo(aclrtMemAttr attr, size_t *free, size_t *total)
  ```
  其中的attr参数指的是内存的类型，枚举定义如下所示： 
  ```
  typedef enum aclrtMemAttr {
      ACL_DDR_MEM, //DDR内存，DDR上所有大页内存+普通内存
      ACL_HBM_MEM, //HBM内存，HBM上所有大页内存+普通内存
      ACL_DDR_MEM_HUGE,  //DDR大页内存
      ACL_DDR_MEM_NORMAL,  //DDR普通内存
      ACL_HBM_MEM_HUGE,  //HBM大页内存
      ACL_HBM_MEM_NORMAL,  //HBM普通内存
      ACL_DDR_MEM_P2P_HUGE,  //DDR中用于Device间数据复制的大页内存
      ACL_DDR_MEM_P2P_NORMAL,  //DDR中用于Device间数据复制的普通内存
      ACL_HBM_MEM_P2P_HUGE,  //HBM中用于Device间数据复制的大页内存
      ACL_HBM_MEM_P2P_NORMAL,  //HBM中用于Device间数据复制的普通内存
  } aclrtMemAttr;
  ```
  这里提到了DDR和HBM，在这里，只需要知道Ascend910芯片中有HBM内存，在内存申请时会优先使用，使用完毕后再使用DDR内存；而Ascend310芯片中只有DDR内存。   
  所以调用时，只需要根据自己的场景查询所有内存即可。

### 模型加载
模型加载有多种方式，但是初步学习过程中，只需要掌握以下接口，从文件中加载模型即可。加载卸载模型接口如下：
```
aclError aclmdlLoadFromFile(const char *modelPath, uint32_t *modelId);
aclError aclmdlUnload(uint32_t modelId)
```
参数表中的modelPath是入参，指的是离线模型文件在磁盘上的路径；而modelId则是出参，模型加载进内存后，AscendCL会生成一个modelId，后续在分析、使用模型的时候会用到，每次加载模型生成的modelId都是不一样的， 在一个进程空间内，modelId会保持唯一。       
调用伪码如下所示：
```
...
const char *modelPath = "./XXX.om";
uint32_t modelId;
ret = aclmdlLoadFromFile(modelPath, &modelId);
aclmdlUnload(modelId);
...
```

### 模型输入输出准备
数据传输过程中，会将推理数据（比如图片）传递到Device上，但此时传递到Device上的图片还是裸数据流，这种数据流是没法直接送进模型进行推理的，所以在推理之前，要为模型准备独特的数据结构。 如下图所示：     
![模型离线推理步骤](https://r.huaweistatic.com/s/ascendstatic/lst/onlineExperiment/664598/3.png)     
每个模型都有输入和输出，而不同模型的输入/输出数量不尽相同。在AscendCL中，输入和输出的组织形式是类似的，这里以“输入”的数据结构作为例子来讲解：
1. 一个模型的所有输入抽象为一个“DataSet”对象
2. 每一个输入抽象为一个“DataBuffer”对象

比如一个模型有2个输入，其中第一个输入是若干张图片，第二个输入是每张图片的元数据等信息，那么在编程中需要这样做：   
1. 用第一个输入，所有图片，创建一个DataBuffer对象
2. 用第二个输入，图片的信息，创建另一个DataBuffer对象
3. 创建一个DataSet对象
4. 把第1/2步中创建的2个DataBuffer对象放到DataSet对象中
一个模型有且只有1个“输入DataSet”（数据集对象），里边包含所有的输入；而如果有多个输入的话，每个输入用一个“DataBuffer”来承载。 

组织输出数据结构时，和输入一样，也是一个DataSet，1-N个DataBuffer；    
但是还没进行推理，没有数据，怎么会有DataBuffer？     
其实在模型确定下来之后，基本上输出的个数和占用内存大小就已经完全确定了。AscenCL提供了“模型描述”系列接口帮助查询这些信息。     
AscendCL不支持推理过程中自动申请输出内存，一定要在调用推理接口之前先把输出内存、DataBuffer、DataSet准备好。

下面将分别介绍这些关键的接口和调用方法。

- 模型描述信息            
  模型加载完成后，会生成modelID，根据modelID，就可以调用以下接口获取到模型描述信息。
  ```
  //创建空的模型描述对象
  aclmdlDesc* aclmdlCreateDesc();
  //获取对应模型描述信息
  aclError aclmdlGetDesc(aclmdlDesc *modelDesc, uint32_t modelId);
  ```
  模型描述信息，顾名思义，就是模型的各个数据，包括输入输出个数、输入输出大小。可以通过以下接口获取：   
  ```
  //获取模型的输入个数
  size_t aclmdlGetNumInputs(aclmdlDesc *modelDesc)
  //获取每个输入的大小
  size_t aclmdlGetInputSizeByIndex(aclmdlDesc *modelDesc, size_t index)
  //获取模型的输出个数
  size_t aclmdlGetNumOutputs(aclmdlDesc *modelDesc)
  //获取每个输出的大小
  size_t aclmdlGetOutputSizeByIndex(aclmdlDesc *modelDesc, size_t index)
  ```
  
- 输入输出创建                                 
  创建DataSet及DateBuffer类型的数据接口如下所示：          
  ```
  aclmdlDataset *aclmdlCreateDataset()
  aclDataBuffer *aclCreateDataBuffer(void *data, size_t size)
  ```
  结合以上信息，可以通过如下伪码了解输入输出的构造过程，伪码中默认仅有一个输入输出。    
  ```
  ...
  modelDesc = aclmdlCreateDesc();
  aclError ret = aclmdlGetDesc(modelDesc, modelId);
  //创建输入，pictureDeviceData为输入数据，该伪码中默认已经存在
  inputDataSet = aclmdlCreateDataset();
  inputDataBuffer = aclCreateDataBuffer(pictureDeviceData, pictureDataSize);
  aclmdlAddDatasetBuffer(inputDataSet, inputDataBuffer);
  //创建输出，默认仅有一个输出
  outputDataSet = aclmdlCreateDataset();
  outputDataSize = aclmdlGetOutputSizeByIndex(modelDesc, 0);
  aclrtMalloc(&outputDeviceData, outputDataSize, ACL_MEM_MALLOC_HUGE_FIRST);
  outputDataBuffer = aclCreateDataBuffer(outputDeviceData, outputDataSize);
  aclmdlAddDatasetBuffer(outputDataSet, outputDataBuffer);
  ...
  ```

### 执行推理过程
准备好如下的数据：
- 模型的modelId
- 输入DataSet
- 输出DataSet
就可以直接调用以下接口进行推理了：
```
aclError aclmdlExecute(uint32_t modelId, const aclmdlDataset *input, aclmdlDataset *output)
```
推理完成后，便可以直接获取到推理前创建好的输出内存（如上伪码中的outputDeviceData）进行处理了。

### 输出解析
推理完成后，输出DataSet中的数据需要取出并传到Host后才能进行处理，AscendCL提供了一些API帮助处理。
- 模型输出为DateSet，需要调用以下接口，获取到所有DateBuffer：    
  ```
  aclDataBuffer* aclmdlGetDatasetBuffer(const aclmdlDataset *dataset, size_t index)
  ```
- 获取到每个DataBuffer后，需要调用以下接口，获取到每个DateBuffer中的内存地址：   
  ```
  void *aclGetDataBufferAddr(const aclDataBuffer *dataBuffer)
  ```
- 获取到每个DataBuffer后，需要调用以下接口，获取到每个DateBuffer中的数据的内存大小：   
  ```
  void *aclGetDataBufferSizeV2(const aclDataBuffer *dataBuffer)
  ```
  
调用伪代码如下所示
```
for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(output); ++i) {
    //获取每个输出的内存地址和内存大小
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output, i);
    void* data = aclGetDataBufferAddr(dataBuffer);
    size_t len = aclGetDataBufferSizeV2(dataBuffer);

    //获取到输出数据后，由用户自行编码，处理输出数据
    ......
}
```
此时，获取到的输出还在Device上，需要将输出data拷贝到Host后就可以进行输出数据处理了。

### <a name="step_5"></a>样例及场景介绍
以下提供了两个场景的样例，开发者可以根据以下样例中的readme进行调测运行，再根据源码理解本专题的知识点。
| 目录  | 场景  |
|---|---|
| [sampleResnetQuickStart](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetQuickStart)  | 使用opencv进行预处理，模型转换时不带AIPP的Resnet50分类样例  |
| [sampleResnetAIPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetAIPP)    | 使用opencv进行预处理，模型转换时使用AIPP的Resnet50分类样例  |
