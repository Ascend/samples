中文|[English](./README.md)
# AscendCL概述

### <a name="step_1"></a> 本章学习目标
- 了解AscendCL的基本概念与功能架构
- 了解AscendCL接口的基础调用流程并学会如何运行

### <a name="step_2"></a>芯片介绍
- Ascend310系列芯片：面向推理场景，搭载该芯片的产品有推理卡、Atlas 200DK等。常用的ails就是使用的Atlas 300推理卡。
- Ascend910系列芯片：面向训练场景，搭载该芯片的产品有训练卡、训练服务器等。

### <a name="step_3"></a>逻辑架构
**AscendCL（Ascend Computing Language）**：昇腾计算语言，是一套用于在昇腾平台上开发深度神经网络推理应用的C语言API库。       
![AscendCL逻辑架构](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/AscendCL%E6%9E%B6%E6%9E%84.jpg)    
如上图，AscendCL提供了模型推理，算子调用等一系列API，实现利用昇腾硬件计算资源、在昇腾CANN平台上进行深度学习推理计算、图形图像预处理、单算子加速计算等能力。   
- **AscendCL的应用场景**
  1. 开发应用：用户可以直接调用AscendCL提供的接口开发图片分类应用、目标识别应用等。
  2. 供第三方框架调用：用户可以通过第三方框架调用AscendCL接口，以便使用昇腾AI处理器的计算能力。
  3. 供第三方开发lib库：用户还可以使用AscendCL封装实现第三方lib库，以便提供昇腾AI处理器的运行管理、资源管理等能力。
- **AscendCL的优势**
  1. 高度抽象：算子编译、加载、执行的API归一，相比每个算子一个API，AscendCL大幅减少API数量，降低复杂度。
  2. 向后兼容：AscendCL具备向后兼容，确保软件升级后，基于旧版本编译的程序依然可以在新版本上运行。
  3. 零感知芯片：一套AscendCL接口可以实现应用代码统一，多款昇腾AI处理器无差异。

### <a name="step_4"></a>基本概念解析
**基本概念**
- **Host**：指与Device相连接的x86服务器、ARM服务器，会利用Device提供的NN（Neural-Network）计算能力，完成业务。
- **Device**：指安装了芯片的硬件设备，利用PCIe接口与Host侧连接，为Host提供NN计算能力。若存在多个Device，多个Device之间的内存资源不能共享。
- **Context**：作为容器，管理了所有对象（包括Stream、Event、设备内存等）的生命周期。不同Context的Stream、不同Context的Event是完全隔离的，无法建立同步等待关系。
- **Stream**：用于维护一些异步操作的执行顺序，确保按照应用程序中的代码调用顺序在Device上执行。基于Stream的kernel执行和数据传输能够实现Host运算操作、Host与Device间的数据传输、Device内的运算并行。
- **同步/异步**：同步、异步是站在调用者和执行者的角度，在当前场景下，若在Host调用接口后不等待Device执行完成再返回，则表示Host的调度是异步的；若在Host调用接口后需等待Device执行完成再返回，则表示Host的调度是同步的。

**概念解析**
1. **Host&Device**     
典型场景如，将Atals 300I推理卡插入推理服务器（或个人PC）的主机中，此时程序的运行均在CPU侧进行控制，当需要进行专用计算（模型推理等）时，将CPU侧内存数据传输到NPU侧内存中，NPU侧完成专用计算后将数据回传至CPU侧。       
![Host及Device解析](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/Host%E5%8F%8ADevice%E8%A7%A3%E6%9E%90.jpg)  
那么CPU及内存所在这一侧，就是**Host**侧；而NPU及NPU上的内存这一侧，就是**Device**侧。
 
2. **Device、Context和Stream**     
在AI应用开发过程中，需要指定执行专用计算的Device；并且需要申请Context，用于管理Device上对象的生命周期；最后，需要申请Stream，用于在Device上保序的执行任务。我们统称Device、Context和Stream为**运行管理资源**。
![运行管理资源](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E8%BF%90%E8%A1%8C%E7%AE%A1%E7%90%86%E8%B5%84%E6%BA%90.jpg)  

3. **同步/异步**      
**同步**：当前线程发起一个方法调用，然后阻塞在调用点等待被调用方法执行完毕返回，再继续向下走。  
**异步**：调用者向执行者下发一个任务之后，不等待任务执行完，而是立即返回往下走，暂且不管这个任务是否执行完毕；后续再通过同步接口等待异步操作完成。   
![同步异步](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E5%90%8C%E6%AD%A5%E5%BC%82%E6%AD%A5.jpg) 

在初步的学习过程中，仅需要清楚以下两点。   
- 由于异步操作都是需要通过Stream进行操作保序的，所以在调用异步接口时，都需要传入指定的stream。也就是说，**如果要使用异步接口，那么在当前Context下一定有显示创建的stream。可以在程序开始时显示创建，也可以在异步操作前显示创建**。    
- 调用异步接口后，需要使用同步等待接口（aclrtSynchronizeStream），等待该stream上的异步操作执行完毕。    

### <a name="step_3"></a>接口调用流程与API解析
将AscendCL的应用高度抽象后，可以得到以下流程。
![接口调用流程](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E6%8E%A5%E5%8F%A3%E8%B0%83%E7%94%A8%E6%B5%81%E7%A8%8B.jpg)      
具体业务执行及API会在具体业务中介绍，本章主要介绍通用步骤的调用流程及API。 
 
1. AscendCL初始化   
   使用AscendCL接口开发应用时，必须先初始化AscendCL ，否则可能会导致后续系统内部资源初始化出错，进而导致其它业务异常，接口如下：   
   ```
   aclError aclInit(const char *configPath)
   ```
   参数configPath为json格式的配置文件路径，可以通过特定配置，支持跟推理相关的可配置项（如性能相关的采集信息配置、dump数据采集配置项等）。    
   初步学习过程中，可以向aclInit接口中传入NULL，或者可将配置文件配置为空json串（即配置文件中只有{}）。以上场景调用伪码如下：   
   ```
   // 配置json文件，此处代表acl.json文件在编译出来的可执行文件的../src目录下
   const char *aclConfigPath = "../src/acl.json";
   aclError ret = aclInit(aclConfigPath);
   
   // 不配置json文件
   aclError ret = aclInit(NULL);
   ```

2. 运行管理资源申请
   AscendCL初始化后，需要按顺序依次申请如下运行管理资源：Device、Context、Stream，确保可以使用这些资源执行运算、管理任务。接口如下：
   ```
   aclError aclrtSetDevice(int32_t deviceId)
   aclError aclrtCreateContext(aclrtContext *context, int32_t deviceId)
   aclError aclrtCreateStream(aclrtStream *stream)
   ```
   在申请运行管理资源时，Context、Stream支持隐式创建和显式创建两种申请方式。     
   初步学习过程中，建议使用显式创建；这样能提高程序的可读性与可维护性，且执行异步接口时无需额外创建Stream。调用伪码如下：
   ```
   // 一般来说，昇腾设备上只有一个Device，那deviceId一般为0；有多个Device，一般都是从0开始往上排。
   uint32_t deviceId = 0;
   aclrtContext context;
   aclrtStream stream;
   ret = aclrtSetDevice(deviceId);
   // 一个ctx一定属于一个唯一的device。
   ret = aclrtCreateContext(&context, deviceId);
   // 创建完Context后，才能创建stream；context管理了stream的生命周期。
   ret = aclrtCreateStream(&stream);
   ```

3. 运行管理资源释放
   当所有数据处理都结束后，需要按顺序依次释放运行管理资源：Stream、Context、Device。接口如下：
   ```
   aclError aclrtDestroyStream(aclrtStream stream)
   aclError aclrtDestroyContext(aclrtContext context)
   aclError aclrtResetDevice(int32_t deviceId)
   ```
   显示创建的运行管理资源，必须要调用对应的Destory接口进行资源销毁。隐式创建的则会自动销毁。     
   资源释放时必须要按照创建时相反的顺序进行，否则会释放失败。调用伪码如下：
   ```
   // 先释放stream资源，如果有多个显示创建的stream要逐一释放
   ret = aclrtDestroyStream(stream);
   // 释放context资源
   ret = aclrtDestroyContext(context);
   // 释放device资源
   ret = aclrtResetDevice(deviceId);
   ```

4. AscendCL去初始化
   有初始化就有去初始化，在确定完成了AscendCL的所有调用（包括运行管理资源释放完成）之后，或者进程退出之前，需调用AscendCL接口实现AscendCL去初始化。接口如下：
   ```
   aclError aclFinalize()
   ```
   去初始化接口调用伪码如下：
   ```
   ret = aclFinalize();
   ```

初步学习过程中，这四步的调用一般都是固定的，在应用开发过程中可以固定使用。总体伪码如下所示：
```
// 调用acl接口需要使用acl.h头文件
#include "acl/acl.h"
uint32_t deviceId = 0;
aclrtContext context;
aclrtStream stream;
const char *aclConfigPath = "../src/acl.json";
aclError ret = aclInit(aclConfigPath);
ret = aclrtSetDevice(deviceId);
ret = aclrtCreateContext(&context, deviceId);
ret = aclrtCreateStream(&stream);
/*
 * 业务执行
 */
ret = aclrtDestroyStream(stream);
ret = aclrtDestroyContext(context);
ret = aclrtResetDevice(deviceId);
ret = aclFinalize();
```
### <a name="step_7"></a>样例及场景介绍
为了帮助大家更好的理解，这里提供了样例，开发者可以根据以下样例中的readme进行调测运行，再根据源码理解本专题的知识点。
| 目录  | 场景  |
|---|---|
| [ACL_HELLO_WORLD](https://github.com/Ascend/samples/tree/master/inference/ACLHelloWorld)  | 一个简单样例快速理解AscendCL基础概念。  |