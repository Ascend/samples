中文|[English](./README.md)
# 快速入门

### <a name="step_1"></a> 本章学习目标
- 了解AscendCL的推理样例架构
- 学会编译并运行调用了AscendCL接口的推理代码

### <a name="step_2"></a> 总体说明
本章节将通过一个简单的[图片分类应用](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetQuickStart)说明使用AscendCL接口（c语言接口）开发应用的基本过程以及开发过程中涉及的关键概念。     
图片分类应用，即通过AI模型推理获得图片所属的分类，并进行展示。
![图片分类应用说明](https://www.hiascend.com/doc_center/source/zh/CANNCommunityEdition/600alpha003/infacldevg/aclcppdevg/figure/zh-cn_image_0000001417444636.png)   
在快速入门中，使用的是已经训练好的pytorch框架的ResNet-50开源模型。该模型的基本介绍如下：
- 输入数据：RGB格式、224*224分辨率的输入图片。
- 输出数据：图片的类别标签及其对应置信度。

### <a name="step_3"></a>开发流程
AscendCL推理应用的开发流程如下所示：   
![图片分类应用说明](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/data/growthpath_pic/%E5%BC%80%E5%8F%91%E6%B5%81%E7%A8%8B.jpg#pic_left)    
本章节将会重点介绍总体架构及编译运行的基本原理，旨在了解整体的逻辑，后续再深入学习，了解其它细节。   
样例实际执行请参考[样例及场景介绍]()中的样例readme。

### <a name="step_4"></a>代码目录说明
代码目录如下所示，其中data目录及model目录刚下载时为空目录。在样例执行期间需要额外操作。
```
sampleResnetQuickStart
├── data
│   └─── dog1_1024_683.jpg            // 测试图片，程序编译过程中下载
├── model
│   └── resnet50.onnx                 // 已经训练好的基于pytorch的ResNet-50网络的模型文件，按照readme下载                 
├── script
│   ├── sample_build.sh               // 样例编译脚本，完成图片准备及样例编译工作
│   └── sample_run.sh                 // 样例运行脚本，执行样例
├── src
│   ├── CMakeLists.txt                // cmake编译脚本
└   └── sampleResnetQuickStart.cpp    // 主函数，图片分类功能的实现文件
```
### <a name="step_5"></a>准备模型
使用AscendCL接口进行离线样例开发时，都需要先将训练好的模型转换为昇腾AI处理器能识别的*.om模型文件。  
快速入门样例中，模型转换的命令如下：
```
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/resnet50/resnet50.onnx
atc --model=resnet50.onnx --framework=5 --output=resnet50 --input_shape="actual_input_1:1,3,224,224"  --soc_version=Ascend310
```
其中参数说明如下：
- **--model**：ResNet-50网络的模型文件路径。
- **--framework**：原始框架类型。5表示ONNX。
- **--output**：resnet50.om模型文件的路径。请注意，记录保存该om模型文件的路径，后续开发应用时需要使用。
- **--soc_version**：昇腾AI处理器的版本。进入“CANN软件安装目录/compiler/data/platform_config”目录，".ini"文件的文件名即为昇腾AI处理器的版本，请根据实际情况选择。

模型转换完成后，需要将转换出来的模型放到样例中的model目录下。因为源码中加载模型时，固定了模型文件路径为../model，所以模型必须位于可执行文件的../model目录下，否则程序运行时会报错找不到模型。
    
### <a name="step_6"></a>应用模块说明
下面将介绍应用代码中各个模块的具体功能，了解模块功能后再结合后续知识便能够自行解析推理应用了。
1. main函数定义    
   程序的入口函数为main函数，按照[开发流程](#step_3)中的过程详解，串联整个应用的代码逻辑。
   ```
   int main()
   {	
        // 定义模型路径
        const char* modelPath = "../model/resnet50.om";
        // 定义输入图片路径
        const string imagePath = "../data";
        // 指定device
        int32_t device=0;
        // 使用SampleResnetQuickStart类新建SampleResnet对象，将device和modelPath信息传入类中初始化
        SampleResnetQuickStart sampleResnet(device, modelPath, modelWidth, modelHeight);
        // 执行资源初始化
        Result ret = SampleResnet.InitResource();
        // 输入数据处理
        ret = sampleResnet.ProcessInput(fileName);
        // 模型推理
        ret = SampleResnet.Inference();
        // 根据推理结果进行解析
        ret = SampleResnet.GetResult();
        return SUCCESS;
   }
   ```
   了解总体的代码逻辑后，接下来开始介绍其余各个模块。

2. 包含头文件及全局定义   
   源文件的开头需要引入头文件并进行一些全局定义，包括：
   - 依赖头文件，包括AscendCL的头文件、C或C++标准库的头文件、opencv的头文件。
   - 引入命名空间，包括标准命名空间和opencv的命名空间。
   - 结构体定义，代码中需要使用的结构体定义。
   - 类的定义，定义推理代码的类。

   伪码如下所示，具体代码请参考原始样例。   
   ```
   // 头文件引入，其中acl是相对路径，要和后面编译文件中的头文件路径结合
   #include <iostream>
   #include "acl/acl.h"
   ...
   // 命名空间引入
   using namespace cv;
   using namespace std;
   // 返回值结构体定义
   typedef enum Result {
	...
   } Result;
   // 定义类
   class SampleResnetQuickStart {
	...
   };
   ``` 
3. 资源初始化   
   资源初始化过程中，需要初始化如下数据：  
   - AscendCL初始化。
   - 运行管理资源申请。
   - 模型加载。
   - 模型输入输出内存申请。  

   伪码如下所示，具体代码请参考原始样例。
   ```
   Result SampleResnetQuickStart::InitResource()
   {
        // AscendCL初始化
        const char *aclConfigPath = "";
	aclError ret = aclInit(aclConfigPath);
        // 运行管理资源申请。
        ret = aclrtSetDevice(deviceId_);
        ...
        // 模型加载
        ret = aclmdlLoadFromFile(modelPath_, &modelId_);
        ...
        // 模型输入输出内存申请。
        inputDataset_ = aclmdlCreateDataset();
        outputDataset_ = aclmdlCreateDataset();
        ...
        return SUCCESS;
   }
   ```
4. 输入数据处理    
   资源初始化完毕后，就需要读入输入数据并进行处理，本样例主要进行如下处理：   
   - 使用opencv读取图片
   - 使用opencv将图片缩放到模型要求输入大小
   - 使用opencv进行色域转换，将BGR格式的读入图片转换成模型需要的RGB格式  

   伪码如下所示，具体代码请参考原始样例。  
   ```
   Result SampleResnetQuickStart::ProcessInput(const string testImgPath) {
   {
        // 读取图片
        Mat srcImage = imread(testImgPath);
        // 图片缩放
        resize(srcImage, resizedImage, Size(modelWidth_, modelHeight_));
        // 色域转换
        ……
        ……
        picDevBuffer = nchwImage_.data;
        ...
   }
   ```
5. 模型推理     
   输入数据处理完成后，就可以进行模型推理了，模型推理包括如下步骤：   
   - 将输入数据拷贝到device侧
   - 执行模型推理

   伪码如下所示，具体代码请参考原始样例。 
   ```
   Result SampleResnetQuickStart::Inference()
   {
        // 输入数据拷贝
        aclError aclRet = aclrtMemcpy(picDevBuffer_, devBufferSize_, nchwImage_, devBufferSize_, ACL_MEMCPY_HOST_TO_DEVICE);
        // 模型推理
        aclError ret = aclmdlExecute(modelId_, inputDataset_, outputDataset_);
        ...
   }
   ```
6. 输出数据处理     
   模型推理结束后，需要对推理后输出的数据进行分析，主要包含如下步骤：   
   - 获取所有输出的数据内容
   - 将数据拷贝到Host侧
   - 打印置信度

   伪码如下所示，具体代码请参考原始样例。 
   ```
   Result SampleResnetQuickStart::GetResult() {
       ……
       // 获取输出的databuffer
       aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(outputDataset_, outputIndex);
       // 获取databuffer中的数据地址
       void* data = aclGetDataBufferAddr(dataBuffer);
       uint32_t len = aclGetDataBufferSizeV2(dataBuffer);
       ...
       // 将数据拷贝到Host侧
       aclrtMallocHost(&outHostData, len);
       aclError ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
       // 打印置信度
       ...
       for (auto it = resultMap.begin(); it != resultMap.end(); ++it)
       ...
   }
   ```
7. 资源释放     
   推理结束后，需要将所有资源进行释放，包括以下步骤：    
   - 模型输入数据释放
   - 模型输出数据释放
   - 运行管理资源释放
   - AscendCL去初始化

   伪码如下所示，具体代码请参考原始样例。 
   ```
   void SampleResnetQuickStart::ReleaseResource()
   {
        // 模型输入数据释放
	...
	(void)aclDestroyDataBuffer(dataBuffer);
	(void)aclmdlDestroyDataset(inputDataset);
        // 模型输出数据释放
	...
	(void)aclDestroyDataBuffer(dataBuffer);
	(void)aclmdlDestroyDataset(outputDataset);
	...
        // 运行管理资源释放
	ret = aclrtResetDevice(deviceId_);
        // AscendCL去初始化
	ret = aclFinalize();
   }
   ```

### <a name="step_4"></a>编译文件详解
C++代码编译时需要使用CMakeLists.txt文件，本章就通用的CMakeList.txt编译文件进行编译配置的说明。
1. 通用配置    
   CMakeList中可复用的一些通用配置，除工程名需要修改以外，其他内容在新建样例时均可直接复制。    
   ```
   # cmake最低版本限制，如果cmake低于该版本，则编译直接报错
   cmake_minimum_required(VERSION 3.5.1)

   # 工程名设置，需要修改为当前样例的名称
   project(sampleResnetQuickStart)

   # 添加编译选项，标准为c++11
   add_compile_options(-std=c++11)

   # 设置指定可执行文件的输出目录
   set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../../out")
   
   # 设置Debug及Release编译时的编译选项
   set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
   set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")
   ```
2. 环境变量读取与设置    
   代码编译过程中，需要读取环境变量，以便于后续的SO及头文件的加载。   
   其中环境变量DDK_PATH设置的是CANN软件安装目录。   
   环境变量NPU_HOST_LIB设置的是编译对应的so库目录。   
   ```
   # 将环境变量DDK_PATH值设置为变量INC_PATH值
   set(INC_PATH $ENV{DDK_PATH})

   # 如果未定义DDK_PATH环境变量，将给变量INC_PATH设置默认值
   if (NOT DEFINED ENV{DDK_PATH})
        set(INC_PATH "/usr/local/Ascend/ascend-toolkit/latest")
        message(STATUS "set default INC_PATH: ${INC_PATH}")
    else()
        message(STATUS "set INC_PATH: ${INC_PATH}")
    endif ()

   # 将环境变量NPU_HOST_LIB值设置为变量LIB_PATH 值
   set(LIB_PATH $ENV{NPU_HOST_LIB})

   # 如果未定义NPU_HOST_LIB环境变量，将给变量LIB_PATH 设置默认值
   if (NOT DEFINED ENV{NPU_HOST_LIB})
        set(LIB_PATH "/usr/local/Ascend/ascend-toolkit/lastest/runtime/lib64/stub")
        message(STATUS "set default LIB_PATH: ${LIB_PATH}")
    else()
        message(STATUS "set LIB_PATH: ${LIB_PATH}")
    endif ()
   ```
3. 编译配置    
   通过环境变量配置后，就可以获取到样例所需的SO和头文件的路径了，接下来就需要根据这些路径进行编译配置。如下所示：  
   ```
   # 添加头文件对应目录，代码中添加的所有头文件必须全部包含在include_directories中定义的目录下
   include_directories(
       ${INC_PATH}/runtime/include/
   )

   # 仿真测试设置的编译选项，可不用添加，也可直接复制
   if(target STREQUAL "Simulator_Function")
       add_compile_options(-DFUNC_SIM)
   endif()

   # 添加库文件所在的目录
   link_directories(
       ${LIB_PATH}
   )

   # 修改可执行文件的名称（例如：main）、添加*.cpp文件所在的目录
   add_executable(main
        sampleResnetQuickStart.cpp)

   # target_link_libraries：修改可执行文件的名称（与 add_executable中保持一致），添加可执行文件依赖的库文件。
   if(target STREQUAL "Simulator_Function")
        target_link_libraries(main funcsim)
    else()
        target_link_libraries(main ascendcl stdc++ opencv_core opencv_imgproc opencv_imgcodecs dl rt)
    endif()

   # 修改可执行文件的名称（与 add_executable中保持一致），并生成到前面设置的目录中。
   install(TARGETS main DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
   ```
大部分样例中的CMakeList.txt都是上面介绍的结构，开发样例的过程中CMakeList中的大部分内容都是可以直接参考使用的。

### <a name="step_5"></a>编译及运行应用
具备源码和CMake文件后，就可以进行代码的编译与运行。一般来说编译的命令都如下所示：   
```
# 执行cmake编译代码，其中，“../../../src”表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改；
# -DCMAKE_CXX_COMPILER参数是用来指定编译器的，-DCMAKE_SKIP_RPATH参数是代表不将编译时链接的库文件路径添加到编译生成的可执行文件中。
cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
# 执行make命令生成可执行文件。
make
```
运行可执行文件指令一般为：   
```
cd XXX/out
./main
```
具体的编译运行步骤可参考课程附带样例的Readme。  

### <a name="step_7"></a>样例及场景介绍
为了帮助大家更好的理解，这里提供了样例，开发者可以根据以下样例中的readme进行调测运行，再根据源码理解本专题的知识点。
| 目录  | 场景  |
|---|---|
| [sampleResnetQuickStart](https://github.com/Ascend/samples/blob/master/inference/modelInference/sampleResnetQuickStart)  | 演示使用AscendCL执行模型推理一般步骤的基础示例应用，以ResNet50网络模型为例进行模型推理，实现对物体进行分类，并给出TOP5类别置信度和相应类别信息  |