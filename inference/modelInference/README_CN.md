## 模型推理样例说明

以下示例显示如何使用AscendCL同时突出显示不同的功能

<!--注：AscendCL样例仅用于说明目的，不用作生产质量代码的示例-->

| Title  | AscendCL Samples Name  | Description  |
|---|---|---|
| “Hello World” For AscendCL | [sampleResnetQuickStart](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetQuickStart)  | 演示使用AscendCL执行模型推理一般步骤的基础示例应用，以ResNet50网络模型为例进行模型推理，实现对物体进行分类，并给出TOP5类别置信度和相应类别信息。  |
| Enable AIPP And Running Inference | [sampleResnetAIPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetAIPP)  | 以ResNet50网络模型为例，演示如何通过模型转换使能静态AIPP功能，使能AIPP功能后，若实际提供给模型推理的测试图片不满足要求（包括图片格式，图片尺寸等），经过模型转换后，会输出满足模型要求的图片，并将该信息固化到转换后的离线模型中（模型转换后AIPP功能会以aipp算子形式插入离线模型中），图片分类应用。 |
| Image Classification with DVPP | [sampleResnetDVPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetDVPP)  | 使用DVPP加速预处理网络输入，并通过模型转换使能静态AIPP功能，使能AIPP功能后，YUV420SP_U8格式图片转化为RGB，然后减均值和归一化操作，并将该信息固化到转换后的离线模型中，对ResNet50网络执行推理，最终对输入的图片进行分类并且给出TOP5类别置信度和相应的类别信息。 |
| Object Detection with Acllite| [sampleYOLOV7](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7)  | 以YOLOV7网络模型为例，使能Acllite对图片进行预处理，并通过模型转换使能静态AIPP功能，使能AIPP功能后，YUV420SP_U8格式图片转化为RGB，然后减均值和归一化操作，并将该信息固化到转换后的离线模型中，对YOLOV7网络执行推理，对图片进行物体检测和分类，并给出标定框和类别置信度。 |
| Image Classification with DVPP for rtsp| [sampleResnetRtsp](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetRtsp)  | 使用live555创建rtsp流，作为样例的输入数据，使能Acllite解码rtsp，缩放图片，使用ResNet50网络执行推理，最终对输入的rtsp流的每帧图片进行分类并且给出TOP1类别置信度和相应的类别信息。 |
| Car Detection And Color Classification | [sampleCarColor](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleCarColor)  | 以YOLOV7网络模型和颜色分类模型进行串接推理，先用YOLOV7模型进行目标检测，获取目标位置，并将目标剪切下来送入颜色分类模型进行分类，并得到颜色分类结果，最后将类别以及颜色信息标注在图片相应位置并保存。 |
| Object Detection with MultiInput | [sampleYOLOV7MultiInput](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7MultiInput)  | 使用yolov7模型对输入数据进行预测推理，推理检测出图片/视频中所有可检测物体，并将推理结果打印到输出上，是一个是基于多路、多线程方案实现的高性能案例，通过多卡并行处理多路数的数据并输出，支持多种输入输出。 |
| Crowd Counting | [sampleCrowdCounting](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleCrowdCounting)  | 使用DVPP加速预处理网络输入，并通过模型转换使能静态AIPP功能，对CrowdCounting网络执行推理，最终输出带有统计人物数量以及带有颜色标记的jpg图片。 |
| YOLO With NMS ONNX | [sampleYOLOV7NMSONNX](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleYOLOV7NMSONNX)  | 使用yolov7模型对输入图片进行预测推理，后处理使用onnx构建cann的后处理算子模型进行加速处理，其中用到了两个cann算子进行图构建，包括YoloPreDetection，和YoloV5DetectionOutput算子，进行物体检测。在图片上给出物体标注框，类别以及置信度，并将结果打印到输出图片上。 |


## 单个应用样例目录结构说明
以[sampleResnetDVPP](https://github.com/Ascend/samples/tree/master/inference/modelInference/sampleResnetDVPP)样例为例：

```
├── data
│   ├── dog1_1024_683.jpg          // 测试图片

├── model           
│   ├── resnet50.onnx              // ResNet-50网络的模型文件（*.onnx） 
│   ├── aipp.cfg                   // aipp配置文件

├── script                         
│   ├── sample_build.sh            // 样例编译脚本       
│   ├── sample_run.sh              // 样例运行脚本    
                          
├── src                            
│   ├── CMakeLists.txt             // 编译脚本
│   ├── sampleResnetDVPP.cpp       // 图片分类功能的实现文件        

├── python                         // 存放当前应用样例的python代码实现

├── cppAcllite                     // 存放当前应用样例使用acllite的C++代码实现

├── pyAcllite                      // 存放当前应用样例使用acllite的python代码实现
       
```
