中文|[English](README_EN.md)

# sample

#### 介绍

Ascend sample，请根据自己的需要进入对应文件夹获取应用，或者点击下面的说明链接选择需要的应用。


#### 链接地址

<details open><summary>common：sample运行依赖的第三方依赖及环境安装指导文档文件夹</summary><blockquote>

- [install_opencv](https://gitee.com/ascend/samples/tree/master/common/install_opencv)：opencv安装说明。
- [install_presenteragent](https://gitee.com/ascend/samples/tree/master/common/install_presenteragent)：presenteragent安装说明。
- [install_python3env](https://gitee.com/ascend/samples/tree/master/common/install_python3env)：python3环境安装说明。
</blockquote></details> 

<details open><summary>classification：基于googlenet的分类应用，输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，对预处理后的图像中的物体进行分类，最后通过opencv进行相应后处理。    
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，对预处理后的图像中的物体进行分类，最后通过opencv进行相应后处理。 
</blockquote></details>  

<details open><summary>classification_dynamicbatch
：基于googlenet的分类应用，输入为bin文件，输出为打印结果。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification_dynamicbatch/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用ifstream读取已对图像做好预处理的bin文件，在模型推理时，需调用AscendCL提供的接口设置模型推理时需使用的Batch数。对预处理后的文件进行推理，最后将推理结果打印屏幕上。    
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification_dynamicbatch/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用ifstream读取已对图像做好预处理的bin文件，在模型推理时，需调用AscendCL提供的接口设置模型推理时需使用的Batch数。对预处理后的文件进行推理，最后将推理结果打印屏幕上。 
</blockquote></details>  

<details open><summary>classification_multibatch
：基于googlenet的分类应用，输入为bin文件，输出为打印结果。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification_multibatch%20%20%20%20/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用ifstream读取已对图像做好预处理的bin文件，等输入数据满足多Batch的要求，申请Device上的内存存放多Batch的数据，对预处理后的文件进行推理，最后将推理结果打印屏幕上。    
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification_multibatch%20%20%20%20/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用ifstream读取已对图像做好预处理的bin文件，等输入数据满足多Batch的要求，申请Device上的内存存放多Batch的数据，对预处理后的文件进行推理，最后将推理结果打印屏幕上。 
</blockquote></details>  

<details open><summary>classification_video：基于googlenet的分类应用，输入为视频，输出为视频。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification_video/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv对视频帧进行预处理，对预处理后的视频帧中的物体进行分类，最后通过opencv进行相应后处理。   
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/classification_video/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv对视频帧进行预处理，对预处理后的视频帧中的物体进行分类，最后通过opencv进行相应后处理。   
</blockquote></details>


<details open><summary>colorization：基于alexnet的黑白图像上色应用，输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/colorization/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，对预处理后的图像进行色彩通道预测，最后通过opencv进行相应后处理。  
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/colorization/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，对预处理后的图像进行色彩通道预测，最后通过opencv进行相应后处理。
  
</blockquote></details>  

   
<details open><summary>colorization_video：基于alexnet的黑白图像上色应用，输入为视频，输出为视频。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/colorization_video/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv对视频帧进行预处理，对预处理后的视频帧进行色彩通道预测，最后通过opencv进行相应后处理。  
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/colorization_video/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv对视频帧进行预处理，对预处理后的视频帧进行色彩通道预测，最后通过opencv进行相应后处理。
</blockquote></details>


<details open><summary>objectdetection：基于yolov3的目标检测应用，输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，对预处理后的图像中的物体进行目标检测，最后通过opencv进行相应后处理。  
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，对预处理后的图像中的物体进行目标检测，最后通过opencv进行相应后处理。  
</blockquote></details>

<details open><summary> objectdetection_dynamic_aipp：基于yolov3的目标检测应用，输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_dynamic_aipp/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，在模型推理时，需调用AscendCL提供的接口设置模型推理时需使用的AIPP配置，再对预处理后的图像中的物体进行目标检测，最后通过opencv进行相应后处理。  
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_dynamic_aipp/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv对图像进行预处理，在模型推理时，需调用AscendCL提供的接口设置模型推理时需使用的AIPP配置，再对预处理后的图像中的物体进行目标检测，最后通过opencv进行相应后处理。  
</blockquote></details>

<details open><summary> objectdetection_cvwithaipp：基于vgg_ssd的目标检测应用，使用opencv为输入图像数据进行预处理并在模型转换时开启AIPP功能。输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_cvwithaipp/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv为输入图像数据进行预处理，并在模型转换时开启AIPP功能，实现目标检测的功能。  
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_cvwithaipp/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv为输入图像数据进行预处理，并在模型转换时开启AIPP功能，实现目标检测的功能。  
</blockquote></details>   

<details open><summary> objectdetection_cvwithoutaipp：基于vgg_ssd的目标检测应用，使用opencv为输入图像数据进行预处理并在模型转换时关闭AIPP功能。输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_cvwithoutaipp/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv为输入图像数据进行预处理，并在模型转换时关闭AIPP功能，实现目标检测的功能。  
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_cvwithoutaipp/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv为输入图像数据进行预处理，并在模型转换时关闭AIPP功能，实现目标检测的功能。  
</blockquote></details>

<details open><summary> objectdetection_dvppwithaipp：基于vgg_ssd的目标检测应用，使用dvpp对输入图像数据进行预处理并在模型转换时开启AIPP功能。输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_dvppwithaipp/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用dvpp对输入图像数据进行预处理，并在模型转换时开启AIPP功能，实现目标检测的功能。  
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_dvppwithaipp/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用dvpp对输入图像数据进行预处理，并在模型转换时开启AIPP功能，实现目标检测的功能。  
</blockquote></details>
<details open><summary>objectdetection_video：基于yolov3的目标检测应用，输入为视频，输出为视频。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_video/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用opencv对视频帧进行预处理，对预处理后的视频帧中的物体进行目标检测，最后通过opencv进行相应后处理。    
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/objectdetection_video/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用opencv对视频帧进行预处理，对预处理后的视频帧中的物体进行目标检测，最后通过opencv进行相应后处理。  
</blockquote></details>

<details open><summary>facedetection：基于caffe-ssd的人脸检测应用，输入为树莓派摄像头，输出为视频。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/facedetection/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用dvpp对视频帧进行预处理，对预处理后的视频帧进行人脸检测，最后进行相应后处理。  
</blockquote></details> 

<details open><summary>mark_detection：基于yolov3的口罩识别应用，输入为图片，输出为图片。</summary><blockquote>

- [for_atlas200dk_1.3x.0.0_python](https://gitee.com/ascend/samples/tree/master/mark_detection/for_atlas200dk_1.3x.0.0_python)：该分支是运行在200DK上基于1.3x.0.0版本的python样例。使用opencv对图像进行预处理，对预处理后的图像进行口罩识别，最后通过opencv进行相应后处理。 
</blockquote></details>


<details open><summary>mark_detection_video：基于yolov3的口罩识别应用，输入为视频，输出为视频。</summary><blockquote>

- [for_atlas200dk_1.3x.0.0_c++](https://gitee.com/ascend/samples/tree/master/mark_detection_video/for_atlas200dk_1.3x.0.0_c++)：该分支是运行在200DK上基于1.3x.0.0版本的C++样例。使用dvpp对视频帧进行预处理，对预处理后的视频帧进行口罩识别，最后进行相应后处理。 
- [for_atlas200dk_1.3x.0.0_python](https://gitee.com/ascend/samples/tree/master/mark_detection_video/for_atlas200dk_1.3x.0.0_python)：该分支是运行在200DK上基于1.3x.0.0版本的python样例。使用opencv对视频帧进行预处理，对预处理后的视频帧进行口罩识别，最后进行相应后处理。
</blockquote></details>


<details open><summary> wav2word：基于test_model.pb的语言转换文字应用，输入为语音，输出为文本。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/wav2word/for_atlas200dk_1.7x.0.0_c++)：该分支是运行在200DK上基于1.7x.0.0版本的C++样例。使用python wave库对音频文件进行特征提取，提取200维的音频特征进行计算输入，之后进行语音信号的预处理分帧，加窗，傅里叶变换等进行预处理，对预处理后语音转换成相应文字，最后通过调用Keras附带的CTC_decode函数库进行解码进行相应后处理。    
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/wav2word/for_atlas300_1.7x.0.0_c++)：该分支是运行在ai1环境上基于1.7x.0.0版本的C++样例。使用python wave库对音频文件进行特征提取，提取200维的音频特征进行计算输入，之后进行语音信号的预处理分帧，加窗，傅里叶变换等对语音进行预处理，对预处理后语音转换成相应文字，最后通过调用Keras附带的CTC_decode函数库进行解码进行相应后处理。  
</blockquote></details>     


<details open><summary>hardware-peripheral：基于Atlas200DK的各硬件接口使用样例，详细介绍了gpio、i2c、uart使用说明。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/hardware-peripheral%20/for_atlas200dk_1.7x.0.0_c++)：本仓包含Atlas200DK各硬件接口的使用样例，各文件夹对应不同硬件设备的样例，以供用户参考。  
</blockquote></details>      

<details open><summary>dvpp_samples：Atlas200DK和Atlas300支持的媒体数据处理功能。</summary><blockquote>

- [for_atlas200dk_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/dvpp_samples/for_atlas200dk_1.7x.0.0_c++)：详细介绍了Atlas200dk中venc的功能及约束说明。 
- [for_atlas300_1.7x.0.0_c++](https://gitee.com/ascend/samples/tree/master/dvpp_samples/for_atlas200dk_1.7x.0.0_c++)：详细介绍了Atlas300dk中crop、cropandpaste、jpegd、jpege、resize、vdec的功能及约束说明。
</blockquote></details>