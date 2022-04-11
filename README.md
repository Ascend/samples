English|[中文](README_CN.md)

## About Ascend Samples Repository
   
Ascend Computing Language (AscendCL) provides a collection of C language APIs for users to develop deep neural network apps for object recognition and image classification, ranging from device management, context management, stream management, memory management, to model loading and execution, operator loading and execution, and media data processing. You can call AscendCL APIs through a third-party framework to utilize the compute capability of the Ascend AI Processor, or encapsulate AscendCL into third-party libraries, to provide the runtime and resource management capabilities of the Ascend AI Processor.

This repository provides a wide range of samples developed based on AscendCL APIs. When developing your own samples, feel free to refer to the existing samples in this repository.

## Release Notes

**For the CANN version and Ascend product supported by each sample in the master branch, see [Version and Product Support per Sample](#Version-of-samples).**      
**For historical version, please refer to [CANN Version Description](#Version-Description).**

**Table 1** CANN Version Description <a name="Version-Description"></a>
| CANN Version | Driver Version | Samples Repository Support | Download Link |
|---|---|---|---|
| [5.0.4.alpha002/5.0.4.alpha003/<br>5.0.4.alpha005/5.0.5.alpha001/<br>5.1.RC1.alpha001/5.1.RC1.alpha002/<br>5.1.RC1.alpha003](https://www.hiascend.com/software/cann/community) | [1.0.12.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes |Release 0.6.0: [click here](https://github.com/Ascend/samples/releases/v0.6.0) |
| [5.0.2.alpha005/5.0.3.alpha001/<br>5.0.3.alpha002/5.0.3.alpha003/<br>5.0.3.alpha005](https://ascend.huawei.com/#/software/cann/download) | [1.0.11.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.5.0: [click here](https://github.com/Ascend/samples/releases/v0.5.0) |
| [5.0.2.alpha003](https://ascend.huawei.com/#/software/cann/download) | [1.0.10.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.4.0: [click here](https://github.com/Ascend/samples/releases/v0.4.0) |
| [3.3.0.alpha001/3.3.0.alpha005/<br>3.3.0.alpha006/5.0.2.alpha001/<br>5.0.2.alpha002](https://ascend.huawei.com/#/software/cann/download) | [1.0.9.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.3.0: [click here](https://github.com/Ascend/samples/releases/v0.3.0) |
| [3.2.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.8.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.2.0: [click here](https://github.com/Ascend/samples/releases/v0.2.0) |
| [3.1.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.7.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.1.0: [click here](https://github.com/Ascend/samples/releases/v0.1.0) |

## Directory Structure
| directory | description |
|---|---|
| [common](./common) | Sample warehouse public file directory |
| [cplusplus](./cplusplus) | C++ sample directory of samples warehouse |
| [python](./python) | samples warehouse python sample directory |
| [st](./st) | Samples warehouse sample test case directory | 

## Version and Product Support per Sample<a name="Version and Product Support per Sample"></a>

| Sample Name                              | Language | Product                | Description |
| ---------------------------------------- | -------- | ------------ | ---------------------- |
| [DVPP interface sample](./cplusplus/level2_simple_inference/0_data_process) |  c++ | >=5.0.4 | Call the relevant interface of dvpp to realize image processing. Includes crop/vdec/venc/jpegd/jpege/resize/batchcrop/cropandpaste and other functions |
| [Custom operator sample](./cplusplus/level1_single_api/4_op_dev/2_verify_op) |  c++ | >=5.0.4 | Custom operator operation verification, including Add operator/batchnorm operator/conv2d operator/lstm operator/matmul operator/reshape operator, etc.|
| [200DK peripheral sample](./cplusplus/level1_single_api/5_200dk_peripheral) |  c++ | >=5.0.4 | 200DK peripheral interface related cases, including functions such as configuring GPIO pins/using i2c to read and write data/using uart1 serial port to send and receive data/using camera to take photos or videos.|
| [C++ classification sample](./cplusplus/level2_simple_inference/1_classification) |  c++ | >=5.0.4 | Use the googlenet/ResNet-50 model to classify and infer the input data. Contains multiple feature samples such as pictures/videos/dynamic batch/multi-batch/video stream/universal camera.|
| [C++ detection sample](./cplusplus/level2_simple_inference/2_object_detection) |  c++ | >=5.0.4 | Use the face detection/yolov3/yolov4/vgg_ssd/faster_rcnn model to detect the input data. Contains various feature samples such as general picture/universal video//video stream/universal camera.|
| [C++ natural language processing sample](./cplusplus/level2_simple_inference/5_nlp) |  c++ | >=5.0.4 | Use the nlp model to reason about the input data. |
| [C++ other sample](./cplusplus/level2_simple_inference/6_other) |  c++ | >=5.0.4 | Other model reasoning examples, including black and white image coloring, super-segmentation, image enhancement, etc. | 
| [C++ multithreading sample](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread) |  c++ | >=5.0.4 | Use yolov3, face detection and other models to perform multi-threaded inference examples on input data. |
| [C++ user contribution sample](./cplusplus/contrib) |  c++ | >=5.0.4 | Inference sample contributed by users.|
| [Python classification sample](./python/level2_simple_inference/1_classification) |  python | >=5.0.4 | Use the googlenet/inceptionv3/vgg16 model to classify and infer the input data. |
| [python detection sample](./python/level2_simple_inference/1_classification) |  python | >=5.0.4 | Use the googlenet/inceptionv3/vgg16 model to classify and infer the input data. |
| [Python segmentation sample](./python/level2_simple_inference/3_segmentation) | python | >=5.0.4 | Use the segmentation model to segment the input image. |
| [python natural language processing sample](./python/level2_simple_inference/5_nlp) | python | >=5.0.4 | Use the nlp model to reason about the input data. |
| [python other sample](./python/level2_simple_inference/6_other) | python | >=5.0.4 | Other model reasoning sample, including black and white image coloring, image restoration, etc. |
| [End-to-end sample from training to inference](./python/level2_simple_inference/n_e2e) | python | >=5.0.4 | End-to-end sample guidance from training to deployment, including guidance on mask recognition, garbage classification, cat and dog battles, etc. |
| [Python industry sample](./python/level3_multi_model) | python | >=5.0.4 | For more complex samples, combine hardware or use multi-model and multi-threaded samples. Such as removing the designated foreground target sample of the image, the robot arm sample, etc. |
| [Python user contribution sample](./python/contrib) | python | >=5.0.4 | Inference sample contributed by users. |


## Sample Deployment

   Deploy and run a sample by referring to the corresponding README file.   

## Documentation

Obtain related documentation at [Ascend Documentation](https://support.huaweicloud.com/intl/en-us/ascenddocument/index.html).

## Community

Get support from our developer community. Find answers and talk to other developers in our forum.

Ascend website: https://www.huaweicloud.com/intl/en-us/ascend/home.html

Ascend forum: https://forum.huawei.com/enterprise/en/forum-100504.html

## Contribution

Welcome to contribute to Ascend Samples. For more details, please refer to our [Contribution Wiki](./CONTRIBUTING_EN.md).

## License
[Apache License 2.0](LICENSE)