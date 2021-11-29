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
| [3.1.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.7.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.1.0: [click here](https://github.com/Ascend/samples/releases/v0.1.0) |
| [3.2.0.alpha001](https://ascend.huawei.com/#/software/cann/download) | [1.0.8.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.2.0: [click here](https://github.com/Ascend/samples/releases/v0.2.0) |
| [3.3.0.alpha001/3.3.0.alpha005/<br>3.3.0.alpha006/5.0.2.alpha001/<br>5.0.2.alpha002](https://ascend.huawei.com/#/software/cann/download) | [1.0.9.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.3.0: [click here](https://github.com/Ascend/samples/releases/v0.3.0) |
| [5.0.2.alpha003](https://ascend.huawei.com/#/software/cann/download) | [1.0.10.alpha](https://www.hiascend.com/en/hardware/firmware-drivers?tag=community) | Yes | Release 0.4.0: [click here](https://github.com/Ascend/samples/releases/v0.4.0) |
## Directory Structure

**./**   
├── [C++](./cplusplus)：**C++ samples**    
│   ├── common   
│   ├── contrib   
│   ├── level1_single_api   
│   ├── level2_simple_inference   
│   └── level3_multi_model    
├── [python](./python)：**Python samples**  
│   ├── common   
│   ├── contrib   
│   ├── level1_single_api   
│   ├── level2_simple_inference   
└   └── level3_multi_model   

## Version and Product Support per Sample<a name="Version and Product Support per Sample"></a>

| Sample Name                              | Language | CANN Version | Product                | Description&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; |
| ---------------------------------------- | -------- | ------------ | ---------------------- | ---------------------------------------- |
| [DVPP API samples](./cplusplus/level2_simple_inference/0_data_process) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Implements image processing by using DVPP APIs, covering functions such as cropping, VDEC, VENC, JPEGD, JPEGE, resizing, batch cropping, and cropping and pasting. |
| [Custom operator samples](./cplusplus/level1_single_api/4_op_dev/2_verify_op) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Verifies the functionality of custom operators, including the Add, batchnorm, Conv2D, lstm, matmul, and reshape operators. |
| [Atlas 200 DK external samples](./cplusplus/level1_single_api/5_200dk_peripheral) | C++      | >3.0.0       | Atlas 200 DK           | Uses the Atlas 200 DK external interfaces to configure GPIO pins, read and write data over the I2C interface, transfer data over the UART1 serial port, and use the camera to capture photos or videos. |
| [C++ general classification example](./cplusplus/level2_simple_inference/1_classification) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs image classification by using a GoogLeNet or ResNet-50 model. Provides samples in typical inference scenarios with common images, common videos, dynamic batch sizes, batching, video streams, and general cameras. |
| [C++ general detection example](./cplusplus/level2_simple_inference/2_object_detection) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Detects targets in the input data by using the face detection YOLOv3, YOLOv4, vgg_ssd, or faster_rcnn model. Includes feature samples such as common images, common videos, video code streams, and general cameras. |
| [gemm](./cplusplus/level1_single_api/1_acl/4_blas/gemm) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Implements matrix-matrix multiplication. |
| [WAV_to_word](./cplusplus/level2_simple_inference/5_nlp/WAV_to_word) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs inference on the input speech by using a speech to text model. |
| [colorization](./cplusplus/level2_simple_inference/6_other/colorization) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs colorization inference on an input black-and-white image by using a colorization model. |
| [colorization_video](./cplusplus/level2_simple_inference/6_other/colorization_video) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs inference on an input black-and-white video by using a colorization model. |
| [DeRain](./cplusplus/level2_simple_inference/6_other/DeRain) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Restores a degraded local heavy rain image by deraining and defogging to enhance an image. |
| [DeblurGAN_GOPRO_Blur2Sharp](./cplusplus/level2_simple_inference/6_other/DeblurGAN_GOPRO_Blur2Sharp) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Sharpens a blurred image by using the DeblurGAN model. |
| [YOLOV3_coco_detection_multi_thread_VENC](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/YOLOV3_coco_detection_multi_thread_VENC) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs object detection on the input video by using the YOLOv3 model using multithreaded processing. |
| [multi_channels_rtsp](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Detects faces from two video stream inputs at the same time by using a face detection model. |
| [AI_painting](./cplusplus/contrib/AI_painting) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Creates a landscape image based on the input category and layout information by using an AI painting model. |
| [AscendBot](./cplusplus/contrib/AscendBot) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Implements object tracking, vehicle tracking, and anti-falling functions by remotely controlling the vehicle on the mobile phone with the APK installed. |
| [HandWrite](./cplusplus/contrib/HandWrite) | C++      | >3.0.0       | Atlas 200 DK           | Recognizes the Chinese characters captured by the camera and displays the result on the Presenter Server WebUI. |
| [ar_shadowgan](./cplusplus/contrib/ar_shadowgan) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Provides an AI image sample with shadow generated, which shows how to perform inference on the input image by using the GAN model. |
| [cartoonGAN_picture](./cplusplus/contrib/cartoonGAN_picture) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Provides a cartoon image sample, which shows how to cartoonize the input image by using the CartoonGAN model. |
| [human_segmentation](./cplusplus/contrib/human_segmentation) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs semantic segmentation inference on the human bodies in the input video. |
| [super_resolution](./cplusplus/contrib/super_resolution) | C++      | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs super-resolution processing on the input image by using the SRCNN, FSRCNN, VDSR, or ESPCN model. |
| [General classification with Python](./python/level2_simple_inference/1_classification) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs classification by using the GoogLeNet, Inception-v3, or VGG-16 model. |
| [General detection with Python](./python/level2_simple_inference/1_classification) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Detects faces by using the YOLOv3 or YOLOv4 model. |
| [deeplabv3_pascal_pic](./python/level2_simple_inference/3_segmentation/deeplabv3_pascal_pic) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs segmentation on the input image by using the DeepLab-v3+ model. |
| [bert_text_classification](./python/level2_simple_inference/5_nlp/bert_text_classification) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Classifies texts by using the BERT model. |
| [colorization_picture](./python/level2_simple_inference/6_other/colorization_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs colorization inference on an input black-and-white image by using a colorization model. |
| [colorization_video](./python/level2_simple_inference/6_other/colorization_video) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs inference on an input black-and-white video by using a colorization model. |
| [imageinpainting_hifill](./python/level2_simple_inference/6_other/imageinpainting_hifill) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Inpaints a JPG image based on a mask to ultra-high resolution. |
| [YOLOV3_mask_detection_e2e](./python/level2_simple_inference/n_e2e/YOLOV3_mask_detection_e2e) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Trains a model on ModelArts and deploys it on an Ascend 310 AI Processor to develop a mask recognition AI application (image input+image output). |
| [garbage_e2e](./python/level2_simple_inference/n_e2e/garbage_e2e) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Trains a MindSpore model and deploys it on an Ascend 310 AI Processor to develop a garbage classification AI application (image input+image output). |
| [vgg16_cat_dog_e2e](./python/level2_simple_inference/n_e2e/vgg16_cat_dog_e2e) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Trains a model on Jupyter notebook on ModelArts and deploys it on an Ascend 310 AI Processor to develop an AI application (image input+image output) for cat/dog recognition. |
| [Robotic_Arm_Object_Following](./python/level3_multi_model/Robotic_Arm_Object_Following) | Python   | >3.0.0       | Atlas 200 DK           | Infers the RGB data streams provided by the binocular depth camera by running the YOLOv3 model on Atlas 200 DK, to locate a target object in an image in real time. Controls the manipulator's postures to make it move with the target based on the depth data flow of the camera. |
| [3Dgesture_recognition](./python/contrib/3Dgesture_recognition) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs classification by using the 3D-CNN model. |
| [SentimentAnalysis](./python/contrib/SentimentAnalysis) | Python   | >3.0.0       | Atlas 200 DK           | Implements the inference function of the sentence-level emotional polarity classification network and outputs the confidence value of each category. |
| [YOLOV3_plane_detection](./python/contrib/YOLOV3_plane_detection) | Python   | >3.0.0       | Atlas 200 DK           | Realizes aircraft target detection in remote sensing image. |
| [cartoonGAN_picture](./python/contrib/cartoonGAN_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Cartoonizes an input image by using the CartoonGAN model. |
| [crowd_count_picture](./python/contrib/crowd_count_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Counts the number of people by using the count_person.caffe model. |
| [dehaze_picture](./python/contrib/crowd_count_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Defogs an input image by using the deploy_vel model. |
| [edge_detection_picture](./python/contrib/edge_detection_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs edge detection on an input image by using the RCF model. |
| [garbage_picture](./python/contrib/garbage_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs image classification by using the MobileNet-v2 model. |
| [gesture_recognition_picture](./python/contrib/garbage_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Recognizes gestures in an input image by using the gesture_yuv model. |
| [human_protein_map_classification](./python/contrib/human_protein_map_classification) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Automatically classifies and evaluates protein images. Thanks for the sample contribution of Shanghai Jiao Tong University. |
| [image_HDR_enhance](./python/contrib/image_HDR_enhance) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs HDR enhancement on an input image with insufficient exposure by using the model. Thanks for the sample contribution of Shenzhen University. |
| [inceptionv2_picture](./python/contrib/inceptionv2_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Performs image classification on baseboard images by using the Inception-v2 model. Thanks for the sample contribution of Suny Wonders. |
| [portrait_picture](./python/contrib/portrait_picture) | Python   | >3.0.0       | Atlas 200 DK/Atlas 300 | Segments the portrait in an input image from the background and then merges the segmented image with a different background by using the PortraitNet model. Thanks for the sample contribution of Tsinghua University. |


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