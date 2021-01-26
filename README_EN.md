English|[中文](README_CN.md)

## About Ascend Samples Repository
   
Ascend Computing Language (AscendCL) provides a collection of C language APIs for users to develop deep neural network apps for object recognition and image classification, ranging from device management, context management, stream management, memory management, to model loading and execution, operator loading and execution, and media data processing. You can call AscendCL APIs through a third-party framework to utilize the compute capability of the Ascend AI Processor, or encapsulate AscendCL into third-party libraries, to provide the runtime and resource management capabilities of the Ascend AI Processor.

This repository provides a wide range of samples developed based on AscendCL APIs. When developing your own samples, feel free to refer to the existing samples in this repository.

## Release Notes

**For the CANN version and Ascend product supported by each sample in the master branch, see [Version and Product Support per Sample](#Version-of-samples). You can also download a specific sample release by referring to [CANN Version Description](#Version-Description).**

**Table 1** CANN Version Description <a name="Version-Description"></a>
| CANN Version | Samples Repository Support | Download Link |
|---|---|---|
| [20.0–20.1](https://ascend.huawei.com/#/software/cann/download) | Yes | Release 0.1.0 [click here](https://gitee.com/ascend/samples/releases/v0.1.0) |

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

## Version and Product Support per Sample <a name="Version-of-samples"></a>

| Sample | Language | Version | Product | Introduction |
|---|---|---|---|---|
| [crop](./cplusplus/level2_simple_inference/0_data_process/crop) |  c++ |20.0/20.1  | A200DK/A300 | Call the crop interface of dvpp to realize the image cropping function.|
| [vdec](./cplusplus/level2_simple_inference/0_data_process/vdec) |  c++ |20.0/20.1  | A200DK/A300 | Call the vdec interface of dvpp to realize the function of picture decoding.|
| [venc](./cplusplus/level2_simple_inference/0_data_process/venc) |  c++ |20.0/20.1  | A200DK/A300 | Call the venc interface of dvpp to realize the video encoding function.|
| [jpegd](./cplusplus/level2_simple_inference/0_data_process/jpegd) |  c++ |20.0/20.1  | A200DK/A300 | Call the jpegd interface of dvpp to realize the function of picture decoding.|
| [jpege](./cplusplus/level2_simple_inference/0_data_process/jpege) |  c++ |20.0/20.1  | A200DK/A300 | Call the jpege interface of dvpp to realize the function of image encoding.|
| [resize](./cplusplus/level2_simple_inference/0_data_process/resize) |  c++ |20.0/20.1  | A200DK/A300 | Call the resize interface of dvpp to realize the image zoom function.|
| [batchcrop](./cplusplus/level2_simple_inference/0_data_process/batchcrop) |  c++ |20.0/20.1  | A200DK/A300 | Cut out eight 224*224 sub-pictures according to the specified area from a YUV picture.|
| [venc<br>_image](./cplusplus/level2_simple_inference/0_data_process/venc_image) |  c++ |20.0/20.1  | A200DK/A300 | Continuously encode a YUV picture to generate a video stream in H265 format.|
| [vdecandvenc](./cplusplus/level2_simple_inference/0_data_process/vdecandvenc) |  c++ |20.0/20.1  | A200DK/A300 | Call the venc and vdec interfaces of dvpp to realize the video encoding function.|
| [cropandpaste](./cplusplus/level2_simple_inference/0_data_process/cropandpaste) |  c++ |20.0/20.1  | A200DK/A300 | Call the cropandpaste interface of dvpp to paste the picture at the specified location and size of the picture to the specified location of the output picture.|
| [smallResolution<br>_cropandpaste](./cplusplus/level2_simple_inference/0_data_process/smallResolution_cropandpaste) | c++ | 20.0/20.1 | A200DK/A300 | Cut out the specified input picture (including the area less than 10*6), and then paste it to the output picture.|
| [gemm](./cplusplus/level1_single_api/1_acl/4_blas/gemm) |  c++ |20.0/20.1  | A200DK/A300 | Realize matrix-matrix multiplication operation.|
| [acl_execute<br>_add](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_add) |  c++ |20.0/20.1  | A200DK/A300 | Custom Add operator to run verification.|
| [acl_execute<br>_batchnorm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_batchnorm) |  c++ |20.0/20.1  | A200DK/A300 | Custom Batchnorm operator to run verification.|
| [acl_execute<br>_conv2d](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_conv2d) |  c++ |20.0/20.1  | A200DK/A300 | Custom Conv2d operator to run verification.|
| [acl_execute<br>_lstm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_lstm) |  c++ |20.0/20.1  | A200DK/A300 | Custom Lstm operator to run verification.|
| [acl_execute<br>_matmul](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_matmul) |  c++ |20.0/20.1  | A200DK/A300 | Custom Matmul operator to run verification.|
| [acl_execute<br>_reshape](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_reshape) |  c++ |20.0/20.1  | A200DK/A300 | Custom Reshape  operator to run verification.|
| [gpio](./cplusplus/level1_single_api/5_200dk_peripheral/gpio) |  c++ |20.0/20.1  | A200DK| Configure the GPIO pins.|
| [i2c](./cplusplus/level1_single_api/5_200dk_peripheral/i2c) |  c++ |20.0/20.1  | A200DK| Use i2c to read and write data.|
| [uart](./cplusplus/level1_single_api/5_200dk_peripheral/uart) |  c++ |20.0/20.1  | A200DK| Use uart1 serial port to send and receive data.|
| [ascendcamera](./cplusplus/level1_single_api/5_200dk_peripheral/ascendcamera) |  c++ |20.0/20.1  | A200DK| Use the camera to take photos or videos.|
| [googlenet_imagenet<br>_video](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_video) |  c++ |20.0/20.1  | A200DK/A300 | Use googlenet model to classify and reason the input video.|
| [googlenet_imagenet<br>_picture](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture) | c++ | 20.0/20.1  | A200DK/A300 | Use googlenet model to classify and reason the input images.|
| [vdec_resnet50<br>_classification](./cplusplus/level2_simple_inference/1_classification/vdec_resnet50_classification) | c++ | 20.0/20.1  | A200DK/A300 | Based on the Caffe ResNet-50 network to realize the classification and reasoning of h265 video stream pictures.|
| [resnet50_imagenet<br>_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | Realize the function of image classification based on Caffe ResNet-50 network.|
| [resnet50_async_imagenet<br>_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_async_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | Based on the Caffe ResNet-50 network (single input, single batch), it implements multi-image asynchronous classification inference.|
| [googlenet_imagenet<br>_dynamic_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_dynamic_batch) | c++ | 20.0/20.1  | A200DK/A300 | Use the googlenet model to classify and reason the input images. This case uses the dynamic batch feature.|
| [googlenet_imagenet<br>_multi_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) | c++ | 20.0/20.1  | A200DK/A300 | Use the googlenet model to classify and reason the input images. This case uses the multi-batch feature.|
| [vpc_resnet50_imagenet<br>_classification](./cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | Image classification based on Caffe ResNet-50 network (picture decoding + scaling + synchronous reasoning)|
| [vpc_jpeg_resnet50<br>_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification) | c++ | 20.0/20.1  | A200DK/A300 | Realize image classification based on Caffe ResNet-50 network (image decoding + matting zoom + image encoding + synchronous reasoning)|
| [face_detection<br>_camera](./cplusplus/level2_simple_inference/2_object_detection/face_detection_camera) |  c++ |20.0/20.1  | A200DK| Use the face detection model to detect the face of the instant video in the raspberry camera.|
| [YOLOV3_coco<br>_detection_video](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video) |  c++ |20.0/20.1  | A200DK/A300 | Detect objects appearing in the video, and give the prediction result in the video.|
| [YOLOV3_coco<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | Use the yolov3 model to perform predictive inference on the input picture, and print the result on the output picture.|
| [YOLOV4_coco<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | Use the yolov4 model to perform predictive inference on the input picture, and print the result on the output picture.|
| [YOLOV3_VOC<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_VOC_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | Use the yolov3 model to perform predictive inference on the input picture, and print the result on the output picture. (For tensorflow model, post-processing is done by code)|
| [YOLOV3_coco<br>_detection_VENC](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_VENC) |  c++ |20.0/20.1  | A200DK/A300 | Detect objects appearing in the video, and give the prediction result in the video.|
| [VGG_SSD_coco_detection<br>_CV_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_with_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | Use the vgg_ssd model to perform predictive inference on the input picture, and print the result on the output picture. (Use opencv and aipp processing)|
| [VGG_SSD_coco_detection<br>_CV_without_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_without_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | Use the vgg_ssd model to perform predictive inference on the input picture, and print the result on the output picture. (Use opencv to process)|
| [VGG_SSD_coco_detection<br>_DVPP_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | Use the vgg_ssd model to perform predictive inference on the input picture, and print the result on the output picture. (Use dvpp and aipp processing)|
| [YOLOV3_coco_detection<br>_dynamic_AIPP](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_dynamic_AIPP) |  c++ |20.0/20.1  | A200DK/A300 | Use the yolov3 model to perform predictive inference on the input picture, and print the result on the output picture. (Used dynamic aipp feature)|
| [YOLOV3_dynamic_batch<br>_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture) |  c++ |20.0/20.1  | A200DK/A300 | Target detection is realized based on Caffe YOLOv3 network. (Dynamic Batch/Dynamic Resolution)|
| [faste_RCNN_VOC_detection<br>_dynamic_resolution](./cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution) |  c++ |20.0/20.1  | A200DK/A300 | Use the faster_rcnn model to perform predictive inference on the input picture, and print the result on the output picture.|
| [WAV_to_word](./cplusplus/level2_simple_inference/5_nlp/WAV_to_word) |  c++ |20.0/20.1  | A200DK/A300 | Use the speech conversion model to reason about the input speech.|
| [colorization](./cplusplus/level2_simple_inference/6_other/colorization) |  c++ |20.0/20.1  | A200DK/A300 | Use the colorization model to perform coloring inference on the input black and white pictures.| 
| [colorization<br>_video](./cplusplus/level2_simple_inference/6_other/colorization_video) |  c++ |20.0/20.1  | A200DK/A300 | Use the black and white image coloring model to infer the input black and white video.|
| [YOLOV3_coco_detection<br>_multi_thread_VENC](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/YOLOV3_coco_detection_multi_thread_VENC) |  c++ |20.0/20.1  | A200DK/A300 | Use the yolov3 model to classify and infer the input video. (Multithreaded processing)|
| [face_recognition<br>_camera](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera) |  c++ |20.1  | A200DK| Use the camera to predict the face information in the video and compare it with the registered face to predict the most likely user.|
| [googlenet_imagenet<br>_multi_batch](./python/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) |  python |20.0/20.1  | A200DK/A300 | Use googlenet model to classify and reason the input images. (Multi batch)|
| [googlenet_imagenet<br>_picture](./python/level2_simple_inference/1_classification/googlenet_imagenet_picture) | python |20.0/20.1  | A200DK/A300 | Use googlenet model to classify and reason the input images.|
| [YOLOV3_coco_detection<br>_picture](./python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) | python |20.0/20.1  | A200DK/A300 | Use the yolov3 model to perform predictive inference on the input picture, and print the result on the output picture.|
| [YOLOV3_mask_detection<br>_picture](./python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_picture) | python |20.0/20.1  | A200DK/A300 | The function of predicting the mask, face, and person information in the picture is realized.|
| [YOLOV3_mask_detection<br>_video](./python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_video) | python |20.0/20.1  | A200DK| The function of predicting masks, faces, and person information in the video is realized.。|
| [face_detection<br>_camera](./python/level2_simple_inference/2_object_detection/face_detection_camera) | python |20.0/20.1  | A200DK| Use the face detection model to detect the face of the instant video in the raspberry camera.|
| [colorization<br>_picture](./python/level2_simple_inference/6_other/colorization_picture) | python |20.0/20.1  | A200DK/A300 | Use the colorization model to perform coloring inference on the input black and white pictures.|
| [colorization<br>_video](./python/level2_simple_inference/6_other/colorization_video) | python |20.0/20.1  | A200DK/A300 | Use the black and white image coloring model to reason about the input black and white video.|

## Sample Deployment

   Deploy and run a sample by referring to the corresponding README file.   

## Documentation

Obtain related documentation at [Ascend Documentation] (https://support.huaweicloud.com/intl/en-us/ascenddocument/index.html).

## Community

Get support from our developer community. Find answers and talk to other developers in our forum.

Ascend website: https://www.huaweicloud.com/intl/en-us/ascend/home.html

Ascend forum: https://forum.huawei.com/enterprise/en/forum-100504.html

## Contribution

Welcome to contribute to Ascend Samples. For more details, please refer to our [Contribution Wiki](./CONTRIBUTING_EN.md).

## License
[Apache License 2.0](LICENSE)