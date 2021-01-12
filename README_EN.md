[English]|[中文](README_CN.md)

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
├── [python](./python)：**Python samples **  
│   ├── common   
│   ├── contrib   
│   ├── level1_single_api   
│   ├── level2_simple_inference   
└   └── level3_multi_model   

##Version and Product Support per Sample <a name="Version-of-samples"></a>

| Sample | Language | Version | Product |
|---|---|---|---|
| [crop](./cplusplus/level1_single_api/1_acl/4_dvpp/crop) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [vdec](./cplusplus/level1_single_api/1_acl/4_dvpp/vdec) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [venc](./cplusplus/level1_single_api/1_acl/4_dvpp/venc) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [jpegd](./cplusplus/level1_single_api/1_acl/4_dvpp/jpegd) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [jpege](./cplusplus/level1_single_api/1_acl/4_dvpp/jpege) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [resize](./cplusplus/level1_single_api/1_acl/4_dvpp/resize) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [batchcrop](./cplusplus/level1_single_api/1_acl/4_dvpp/batchcrop) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [venc_image](./cplusplus/level1_single_api/1_acl/4_dvpp/venc_image) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [vdecandvenc](./cplusplus/level1_single_api/1_acl/4_dvpp/vdecandvenc) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [cropandpaste](./cplusplus/level1_single_api/1_acl/4_dvpp/cropandpaste) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [smallResolution_cropandpaste](./cplusplus/level1_single_api/1_acl/4_dvpp/smallResolution_cropandpaste) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [gemm](./cplusplus/level1_single_api/1_acl/5_blas/gemm) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300  |
| [acl_execute_add](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_add) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300  |
| [acl_execute_batchnorm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_batchnorm) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300  |
| [acl_execute_conv2d](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_conv2d) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300  |
| [acl_execute_lstm](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_lstm) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300  |
| [acl_execute_matmul](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_matmul) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300  |
| [acl_execute_reshape](./cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_reshape) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300  |
| [gpio](./cplusplus/level1_single_api/5_200dk_peripheral) | C++ |20.0/20.1 | Atlas200DK |
| [i2c](./cplusplus/level1_single_api/5_200dk_peripheral/i2c) | C++ |20.0/20.1 | Atlas200DK |
| [uart](./cplusplus/level1_single_api/5_200dk_peripheral/uart) | C++ |20.0/20.1 | Atlas200DK |
| [ascendcamera](./cplusplus/level1_single_api/5_200dk_peripheral/ascendcamera) | C++ |20.0/20.1 | Atlas200DK |
| [googlenet_imagenet_video](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_video) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [googlenet_imagenet_picture](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_picture) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [vdec_resnet50_classification](./cplusplus/level2_simple_inference/1_classification/vdec_resnet50_classification) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [resnet50_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [resnet50_async_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/resnet50_async_imagenet_classification) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [googlenet_imagenet_dynamic_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_dynamic_batch) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [googlenet_imagenet_multi_batch](./cplusplus/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [vpc_resnet50_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [vpc_jpeg_resnet50_imagenet_classification](./cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification) | C++ | 20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [face_detection_camera](./cplusplus/level2_simple_inference/2_object_detection/face_detection_camera) | C++ |20.0/20.1 | Atlas200DK |
| [YOLOV3_coco_detection_video](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_coco_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV4_coco_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV4_coco_detection_picture) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_VOC_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_VOC_detection_picture) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_coco_detection_VENC](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_VENC) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [VGG_SSD_coco_detection_CV_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_with_AIPP) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [VGG_SSD_coco_detection_CV_without_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_CV_without_AIPP) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [VGG_SSD_coco_detection_DVPP_with_AIPP](./cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_coco_detection_dynamic_AIPP](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_dynamic_AIPP) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_dynamic_batch_detection_picture](./cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [faste_RCNN_VOC_detection_dynamic_resolution](./cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [WAV_to_word](./cplusplus/level2_simple_inference/5_nlp/WAV_to_word) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [colorization](./cplusplus/level2_simple_inference/6_other/colorization) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [colorization_video](./cplusplus/level2_simple_inference/6_other/colorization_video) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_coco_detection_multi_thread_VENC](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/YOLOV3_coco_detection_multi_thread_VENC) | C++ |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [face_recognition_camera](./cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/face_recognition_camera) | C++ |20.1  | Atlas200DK |
| [googlenet_imagenet_multi_batch](./python/level2_simple_inference/1_classification/googlenet_imagenet_multi_batch) | Python |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [googlenet_imagenet_picture](./python/level2_simple_inference/1_classification/googlenet_imagenet_picture) | Python |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_coco_detection_picture](./python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_picture) | Python |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_coco_detection_video](./python/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video) | Python |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [YOLOV3_mask_detection_camera](./python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_camera) | Python |20.0/20.1 | Atlas200DK |
| [face_detection_camera](./python/level2_simple_inference/2_object_detection/face_detection_camera) | Python |20.0/20.1 | Atlas200DK |
| [colorization_picture](./python/level2_simple_inference/6_other/colorization_picture) | Python |20.0/20.1 | Atlas 200 DK/Atlas 300 |
| [colorization_video](./python/level2_simple_inference/6_other/colorization_video) | Python |20.0/20.1 | Atlas 200 DK/Atlas 300 |

## Sample Deployment

   Deploy and run a sample by referring to the corresponding README file.   

## Documentation

Obtain related documentation at [Ascend Documentation] (https://support.huaweicloud.com/intl/en-us/ascenddocument/index.html).

## Community

Get support from our developer community. Find answers and talk to other developers in our forum.

Ascend website: https://www.huaweicloud.com/intl/en-us/ascend/home.html

Ascend forum: https://forum.huawei.com/enterprise/en/forum-100504.html

## Contribution

Welcome to contribute to Ascend Samples. For more details, please refer to our [Contribution Wiki](./CONTRIBUTING.md).

## License
[Apache License 2.0](LICENSE)