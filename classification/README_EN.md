EN|[中文](README.md)

# Image Classification (Input: Image; Output: Image)

## Overview

The deep convolutional neural network (CNN) AlexNet won the 2012 ImageNet Large Scale Visual Recognition Challenge (ILSVRC) championship. Since then, CNN has won the game, featuring 5%–10% higher accuracy than human vision. CNN-based image classification has become the most active research topic in the AI field. This application provides a demo for developing a classification network application with the Atlas 200 DK or AI acceleration cloud server.

This application runs on the Atlas 200 DK or AI acceleration cloud server to classify objects in local images. Multiple images can be uploaded to the Atlas 200 DK or AI acceleration cloud server as the input images of the trained model for inference, and the classification result is labeled on the images for display.

## Overall design

![Input image description](https://images.gitee.com/uploads/images/2020/0810/152928_dffd1a38_5408865.png "屏幕截图.png")

1. **Allocate runtime resources**: Initialize system resources.

2. **Load the model and create model output**: Load the offline model to the memory managed by the user, and the model information can be obtained, including the buffer sizes of the input and output data. Based on the model information, create model output for inference.

3. **Read and preprocess local images**: Loop over images in the local directory, resize the images to the required size, and create model input data.

4. **Perform model inference**: Perform model inference with the model input data.

5. **Parse the inference result**: Based on the model output, parse the image classification result, obtain the class and the corresponding confidence, and label the class on the image.

## Model architecture

![Input image description](https://images.gitee.com/uploads/images/2020/0810/153018_5bb4f333_5408865.jpeg "googlenet网络结构图.jpg")

## Original model

To obtain the model architecture and the pre-trained model, visit the following website: https://github.com/BVLC/caffe/tree/master/models/bvlc_googlenet

## Image preprocessing

The model requires 224 x 224 images in BGR format, and mean subtraction is performed on the color channels. OpenCV and artificial intelligence preprocessing (AIPP) are used for image preprocessing as follows:

OpenCV:

1. Uses **imread()** to read images in BGR format.
2. Resizes images to 224 x 224.
3. Outputs image data of the uint8 type.

AIPP:

1. Performs mean subtraction on color channels.
2. Converts uint8 data to fp16 data (Da Vinci requires fp16 data as input for model reference).

## Image postprocessing

The output of the model is 1000 pieces of float data, indicating 1000 classes. Each piece of float data is normalized to the range of 0 to 1, indicating the confidence of a class. The class with the highest confidence is chosen as the classification result.

## Application performance

Duration per processing: (preprocessing -> inference -> postprocessing): 17.2 ms

Duration per inference: 3.5 ms

## Optimization

According to the performance, the inference is fast, while the preprocessing and postprocessing take a long time, leading to a long duration per processing. The preprocessing and postprocessing can be optimized by replacing OpenCV with digital vision preprocessing (DVPP), or using a single operator for postprocessing to improve efficiency.