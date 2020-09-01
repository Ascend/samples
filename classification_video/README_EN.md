EN|[中文](README.md)

# Video Classification (Input: Video; Output: Video)

## Overview

The deep convolutional neural network (CNN) AlexNet won the 2012 ImageNet Large Scale Visual Recognition Challenge (ILSVRC) championship. Since then, CNN has won the game, featuring 5%–10% higher accuracy than human vision. CNN-based image classification has become the most active research topic in the AI field. This application provides a demo for developing a classification network application with the Atlas 200 DK or AI acceleration cloud server.

This application runs on the Atlas 200 DK or AI acceleration cloud server to classify objects in local video frames, and the result is displayed in video frames.

## Overall design

![Input image description](https://images.gitee.com/uploads/images/2020/0810/162403_39b2ad88_5408865.png "1.png")

1. **Allocate runtime resources**: Initialize system resources.

2. **Load the model and create output**: Load the offline model from a file, based on which the model information can be obtained, including the buffer sizes of the input and output data. Based on the model information, create model output for inference.

3. **Read and preprocess the local video**: Use OpenCV to loop over each frame of the local video, resize the images to the required size, and create model input data.

4. **Perform model inference**: Call the model inference API to perform model inference with the model input data.

5. **Parse the inference result**: Based on the model output, parse the inference result, and obtain the object classes in each video frame. After this, call Presenter Agent to send the result to Presenter Server on the host for web UI display.

6. **View the classification information**: Presenter Server labels the classes on each frame based on the inference result, and sends the image information to the web Ul on the host so that the video object classification information can be viewed in real time by browsing Presenter Server.

## Model architecture

![Input image description](https://images.gitee.com/uploads/images/2020/0810/161511_e5e94b94_5408865.jpeg "googlenet网络结构图.jpg")

## Original model

To obtain the model architecture and the pre-trained model, visit the following website: https://github.com/BVLC/caffe/tree/master/models/bvlc_googlenet

## Preprocessing

The model requires 224 x 224 video frames in BGR format, and mean subtraction is performed on the color channels. OpenCV and artificial intelligence preprocessing (AIPP) are used for image preprocessing as follows:

OpenCV:

1. Loops over video frames in a video.
2. Resizes the video frames to 224 x 224.
3. Outputs video frame data of the uint8 type.

AIPP:

1. Performs mean subtraction on color channels.
2. Converts uint8 data to fp16 data (Da Vinci requires fp16 data as input for model reference).

## Postprocessing

The output of the model is 1000 pieces of float data, indicating 1000 classes. Each piece of float data is normalized to the range of 0 to 1, indicating the confidence of a class. The class with the highest confidence is chosen as the classification result.

## Application performance

Duration per processing: (preprocessing -> inference -> postprocessing): 17.2 ms

Duration per inference: 3.5 ms

## Optimization

According to the performance, the inference is fast, while the preprocessing and postprocessing take a long time, leading to a long duration per processing. The preprocessing and postprocessing can be optimized by replacing OpenCV with digital vision preprocessing (DVPP), or using a single operator for postprocessing to improve efficiency.