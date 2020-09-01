EN|[中文](README.md)

# Image Colorization (Input: Image; Output: Image)

## Overview

With the popularity of smartphones, it is easy to take a bright and clear photo today. However, most old photos are black and white. With AI, old photos can be colorized to some extent.

This application runs on the Atlas 200 DK to colorize black and white images. Multiple black and white images can be uploaded to the Atlas 200 DK as input images of the trained model for colorization, and the colored images are saved.

## Overall design

![Workflow](https://images.gitee.com/uploads/images/2020/0805/095543_cea73640_5395865.png "屏幕截图.png")

1) **Allocate runtime resources**: Initialize system resources.
2) **Load the model and create model output**: Load the offline model to the memory managed by the user, and the model information can be obtained, including the buffer sizes of the input and output data. Based on the model information, create model output for inference.
3) **Read and preprocess local images**: Use OpenCV to loop over images in a local directory, preprocess the images, and create model input data.
4) **Perform model inference**: Perform model inference with the model input data.
5) **Parse the inference result**: Parse the inference result based on the model output. Use OpenCV to write the colorized image data to an image file.

## Model architecture

![Model architecture](https://images.gitee.com/uploads/images/2020/0805/095721_70b4f185_5395865.png "屏幕截图.png")

## Original model

To obtain the model architecture and pre-trained model, visit the following website: https://github.com/richzhang/colorization

## Image preprocessing

OpenCV reads images with **imread()** in BGR format. Since the model requires 224 x 224, the read images need to be resized to 224 x 224. Normalize the images after resizing. As described in algorithm design, the model uses the Lab color space. Therefore, images in BGR format need to be converted into Lab format. The model predicts the ab channels with data of the L channel. Therefore, L channel needs to be separated from the Lab image data. L channel after mean subtraction is used as the model input data.

![Preprocessing](https://images.gitee.com/uploads/images/2020/0805/095959_0e2bdf81_5395865.png "屏幕截图.png")

## Image postprocessing

OpenCV is used for image preprocessing. Obtain the predicted ab channels from the inference result. Resize the ab channels back to the source image size, and combine them with the L channel data to form a Lab image. Convert the Lab image back to BGR format and save it as a JPEG image as the colorized image.

![Postprocessing](https://images.gitee.com/uploads/images/2020/0805/100036_247920c8_5395865.png "屏幕截图.png")

## Application performance

Duration per processing: (preprocessing -> inference -> postprocessing): 144 ms

Duration per inference: 0.43 ms

## Optimization

According to the performance, the inference is fast, while the preprocessing and postprocessing take a long time, leading to a long duration per processing. The preprocessing and postprocessing can be optimized by replacing OpenCV with digital vision preprocessing (DVPP), or using a single operator for postprocessing to improve efficiency.