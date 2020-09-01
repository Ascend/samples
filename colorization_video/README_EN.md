EN|[中文](README.md)

# Video Colorization (Input: Video; Output: Video)

## Overview

Old movies are wonderful. However, most of them are black and white. With AI, old videos can be colorized to some extent.

This application runs on the Atlas 200 DK to colorize black and white videos. Black and white videos in multiple formats can be uploaded to the Atlas 200 DK as the input of the trained model for colorization, and the colored video frames can be played.

## Overall design

![Overall design](https://images.gitee.com/uploads/images/2020/0806/161544_a91ea99a_5395865.png "屏幕截图.png")

1) **Allocate runtime resources**: Initialize system resources.
2) **Load the model and create output**: Load the offline model from a file, based on which the model information can be obtained, including the buffer sizes of the input and output data. Based on the model information, create model output for inference.
3) **Read and preprocess the local video**: Use OpenCV to loop over each frame of a local video, preprocess the image data, and create model input data.
4) **Perform model inference**: Call the model inference API to perform model inference with the model input data.
5) **Parse the inference result**: Parse the inference result based on the model output. Colorize each video frame, and call Presenter Agent to send the result to Presenter Server on the host for web UI display.
6) **View the colorized video**: Presenter Server sends the color image information to the web UI on the host so that the colorized video can be viewed in real time by browsing Presenter Server.

## Model architecture

![Model architecture](https://images.gitee.com/uploads/images/2020/0805/095721_70b4f185_5395865.png "屏幕截图.png")

## Original model

To obtain the model architecture and pre-trained model, visit the following website: https://github.com/richzhang/colorization

## Preprocessing

OpenCV **capture.read()** is called to loop over video frames in BGR format. Since the model requires 224 x 224, the read images need to be resized to 224 x 224. Normalize the images after resizing. As described in algorithm design, the model uses the Lab color space. Therefore, images in BGR format need to be converted into Lab format. The model predicts the ab channels with data of the L channel. Therefore, L channel needs to be separated from the Lab image data. L channel after mean subtraction is used as the model input data.

![Preprocessing](https://images.gitee.com/uploads/images/2020/0805/095959_0e2bdf81_5395865.png "屏幕截图.png")

## Postprocessing

OpenCV is used for postprocessing. Obtain the predicted ab channels from the inference result. Resize the ab channels back to the source image size, and combine them with the L channel data to form a Lab image. Convert the Lab image back to BGR format and save it as a JPEG image as the colorized video frame.

![Postprocessing](https://images.gitee.com/uploads/images/2020/0805/100036_247920c8_5395865.png "屏幕截图.png")

## Application performance

Duration per processing: (preprocessing -> inference -> postprocessing): 144 ms

Duration per inference: 0.43 ms

## Optimization

According to the performance, the inference is fast, while the preprocessing and postprocessing take a long time, leading to a long duration per processing. The preprocessing and postprocessing can be optimized by replacing OpenCV with digital vision preprocessing (DVPP), or using a single operator for postprocessing to improve efficiency.