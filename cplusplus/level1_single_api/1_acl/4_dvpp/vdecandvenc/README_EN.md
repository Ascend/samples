EN|[中文](README.md)

# Video Face Detection (Input: Video Stream of Raspberry Pi Camera; Output: Video)

## Overview

A face contains rich information, providing vital information for mutual recognition of human beings. It is also one of the most interested objects in images and videos. Compared with detection using other human biological features such as fingerprint and voice, face detection is more direct and friendly, and is widely used in identification, access control, video conference, archive management, object-based image and video retrieval. It has been a research hotspot in the AI field.

This application uses the video captured by a camera connected to the Atlas 200 DK as the input, detects faces in video frames in real time, labels face information, and sends the information to the web UI for display.

## Overall design

![Input image description](https://images.gitee.com/uploads/images/2020/0811/185601_4b03a526_5408865.png "人脸检测Camera版本.png")

1. **Allocate runtime resources**: Initialize system resources.

2. **Load the model and create output**: Load the offline model from a file, based on which the model information can be obtained, including the buffer sizes of the input and output data. Based on the model information, create model output for inference.

3. **Read and preprocess the local video**: Use the media API library set provided by the Atlas 200 DK to loop over each frame in YUV420SP format, resize the images to the required size, and create model input data.

4. **Perform model inference**: Call the model inference API to perform model inference with the model input data.

5. **Parse the inference result**: Based on the model output, parse the face detection result, obtain the position of the detected face as well as its confidence in the video frame, convert the image into JPEG format, and call Presenter Agent to send the result to Presenter Server on the host for web UI display.

6. **View the detection information**: Presenter Server labels the position of the detected face as well as the confidence on each frame based on the inference result, and sends the image information to the web Ul on the host so that the video face detection information can be viewed by browsing Presenter Server.

## Original model

To obtain the model architecture and the pre-trained model, visit the following website: https://github.com/opencv/opencv/tree/master/samples/dnn/face_detector

## Preprocessing

The model requires 300 x 300 images in BGR format, and mean subtraction is performed on the color channels. Digital vision preprocessing (DVPP) and artificial intelligence preprocessing (AIPP) are used for image preprocessing as follows:

DVPP:

1. Resizes images to 300 x 304. (DVPP requires heights rounded up to the nearest multiple of 2, and widths to 16, which leads to padding)
2. Outputs image data of the uint8 type.

AIPP:

1. Performs mean subtraction on color channels.
2. Crops images to 300 x 300 by removing the padding generated during DVPP.
3. Performs color space conversion (CSC) and converts images from YUV420SP to BGR.
4. Converts uint8 data to fp16 data (Da Vinci requires fp16 data as input for model reference).

## Postprocessing

The model has two outputs. The first output is the number of bounding boxes. The second output is the bounding box information, including the coordinates and the corresponding confidence of the detected face.

## Application performance

Duration per processing: (preprocessing -> inference -> postprocessing): 5 ms

Duration per inference: 0.36 ms