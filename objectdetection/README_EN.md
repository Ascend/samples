EN|[中文](README.md)

# Image Object Detection (Input: Image; Output: Image)

## Overview

Object detection is one of the basic tasks in computer vision field and has been under academia research for nearly 20 years. With the growth of deep learning technology in recent years, object detection algorithms have shifted from the traditional way based on handwork features to the detection technology based on deep neural network. This application provides a demo for developing an object detection network application with the Atlas 200 DK or AI acceleration cloud server.

This application runs on the Atlas 200 DK or AI acceleration cloud server to detect objects in local images using the YOLOv3 network. The confidence score and the bounding box of the object are displayed.

## Overall Design

![Input image description](https://images.gitee.com/uploads/images/2020/0811/091631_097fef9d_5408865.png "1.png")

1. **Allocate runtime resources**: Initialize system resources.

2. **Load the model and create output**: Load the offline model to the memory managed by the user, and the model information can be obtained, including the buffer sizes of the input and output data. Based on the model information, create model output for inference.

3. **Read and preprocess local images**: Loop over JPEG images in a local directory, use the JPEGD function of digital vision preprocessing (DVPP) to decode JPG images into YUV420SP format, use the vision preprocessing core (VPC) function of DVPP to resize images to the required image size, and create model input data.

4. **Perform model inference**: Perform model inference with the model input data.

5. **Parse the inference result:** Based on the model output, obtain the bounding boxes, classes, and confidence scores. Use OpenCV to label the detection result on the source image and write the result to a local file.

## Model Architecture

![Input image description](https://images.gitee.com/uploads/images/2020/0811/091907_8e720f86_5408865.jpeg "yolov3_416网络结构图.jpg")

## Original Model

To obtain the model architecture and pre-trained model, visit the following website: https://github.com/maxuehao/YOLOV3

## Image Preprocessing

The model requires 416 x 416 images in BGR format, and mean subtraction is performed on the color channels. Digital vision preprocessing (DVPP) and artificial intelligence preprocessing (AIPP) are used for image preprocessing as follows:

DVPP:

1. Decodes JPEG images into YUV420SP format.
2. Resizes images to 416 x 416.
3. Outputs image data of the uint8 type.

AIPP:

1. Performs color space conversion (CSC) by converting YUV to BGR format.
2. Performs mean subtraction.
3. Converts uint8 data to fp16 data (Da Vinci requires fp16 data as input for model reference).

## Image Postprocessing

![Input image description](https://images.gitee.com/uploads/images/2020/0811/092217_5fa87d6b_5408865.png "image-20200810195221950.png")

Based on the result and your understanding of the network, the following information can be obtained:

The second output indicates the number of detected objects in the current image.

The first output indicates the information about all bounding boxes.

As shown in the preceding figure, when the number of detected objects in the image is ***n***, the information about the ***i***th (****i**** starts from 0) object is as follows:

X-coordinate of the upper left corner of the bounding box (relative to the width of the model input image). The position offset is [*n**0 + i].

Y-coordinate of the upper left corner of the bounding box (relative to the height of the model input image). The position offset is [*n**1 + i].

X-coordinate of the lower right corner of the bounding box (relative to the width of the model input image). The position offset is [*n**2 + i].

Y-coordinate of the lower right corner of the bounding box (relative to the height of the model input image). The position offset is [*n**3 + i].

Confidence of the object in the bounding box. The position offset is [*n**4 + i].

Class index of the object in the bounding box. The position offset is [*n**5 + i].

**For example, the information about the second object detected in an image is as follows:**

X-coordinate of the upper left corner of the bounding box (relative to the width of the model input image) [3*0 + 1]: 295.000

Y-coordinate of the upper left corner of the bounding box (relative to the height of the model input image) [3*1 + 1]: 46.719

X-coordinate of the lower right corner of the bounding box (relative to the width of the model input image) [3*2 + 1]: 389.750

Y-coordinate of the lower right corner of the bounding box (relative to the height of the model input image) [3*3 + 1]: 352.250

Confidence of the object in the bounding box [3*4 + 1]: 0.984

Class index of the object in the bounding box [3*5 + 1]: 14

## Application Performance

Duration per processing loop (preprocessing -> inference -> postprocessing): 16.39 ms

Duration per inference loop: 12 ms

## Optimization

The inference is fast, while the preprocessing and postprocessing take a long time, leading to a long duration per processing loop. The preprocessing and postprocessing can be optimized by replacing OpenCV with digital vision preprocessing (DVPP), or using a single operator for postprocessing to improve efficiency.