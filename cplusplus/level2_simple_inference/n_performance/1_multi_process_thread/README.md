English|[中文](README_CN.md)

# Multi-threaded high-performance samples

#### Directory structure and description
This warehouse contains a variety of multi-threaded samples for users' reference. The directory structure and specific instructions are as follows.
| sample  | description  | support chip |
|---|---|---|
| [YOLOV3_coco_detection_4_thread](./YOLOV3_coco_detection_4_thread)  | Use the yolov3 model to classify and infer the input video. C-style thread creation, fixed 4-way processing, output is screen printing information | Ascend310 |
| [YOLOV3_coco_detection_multi_thread_VENC](./YOLOV3_coco_detection_multi_thread_VENC)  | Use the yolov3 model to classify and infer the input video. C-style thread creation, fixed 2-way processing, output as VENC encoded video file | Ascend310 |
| [multi_channels_rtsp](./multi_channels_rtsp)  | Use the object detection model to perform object detection on two video streams at the same time, create threads by encapsulating classes, and output as the presenter interface display | Ascend310 |
| [yolov3_coco_detection_multi_thread](./yolov3_coco_detection_multi_thread)  | Use the object detection model to perform object detection on 1-10 video streams at the same time, the number of channels can be configured by yourself, and the output is displayed on the screen | Ascend310 |