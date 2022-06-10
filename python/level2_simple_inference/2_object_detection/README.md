English|[中文](README_CN.md)

# Detection samples

#### Directory structure and description
This catalog contains a variety of test samples for users' reference. The directory structure and specific instructions are as follows.

- **yolov3 series samples**
  | Sample name  | Sample description  | Characteristic analysis  | support chip |
  |---|---|---|---|
  | [YOLOV3_coco_detection_picture](./YOLOV3_coco_detection_picture)  | Picture detection  | The input and output are all JPG images, using the acllite public library, the model model is the yolov3-caffe network, and the data set is the coco data set  | Ascend310 |
  | [YOLOV3_coco_detection_picture<br>_with_postprocess_op](./YOLOV3_coco_detection_picture) | Picture detection | The input and output are all JPG images, using the acllite public library, the model model is the yolov3-tf network, the data set is the coco data set, and the post-processing uses a single operator model | Ascend310 |
  | [YOLOV3_mask_detection_picture](./YOLOV3_mask_detection_picture) | Picture detection | The input and output are all JPG images, using the acllite public library, the model model is the yolov3-tf network, and the data set is the mask recognition data set | Ascend310 |
  | [YOLOV3_mask_detection_video](./YOLOV3_mask_detection_video) | Video detection | The input is an mp4 file, the output is a presenter display, using the acllite public library, the model is the yolov3-tf network, and the data set is the mask recognition data set | Ascend310 |
  | [coco_detection_rtsp](./coco_detection_rtsp) | Video detection | The input is rtsp video stream or H264 file, the output is terminal interface screen display, using acllite public library, model is yolov3-caffe network, data set is coco data set | Ascend310 |

- **yolov4 series samples**
  | Sample name  | Sample description  | Characteristic analysis  | support chip |
  |---|---|---|---|
  | [YOLOV4_coco_detection_car_picture](./YOLOV4_coco_detection_car_picture) | Picture detection | The input and output are all JPG pictures, and the model is yolov4-onnx network. Output lane line and vehicle detection information | Ascend310 |
  | [YOLOV4_coco_detection_car_video](./YOLOV4_coco_detection_car_video) | Video detection | The input and output are all video files, and the model is yolov4-onnx network. Output lane line and vehicle detection information | Ascend310 |
  | [YOLOV4_coco_detection_picture](./YOLOV4_coco_detection_picture) | Video detection | The input and output are all video files, and the model is yolov4-onnx network. Output all test results of COCO data set | Ascend310 |