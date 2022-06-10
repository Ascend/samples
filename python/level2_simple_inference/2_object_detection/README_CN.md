中文|[English](README.md)

# 检测样例

#### 介绍
本目录包含多种检测样例，以供用户参考。目录结构和具体说明如下。

- **yolov3系列样例**
  | 样例名称  | 样例说明  | 特性解析  | 支持芯片 |
  |---|---|---|---|
  | [YOLOV3_coco_detection_picture](./YOLOV3_coco_detection_picture)  | 图片检测  | 输入输出均为JPG图片，使用acllite公共库，模型模型为yolov3-caffe网络，数据集为coco数据集  | Ascend310 |
  | [YOLOV3_coco_detection_picture<br>_with_postprocess_op](./YOLOV3_coco_detection_picture) | 图片检测 | 输入输出均为JPG图片，使用acllite公共库，模型模型为yolov3-tf网络，数据集为coco数据集，后处理使用单算子模型 | Ascend310 |
  | [YOLOV3_mask_detection_picture](./YOLOV3_mask_detection_picture) | 图片检测 | 输入输出均为JPG图片，使用acllite公共库，模型模型为yolov3-tf网络，数据集为口罩识别数据集 | Ascend310 |
  | [YOLOV3_mask_detection_video](./YOLOV3_mask_detection_video) | 视频检测 | 输入为mp4文件，输出为presenter展示，使用acllite公共库，模型为yolov3-tf网络，数据集为口罩识别数据集 | Ascend310 |
  | [coco_detection_rtsp](./coco_detection_rtsp) | 视频检测 | 输入为rtsp视频流或H264文件，输出为终端界面打屏显示，使用acllite公共库，模型为yolov3-caffe网络，数据集为coco数据集 | Ascend310 |

- **yolov4系列样例**
  | 样例名称  | 样例说明  | 特性解析  | 支持芯片 |
  |---|---|---|---|
  | [YOLOV4_coco_detection_car_picture](./YOLOV4_coco_detection_car_picture) | 图片检测 | 输入输出均为JPG图片，模型为yolov4-onnx网络。输出车道线和车辆检测信息 | Ascend310 |
  | [YOLOV4_coco_detection_car_video](./YOLOV4_coco_detection_car_video) | 视频检测 | 输入输出均为视频文件，模型为yolov4-onnx网络。输出车道线和车辆检测信息 | Ascend310 |
  | [YOLOV4_coco_detection_picture](./YOLOV4_coco_detection_picture) | 视频检测 | 输入输出均为视频文件，模型为yolov4-onnx网络。输出COCO数据集所有检测结果 | Ascend310 |