中文|[English](README.md)

# 检测样例

#### 介绍
本仓包含多种检测样例，以供用户参考。目录结构和具体说明如下。    
- **VGG_SSD系列样例**
  | 样例名称  | 样例说明  | 特性解析  | 支持芯片 |
  |---|---|---|---|
  | [VGG_SSD_coco_detection<br>_CV_with_AIPP](./VGG_SSD_coco_detection_CV_with_AIPP)  | 图片检测样例  | 输入输出均为JPG图片，使用opencv进行图片编解码及裁剪，模型转换时使用了AIPP功能  | Ascend310 |
  | [VGG_SSD_coco_detection<br>_CV_without_AIPP](./VGG_SSD_coco_detection_CV_without_AIPP)  | 图片检测样例  | 输入输出均为JPG图片，使用opencv进行图片编解码及裁剪，，模型转换时不使用AIPP功能   | Ascend310 |
  | [VGG_SSD_coco_detection<br>_DVPP_with_AIPP](./VGG_SSD_coco_detection_DVPP_with_AIPP)  | 图片检测样例  |  输入输出均为JPG图片，使用dvpp进行图片解码及裁剪，使用opencv进行图片编码，模型转换时使用了AIPP功能  | Ascend310 |

- **YOLO3系列样例**
  | 样例名称  | 样例说明  | 特性解析  | 支持芯片 |
  |---|---|---|---|
  | [YOLOV3_coco_detection<br>_picture](./YOLOV3_coco_detection_picture)  | 图片检测样例  | 输入输出均为JPG图片，使用dvpp进行图片解码及裁剪，使用opencv进行图片编码，未使用acllite公共库，模型为yolov3-caffe网络（直接修改prototxt，简化后处理为算子操作）  | Ascend310 |
  | [YOLOV3_VOC_detection<br>_picture](./YOLOV3_VOC_detection_picture)  | 图片检测样例  | 输入输出均为JPG图片，使用dvpp进行图片解码及裁剪，使用opencv进行图片编码，使用acllite公共库，模型为yolov3-tf网络（后处理使用代码实现）  | Ascend310 |
  | [YOLOV3_coco_detection<br>_picture_with_freetype](./YOLOV3_coco_detection_picture_with_freetype)  | 图片检测样例  | 输入为JPG图片，输出为YUV文件，使用dvpp进行图片编解码及裁剪，使用acllite公共库，模型为yolov3-caffe网络，后处理使用freetype直接在YUV图片上写文字，并使用修改内存方式在YUV图片上画框  | Ascend310 |
  | [YOLOV3_coco_detection<br>_dynamic_AIPP](./YOLOV3_coco_detection_dynamic_AIPP)  | 图片检测样例  |  输入输出均为JPG图片，使用dvpp进行图片解码及裁剪，使用opencv进行图片编码，使用acllite公共库，模型为yolov3-caffe网络，使用了动态AIPP特性  | Ascend310 |
  | [YOLOV3_dynamic_batch<br>_detection_picture](./YOLOV3_dynamic_batch_detection_picture)  | 图片检测样例  | 输入为BIN文件，输出为屏幕打印，未使用acllite公共库，模型为yolov3-caffe网络，使用了动态Batch/动态分辨率特性 | Ascend310 |
  | [YOLOV3_coco_detection<br>_video](./YOLOV3_coco_detection_video)  | 视频检测样例  | 输入为mp4视频，输出为presentser界面展示。 使用opencv进行视频编解码及帧裁剪，未使用acllite公共库，模型为yolov3-caffe网络 | Ascend310 |
  | [YOLOV3_coco_detection<br>_VENC](./YOLOV3_coco_detection_VENC)  | 视频检测样例  | 输入为mp4视频，输出为h264文件，使用opencv进行视频解码及帧裁剪，使用VENC进行图片编码，未使用acllite公共库，模型为yolov3-caffe网络  | Ascend310 |
   
- **其它样例**
  | 样例  | 说明  | 支持芯片 |
  |---|---|---|
  | [YOLOV4_coco_detection<br>_picture](./YOLOV4_coco_detection_picture)  | 使用yolov4模型对输入图片进行预测推理，并将结果打印到输出图片上。  | Ascend310 |
