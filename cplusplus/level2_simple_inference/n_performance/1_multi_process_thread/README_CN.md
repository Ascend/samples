中文|[English](README.md)

# 多线程高性能样例

#### 介绍
本仓包含多种多线程样例，以供用户参考。目录结构和具体说明如下。
| 样例  | 说明  | 支持芯片 |
|---|---|---|
| [YOLOV3_coco_detection_4_thread](./YOLOV3_coco_detection_4_thread)  | 使用yolov3模型对输入视频进行分类推理。类C风格的线程创建，固定4路处理，输出为打屏信息|
| [YOLOV3_coco_detection_multi_thread_VENC](./YOLOV3_coco_detection_multi_thread_VENC)  | 使用yolov3模型对输入视频进行分类推理。类C风格的线程创建，固定2路处理，输出为VENC编码后的视频文件 | Ascend310 |
| [multi_channels_rtsp](./multi_channels_rtsp)  | 使用物体检测模型同时对两路视频流进行物体检测，封装类的方式创建线程，输出为presenter界面展示 | Ascend310 |
| [yolov3_coco_detection_multi_thread](./yolov3_coco_detection_multi_thread)  | 使用物体检测模型同时对1-10路视频流进行物体检测，可自行配置路数，输出为打屏展示 | Ascend310 |
| [deeplabv3_multi_thread_one_device](./deeplabv3_multi_thread_one_device)  | 使用deeplabv3模型在单device上对多张图片进行语义分割，输出为多张图片 | Ascend310 |
| [deeplabv3_multi_thread_multi_device](./deeplabv3_multi_thread_multi_device)  | 使用deeplabv3模型在多device上对多张图片进行语义分割，可自行配置device数，输出为多张图片 | Ascend310 |
| [deeplabv3_multi_thread_multi_device_video](./deeplabv3_multi_thread_multi_device_video)  | 使用deeplabv3模型在多device上对一路视频进行语义分割，可自行配置device数，输出为MP4视频文件 | Ascend310 |
| [deeplabv3_multi_thread_multi_device_presenter](./deeplabv3_multi_thread_multi_device_presenter)  | 使用deeplabv3模型在多device上对一路视频进行语义分割，可自行配置device数，输出为presenter界面展示| Ascend310 |