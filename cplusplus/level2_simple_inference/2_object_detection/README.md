English|[中文](README_CN.md)

# Detection samples

#### Directory structure and description
This warehouse contains a variety of detection samples for users' reference. The directory structure and specific instructions are as follows.    
- **VGG_SSD series samples**
  | Sample name  | Sample description  | Characteristic analysis  | support chip |
  |---|---|---|---|
  | [VGG_SSD_coco_detection<br>_CV_with_AIPP](./VGG_SSD_coco_detection_CV_with_AIPP)  | Picture detection sample  | Both input and output are JPG images, and opencv is used for image encoding, decoding and cropping. AIPP is used for model conversion.  | Ascend310 |
  | [VGG_SSD_coco_detection<br>_CV_without_AIPP](./VGG_SSD_coco_detection_CV_without_AIPP)  | Picture detection sample  | The input and output are all JPG images, use opencv for image encoding, decoding and cropping, and AIPP function is not used during model conversion   | Ascend310 |
  | [VGG_SSD_coco_detection<br>_DVPP_with_AIPP](./VGG_SSD_coco_detection_DVPP_with_AIPP)  | Picture detection sample  |  The input and output are all JPG pictures, dvpp is used for picture decoding and cropping, opencv is used for picture coding, and the AIPP function is used for model conversion  | Ascend310 |

- **YOLO3 series samples**
  | Sample name  | Sample description  | Characteristic analysis  | support chip |
  |---|---|---|---|
  | [YOLOV3_coco_detection<br>_picture](./YOLOV3_coco_detection_picture)  | Picture detection sample  | The input and output are all JPG images, using dvpp for image decoding and cropping, using opencv for image encoding, not using the acllite public library, and the model is the yolov3-caffe network (modify the prototxt directly, and simplify the post-processing as an operator operation)  | Ascend310 |
  | [YOLOV3_VOC_detection<br>_picture](./YOLOV3_VOC_detection_picture)  | Picture detection sample  | The input and output are all JPG pictures, use dvpp for picture decoding and cropping, use opencv for picture encoding, use the acllite public library, and the model is yolov3-tf network (post-processing is implemented using code)  | Ascend310 |
  | [YOLOV3_coco_detection<br>_picture_with_freetype](./YOLOV3_coco_detection_picture_with_freetype)  | Picture detection sample  | Input is JPG picture, output is YUV file, use dvpp for picture encoding, decoding and cropping, use acllite public library, model is yolov3-caffe network, post-processing uses freetype to write text directly on YUV picture, and use memory modification method in YUV Picture frame  | Ascend310 |
  | [YOLOV3_coco_detection<br>_dynamic_AIPP](./YOLOV3_coco_detection_dynamic_AIPP)  | Picture detection sample  |  Both input and output are JPG images, dvpp is used for image decoding and cropping, opencv is used for image encoding, acllite public library is used, the model is yolov3-caffe network, and the dynamic AIPP feature is used  | Ascend310 |
  | [YOLOV3_dynamic_batch<br>_detection_picture](./YOLOV3_dynamic_batch_detection_picture)  | Picture detection sample  | The input is a BIN file, the output is screen printing, the acllite public library is not used, the model is the yolov3-caffe network, and the dynamic Batch/dynamic resolution feature is used | Ascend310 |
  | [YOLOV3_coco_detection<br>_video](./YOLOV3_coco_detection_video)  | Video detection example  | The input is mp4 video, and the output is the presenter interface display. Use opencv for video encoding and decoding and frame cropping, without using the acllite public library, the model is yolov3-caffe network | Ascend310 |
  | [YOLOV3_coco_detection<br>_VENC](./YOLOV3_coco_detection_VENC)  | Video detection example  | The input is mp4 video, the output is h264 file, opencv is used for video decoding and frame cropping, VENC is used for image encoding, easycl public library is not used, and the model is yolov3-caffe network  | Ascend310 |
   
- **Other samples**
  | sample  | description  | support chip |
  |---|---|---|
  | [YOLOV4_coco_detection<br>_picture](./YOLOV4_coco_detection_picture)  | Use the yolov4 model to perform predictive inference on the input picture, and print the result on the output picture.  | Ascend310 |
