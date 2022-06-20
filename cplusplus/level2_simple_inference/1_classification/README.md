English|[中文](README_CN.md)

# Classification samples

#### Directory structure and description
This warehouse contains a variety of classification samples for users' reference. The directory structure and specific instructions are as follows.

- **googlenet series sample**
  | Sample name  | Sample description  | Characteristic analysis  | support chip |
  |---|---|---|---|
  | [googlenet_imagenet_picture](./googlenet_imagenet_picture)  | Picture classification example  | Input and output are pictures  | Ascend310 |
  | [googlenet_imagenet_multi_batch](./googlenet_imagenet_multi_batch)  | Picture classification example  | The input is a bin file, and the output is the result of inference, using the feature of multiple batches  | Ascend310 |
  | [googlenet_imagenet_dynamic_batch](./googlenet_imagenet_dynamic_batch)  | Picture classification example  | The input is a bin file, and the output is the result of inference, using the characteristics of dynamic batch性  | Ascend310 |
  | [googlenet_imagenet_video](./googlenet_imagenet_video)  | Video classification example  | Input mp4 file, output as presenter interface display  | Ascend310 |

- **resnet50 series sample**
  | Sample name  | Sample description  | Characteristic analysis  | support chip |
  |---|---|---|---|
  | [resnet50_imagenet_classification](./resnet50_imagenet_classification)  | Picture classification example  | Realize the function of image classification (synchronous reasoning) based on Caffe ResNet-50 network (single input, single batch)  | Ascend310，Ascend310P，Ascend910 |
  | [resnet50_async_imagenet_classification](./resnet50_async_imagenet_classification)  | Picture classification example  | Realize the function of image classification (asynchronous reasoning) based on Caffe ResNet-50 network (single input, single batch)  | Ascend310，Ascend310P，Ascend910 |
  | [vdec_resnet50_classification](./vdec_resnet50_classification)  | Picture classification example  | Image classification based on Caffe ResNet-50 network (video decoding + synchronous reasoning)  | Ascend310，Ascend310P，Ascend910 |
  | [vpc_jpeg_resnet50_imagenet_classification](./vpc_jpeg_resnet50_imagenet_classification)  | Picture classification example  | Realize image classification based on Caffe ResNet-50 network (image decoding + matting zoom + image encoding + synchronous reasoning)  | Ascend310，Ascend310P，Ascend910 |
  | [vpc_resnet50_imagenet_classification](./vpc_resnet50_imagenet_classification) | Picture classification example | Image classification based on Caffe ResNet-50 network (picture decoding + scaling + synchronous reasoning) | Ascend310，Ascend310P，Ascend910 |