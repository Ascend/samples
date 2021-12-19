English|[中文](README_CN.md)

# Classification example

#### Directory structure and description
This catalog contains a variety of classification samples for users' reference. The directory structure and specific instructions are as follows.

- **googlenet series sample**
  | Sample name  | Sample description  | Characteristic analysis |
  |---|---|---|
  | [googlenet_imagenet_picture](./googlenet_imagenet_picture)  | Picture Classification  | The input and output are all JPG images, and the model is the GoogLeNet model based on Caffe  |
  | [googlenet_mindspore_picture](./googlenet_mindspore_picture)  | Picture Classification  | Both input and output are JPG images, and the model is the GoogLeNet model based on MindSpore  |
  | [googlenet_onnx_picture](./googlenet_onnx_picture)  | Picture Classification  | Both input and output are JPG images, and the model is GoogLeNet model based on pytorch  |
  | [googlenet_imagenet_multi_batch](./googlenet_imagenet_multi_batch)  | Picture Classification  | The input and output are all JPG images, and the model is the GoogLeNet model based on Caffe, which uses the feature of multiple batches  |

- **resnet50 series sample**
  | Sample name  | Sample description  | Characteristic analysis |
  |---|---|---|
  | [resnet50_imagenet_classification](./resnet50_imagenet_classification)  | Picture Classification  | The input is a JPG picture, and the output is a screen print. Image classification based on Caffe ResNet-50 network (synchronous reasoning)  |
  | [resnet50_async_imagenet_classification](./resnet50_async_imagenet_classification)  | Picture Classification  | The input is a JPG picture, and the output is a screen print. Image classification based on Caffe ResNet-50 network (asynchronous reasoning)  |
  | [resnet50_mindspore_picture](./resnet50_mindspore_picture)  | Picture Classification  | Both input and output are JPG images. Use the MindSpore-based resnet50 model to classify and infer input images  |  
  | [vdec_resnet50_classification](./vdec_resnet50_classification)  | Picture Classification  | The input is an h264 file, and the output is a screen print. Image classification based on Caffe ResNet-50 network (video decoding + synchronous reasoning)  |
  | [vpc_jpeg_resnet50_imagenet_classification](./vpc_jpeg_resnet50_imagenet_classification)  | Picture Classification  | Input is YUV picture, output is screen printing/JPG picture. Realize image classification based on Caffe ResNet-50 network (image decoding + matting zoom + image encoding + synchronous reasoning) |
  | [vpc_resnet50_imagenet_classification](./vpc_resnet50_imagenet_classification)  | Picture Classification  | The input is a JPG picture, and the output is a screen print. Image classification based on Caffe ResNet-50 network (picture decoding + scaling + synchronous reasoning)  |

- **other sample**
  | sample | description  |
  |---|---|
  | [inceptionv3_picture](./inceptionv3_picture)  | Image classification example of IncpetionV3 model based on Pytorch framework  |
  | [lenet_mindspore_picture](./lenet_mindspore_picture)   | Image classification example of lenet model based on Mindspore framework  |
  | [vgg16_cat_dog_picture](./vgg16_cat_dog_picture)   | Example of cat and dog classification based on the vgg16 model of the caffe framework  |