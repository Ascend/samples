#  dvpp_samples-Atlas200DK

#### 介绍
本仓包含Atlas200支持的媒体数据处理功能，各文件夹对应不同功能，以供用户参考。具体说明如下。

1. [crop](https://gitee.com/ascend/samples/blob/master/dvpp_samples/for_atlas300_1.7x.0.0_c++/crop/readme.md)：从输入图片中抠出需要用的图片区域，支持一图多框和多图多框。

2. [cropandpaste](https://gitee.com/ascend/samples/blob/master/dvpp_samples/for_atlas300_1.7x.0.0_c++/cropandpaste/readme.md)：从输入图片中抠出来的图，对抠出的图进行缩放后，放在用户输出图片的指定区域，输出图片可以是空白图片（由用户申请的空输出内存产生的），也可以是已有图片（由用户申请输出内存后将已有图片读入输出内存），只有当输出图片是已有图片时，才表示叠加。

3. [jpegd](https://gitee.com/ascend/samples/blob/master/dvpp_samples/for_atlas300_1.7x.0.0_c++/jpegd/readme.md)：实现.jpg、.jpeg、.JPG、.JPEG图片的解码，对于硬件不支持的格式，会使用软件解码。

4. [jpege](https://gitee.com/ascend/samples/tree/master/dvpp_samples/for_atlas300_1.7x.0.0_c++/jpege)：将YUV格式图片编码成JPEG压缩格式的图片文件。

5. [resize](https://gitee.com/ascend/samples/blob/master/dvpp_samples/for_atlas300_1.7x.0.0_c++/resize/readme.md)：     
针对不同分辨率的图像，VPC的处理方式可分为：
非8K缩放，用于处理“widthStride在32到4096（包括4096）范围内，heightStride在6到4096”的输入图片，不同格式的输入图片，widthStride的取值范围不同。
8K缩放，用于处理“widthStride在4096~8192范围内或heightStride在4096到8192范围内（不包括4096）”的输入图片。
从是否抠多张图的维度，可分为单图裁剪缩放（支持非压缩格式）、一图多框裁剪缩放（支持非压缩格式）。
其它缩放方式，如：原图缩放。

6. [vdec](https://gitee.com/ascend/samples/blob/master/dvpp_samples/for_atlas300_1.7x.0.0_c++/vdec/readme.md)：实现视频的解码，VDEC内部经过VPC处理后，输出YUV420SP格式（包括NV12和NV21）的图片。。