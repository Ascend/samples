#  dvpp接口案例

#### 介绍
本仓包含Atlas支持的媒体数据处理功能，各文件夹对应不同功能，以供用户参考。目录结构与具体说明如下。

**./**   
├── [crop](./crop)：**crop接口，从输入图片中抠出需要用的图片区域**    
│   ├── data   
│   ├── model   
│   └── src   
├── [cropandpaste](./cropandpaste)：**cropandpaste接口，：从输入图片中抠出来的图，对抠出的图进行缩放后，放在用户输出图片的指定区域。**     
│   ├── data   
│   ├── model   
│   └── src   
├── [jpegd](./jpegd)：**jpegd接口，实现.jpg、.jpeg、.JPG、.JPEG图片的解码。**     
│   ├── data   
│   ├── model   
│   └── src   
├── [jpege](./jpege)：**jpege接口，将YUV格式图片编码成JPEG压缩格式的图片文件。**    
│   ├── data   
│   ├── model   
│   └── src   
├── [resize](./resize)：**resize接口。针对图像做缩放操作**    
│   ├── data   
│   ├── model   
│   └── src   
├── [vdec](./vdec)：**vdec接口，实现视频的解码，输出YUV420SP格式（包括NV12和NV21）的图片。**     
│   ├── data   
│   ├── model   
│   └── src   
├── [venc](./venc)：**venc接口，将YUV420SP图片数据编码成H264/H265格式的视频码流。**      
│   ├── data   
│   ├── model   
└   └── src   