# 雨天图像增强案例（输入：图片，输出：图片）

### 简介

本目录为atlas200dk演示工程，部署本案例前，请确保相应开发环境已配置正确（注：图片读写接口使用了opencv，所以请务必参考官方文档安装好ffmpeg和opencv）



### 相关配置

1. 如需计算PSNR、SSIM指标，请取消*classify_process.h Line 31*的注释；
2. 默认打印`./data`目录中所有测试样例的平均指标，如需单独打印每张图片的PSNR、SSIM指标，请取消*classify_process.cpp Line348*的注释；

