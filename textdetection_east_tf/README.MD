## 1、原始模型
https://github.com/argman/EAST, 下载对应的ckpt，转成pb模型

## 2、转om模型
obs链接：obs://nkxiaolei88/ATC Model/EAST/east_text_detection.om

ATC转换命令：
```
/home/HwHiAiUser/Ascend/ascend-toolkit/20.10.0.B023/atc/bin/atc --output_type=FP32 --input_shape="input_images:1,448,448,3" --check_report=/root/modelzoo/east_text_detection/device/network_analysis.report --input_format=NHWC --output="/root/modelzoo/east_text_detection/device/east_text_detection" --soc_version=Ascend310 --framework=3 --model="/home/HwHiAiUser/Orignal_Model/east_text_detection.pb" 
```

## 3、将下载的om文件放在model文件夹

## 4、编译msame推理工具
参考https://gitee.com/ascend/modelzoo/tree/develop/contrib/msame, 编译出msame推理工具

## 5、性能测试
使用msame推理工具，参考如下命令，发起推理性能测试： 

./msame --model model/east_text_detection.om --output output/ --loop 100
```
[INFO] output data success
Inference average time: 11.633170 ms
Inference average time without first time: 11.629354 ms
[INFO] unload model success, model Id is 1
[INFO] Execute sample success.
```
1Batch，shape:1x448x448x3，不带AIPP，平均推理性能11.63ms

## 6、精度测试：
暂未使用cidar2015数据集进行NPU精度测试

## 7、图片推理：
### 1）安装依赖
按照requirements.txt安装python3第三方库依赖
安装opencv后，可能会报module 'cv2' has no attribute 'imread'错误，需要用老版本的opencv：
pip3.7 uninstall opencv-python
pip3.7 install opencv-python==3.4.2.17
pip3.7 install opencv-contrib-python==3.4.2.17

### 2）下载图片
下载包含英文字符的图片（支持后缀.jpg、.JPEG、.png），存放在image_input目录中\
图片预处理：\
因为448x448分辨率的识别效果不佳，故参考第2步，将输入分辨率固定为：768x768 \
因为NPU的推理不支持动态Shape，所以固定输入shape为1x768x768x3，输入图片在preprocess.py中按照这个尺寸进行等比缩放和补齐
```
    if h <= resize_h and w <= resize_w:
        im = cv2.copyMakeBorder(im,0,resize_h-h,0,resize_w-w,cv2.BORDER_CONSTANT,value=(0,0,0))
        ratio_h = 1
        ratio_w = 1
    else:
        ratio_w = ratio_h = resize_h/max(h,w)
        im = cv2.resize(im, (math.floor(w*ratio_w), math.floor(h*ratio_h)))
        im = cv2.copyMakeBorder(im, 0, resize_h - math.floor(h*ratio_h), 0, resize_w - math.floor(w*ratio_w), cv2.BORDER_CONSTANT, value=(0, 0, 0))
```

### 3）执行推理
```
bash start_inference.sh
```
输出的图片和检测框信息存放在image_output目录
```
Test Start!
[INFO] acl init success
[INFO] open device 0 success
[INFO] create context success
[INFO] create stream success
[INFO] get run mode success
[INFO] load model model/east_text_detection.om success
[INFO] create model description success
[INFO] create model output success
[INFO] create model output success
[INFO] start to process file:input//string.jpg.bin
[INFO] model execute success
Inference time: 37.957ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//test1.jpg.bin
[INFO] model execute success
Inference time: 37.851ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//test11.jpg.bin
[INFO] model execute success
Inference time: 37.837ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//test2.jpg.bin
[INFO] model execute success
Inference time: 37.87ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//test4.jpg.bin
[INFO] model execute success
Inference time: 37.883ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//test7.jpg.bin
[INFO] model execute success
Inference time: 37.851ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//tiger.jpg.bin
[INFO] model execute success
Inference time: 37.909ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//timg (1).jpg.bin
[INFO] model execute success
Inference time: 37.824ms
output/
[INFO] output data success
[INFO] create model output success
[INFO] start to process file:input//timg.jpg.bin
[INFO] model execute success
Inference time: 37.887ms
output/
[INFO] output data success
[INFO] unload model success, model Id is 1
[INFO] Execute sample success.
Test Finish!
******************************
[INFO] end to destroy stream
[INFO] end to destroy context
[INFO] end to reset device is 0
[INFO] end to finalize acl
```
NPU推理时间为37ms

实测效果图：见image_output目录下的jpg文件。
