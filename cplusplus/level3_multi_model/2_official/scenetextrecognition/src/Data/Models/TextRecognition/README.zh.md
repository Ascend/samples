# 文字识别模型

## 原始模型网络介绍地址

https://github.com/HangZhouShuChengKeJi/chinese-ocr

## pb模型获取方法

参考地址 https://bbs.huaweicloud.com/forum/thread-77538-1-1.html

## 转换模型至昇腾om模型

由于文字检测模型输出的检测框长宽比例不同，需要设置动态分辨率保证推理结果的准确性。
其中dynamic_image_size的分辨率档次需要与Data/Config/setup.config中TextRecognition.dynamicWidthList保持一致。
```bash
atc --model=./chineseocr.pb \
    --framework=3 \
    --output=./chineseocr \
    --soc_version=Ascend310 \
    --insert_op_conf=./chineseocr_aipp.cfg \
    --input_shape="the_input:1,-1,-1,1" \
    --dynamic_image_size="32,32;32,64;32,96;32,128;32,160;32,192;32,224;32,256;32,288;32,320"
```

动态分辨率配置参数dynamic_image_size详细说明，参考 https://support.huawei.com/enterprise/zh/doc/EDOC1100150964/92c4f314

## 已验证的产品

- Atlas 800 (Model 3000)
- Atlas 800 (Model 3010)
- Atlas 300 (Model 3010)