# 文字检测模型

## 原始模型网络介绍地址

https://github.com/huoyijie/AdvancedEAST

## pb模型获取方法

参考地址 https://bbs.huaweicloud.com/forum/thread-77532-1-1.html

## 转换模型至昇腾om模型

由于输入的图片长宽比例不同，需要设置动态分辨率保证推理结果的准确性。
其中dynamic_image_size的分辨率档次需要与Data/Config/setup.config中TextDetection.dynamicHWList设置保持一致。
```bash
atc --model=./advancedeast.pb \
    --framework=3 \
    --output=./advancedeast \
    --soc_version=Ascend310 \
    --insert_op_conf=./advancedeast_aipp.cfg \
    --input_shape="input_img:1,-1,-1,3" \
    --dynamic_image_size="832,832;416,832;832,416;416,416"
```

动态分辨率配置参数dynamic_image_size详细说明，参考 https://support.huawei.com/enterprise/zh/doc/EDOC1100150964/92c4f314

## 已验证的产品

- Atlas 800 (Model 3000)
- Atlas 800 (Model 3010)
- Atlas 300 (Model 3010)