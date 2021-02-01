# Text Recognition Model

## Original Network Link

https://github.com/HangZhouShuChengKeJi/chinese-ocr

## pb Model Link:

Instructions: https://bbs.huaweicloud.com/forum/thread-77538-1-1.html

## Convert model To Ascend om file

The height and width ratio of Text Detection output is different, we need to set a dynamic image size to ensure the accuracy of the inference result.
The value of dynamic_image_size should be the same as TextRecognition.dynamicWidthList in Data/Config/setup.config.
```bash
atc --model=./chineseocr.pb \
    --framework=3 \
    --output=./chineseocr \
    --soc_version=Ascend310 \
    --insert_op_conf=./chineseocr_aipp.cfg \
    --input_shape="the_input:1,-1,-1,1" \
    --dynamic_image_size="32,32;32,64;32,96;32,128;32,160;32,192;32,224;32,256;32,288;32,320"
```

The description of the parameter dynamic_image_size, please refer to https://support.huawei.com/enterprise/en/doc/EDOC1100150947/42020739/restrictions-and-parameters

## Products that have been verified:

- Atlas 800 (Model 3000)
- Atlas 800 (Model 3010)
- Atlas 300 (Model 3010)