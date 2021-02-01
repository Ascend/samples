# Text Detection Model

## Original Network Link

https://github.com/huoyijie/AdvancedEAST

## pb Model Link:

Instructions: https://bbs.huaweicloud.com/forum/thread-77532-1-1.html

## Convert model To Ascend om file

The height and width ratio of input image is different, we need to set a dynamic image size to ensure the accuracy of the inference result.
The value of dynamic_image_size should be the same as TextDetection.dynamicHWList in Data/Config/setup.config.
```bash
atc --model=./advancedeast.pb \
    --framework=3 \
    --output=./advancedeast \
    --soc_version=Ascend310 \
    --insert_op_conf=./advancedeast_aipp.cfg \
    --input_shape="input_img:1,-1,-1,3" \
    --dynamic_image_size="832,832;416,832;832,416;416,416"
```

The description of the parameter dynamic_image_size, please refer to https://support.huawei.com/enterprise/en/doc/EDOC1100150947/42020739/restrictions-and-parameters

## Products that have been verified:

- Atlas 800 (Model 3000)
- Atlas 800 (Model 3010)
- Atlas 300 (Model 3010)