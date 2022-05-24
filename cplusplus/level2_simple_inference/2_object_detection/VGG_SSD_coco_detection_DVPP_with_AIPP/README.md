English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**   
**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## VGG_SSD_coco_detection_DVPP_with_AIPP Sample
Function: Use the VGG_SSD model to perform prediction and inference on input images and print the results on the output images. In this case, the DVPP mode is used to process the input images.   
Input: source JPG image.    
Output: JPG image with the inference result.   
### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item| Requirement| Remarks|
|---|---|---|
| CANN version| ≥ 5.0.4| Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the *About Ascend Samples Repository*. If the CANN version is earlier than the required version, switch to the **samples** repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md).|
| Hardware| Atlas 200 DK/Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| opencv | For details, see [Third-Party Dependency Installation Guide (C++ Sample)](../../../environment).|

### Sample Preparation

1. Obtain the source package.

   You can download the source code in either of the following ways:  
    - Command line (The download takes a long time, but the procedure is simple.)
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```
       **Note: To switch to another tag (for example, v0.5.0), run the following command:**
       ```
       git checkout v0.5.0
       ```   
    - Compressed package (The download takes a short time, but the procedure is complex.)  
       **Note: If you want to download the code of another version, switch the branch of the samples repository according to the prerequisites.**  
       ``` 
        # 1. Click Clone or Download in the upper right corner of the samples repository and click Download ZIP.   
        # 2. Upload the .zip package to the home directory of a common user in the development environment, for example, ${HOME}/ascend-samples-master.zip.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
        ```

   2. Convert the model.
      | **Model**| **Description**                                | **How to Obtain**                                            |
      | ------------ | -------------------------------------------- | ------------------------------------------------------------ |
      | VGG_SSD     | Image detection inference model. It is a VGG_SSD model based on Caffe.| Download the model and weight file by referring to the links in **README.md** in the [ATC_vgg_ssd_caffe_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/vgg_ssd/ATC_vgg_ssd_caffe_AE) directory of the ModelZoo repository.|

      ```
      # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
      cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP/model     
      wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/VGG_SSD/vgg_ssd.caffemodel
      wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/VGG_SSD/vgg_ssd.prototxt
      wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/VGG_SSD_coco_detection_DVPP_with_AIPP/insert_op.cfg
      atc --output_type=FP32 --input_shape="data:1,3,300,300" --weight=./vgg_ssd.caffemodel --input_format=NCHW --output=./vgg_ssd --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --framework=0 --save_original_model=false --model=./vgg_ssd.prototxt
      ```

### Sample Deployment
Run the following commands to execute the compilation script to start sample compilation:  
```
cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP/scripts  
bash sample_build.sh
```

### Sample Running
**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   
1. Run the following commands to upload the **VGG_SSD_coco_detection_DVPP_with_AIPP** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):

   ```
   # In the following information, xxx.xxx.xxx.xxx is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
   scp -r $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx
   cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/VGG_SSD_coco_detection_DVPP_with_AIPP/scripts   
   ```

2. <a name="step_2"></a>Execute the script to run the sample.        
   ```
   bash sample_run.sh
   ```

### Result Viewing
After the running is complete, an inferred image is generated in the **out/output** directory of the sample project. The comparison is as follows:

![输入图片说明](https://images.gitee.com/uploads/images/2021/1027/162137_91c92f13_7647177.png "image-20211027102217540.png")


### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
