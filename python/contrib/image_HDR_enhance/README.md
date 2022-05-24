English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in the command line. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Running%20Image%20Samples%20in%20MindStudio?sort_id=3736297).**

**Thanks for the sample contribution of Shenzhen University.**

## image_HDR_enhance Sample

Function: Use the model to enhance the HDR effect of underexposure images.   
Input: PNG image.    
Output: enhanced PNG image.

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item      | Requirement                                                        | Remarks                                                        |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CANN version  | ≥ 5.0.4                                                    | Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the *About Ascend Samples Repository*. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md).|
| Hardware  | Atlas200DK/Atlas300([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| OpenCV, python-acllite                                      | Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://github.com/Ascend/samples/tree/master/python/environment).|

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

2. Obtain the model required by the application.
    | **Model**     | **Description**           | **How to Obtain**                                            |
    | ----------------- | ----------------------- | ------------------------------------------------------------ |
    | image_HDR_enhance | TensorFlow-based HDR enhancement.| Download the model and weight files by referring to the links in **README.md** in the [ATC_VGG16_TensorFlow_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/image_HDR_enhance/ATC_VGG16_TensorFlow_AE) directory of the ModelZoo repository.|
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
    cd ${HOME}/samples/python/contrib/image_HDR_enhance/model    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/image_HDR_enhance/image_HDR_enhance.pb   
    atc --model=./image_HDR_enhance.pb --framework=3 --output=image_HDR_enhance --soc_version=Ascend310  --input_shape="input:1,512,512,3" --input_format=NHWC --output_type=FP32
    ```
3. Obtain the test image required by the sample and switch the folder.
    ```
    # Run the following commands to go to the data folder of the sample and download the corresponding test images:
    cd ${HOME}/samples/python/contrib/image_HDR_enhance/data
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/image_HDR_enhance/data1.png
    cd ../src
    ```

### Sample Running
**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to step 2 directly.**   
1. Run the following commands to upload the **edge_detection_picture** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
    ```
    # In the following information, xxx.xxx.xxx.xxx is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r $HOME/samples/python/contrib/image_HDR_enhance/  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/image_HDR_enhance/src    
    ```
2. Run the executable file.
    ```
    python3.6 main.py
    ```

### Result Viewing
After the execution is complete, the inference result is displayed on the CLI of the operating environment.    
![输入图片说明](https://images.gitee.com/uploads/images/2021/1109/150638_2d471e4b_5400693.png "屏幕截图.png")
![输入图片说明](https://images.gitee.com/uploads/images/2021/1109/150627_b2f8f08e_5400693.png "屏幕截图.png")

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
