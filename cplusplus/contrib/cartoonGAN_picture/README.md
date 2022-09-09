[中文](README_CN.md)|English

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## Sample of Cartoon Image Generation
Function: Cartoonize input images by using the CartoonGAN model.  
Input: Source JPG image.  
Output: JPG image with the inference result.   

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)

| Item| Requirement| Remarks|
|---|---|---|
| CANN version| >=5.0.4 | Install the CANN by referring to [Installation](/README.md#installation) in the *About Ascend Samples Repository*. If the CANN version is earlier than the required version, switch to the **samples** repository specific to the CANN version. See [Release Notes](/README.md#release-notes).|
| Hardware| Atlas 200 DK/Atlas 300 (ai1s)  | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| opencv, ffmpeg+acllite | For details, see [Third-Party Dependency Installation Guide (C++ Sample)](../../environment/README.md).|

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
        # 1. Click "Clone or Download" in the upper right corner of the samples repository and click "Download ZIP".   
        # 2. Upload the ZIP package to the home directory of a common user in the development environment, for example, "${HOME}/ascend-samples-master.zip".    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
        ```

2. Convert the model.      
    - Version 3.1.0
      This model cannot be correctly converted in version 3.1.0. You can directly obtain the OM model for version 3.1.0.    
      ``` 
        cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/model
        wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoonGAN_picture/cplus/cartoonization.om
      ```

    - Version 3.0.0      
        | **Model** | **Description** | **How to Obtain** |
        |---|---|---|
        | cartoonization | Image generation inference model.  | For details, see the model file in section about downloading the original model in the README.md of [ATC_cartoonization_tf_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/cartoonization/ATC_cartoonization_tf_AE). |

        ```
        # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
        cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/model     
        wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/cartoon/cartoonization.pb
        atc --output_type=FP32 --input_shape="train_real_A:1,256,256,3" --input_format=NHWC --output="./cartoonization" --soc_version=Ascend310 --framework=3 --save_original_model=false --model="./cartoonization.pb" --precision_mode=allow_fp32_to_fp16
        ```

### Sample Deployment
 Run the following commands to execute the compilation script to start sample compilation:    
```
cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/scripts   
bash sample_build.sh
```

### Sample Running
**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**       
1. Run the following commands to upload the **cartoonGAN_picture** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):    
   ```
   # In the following information, **xxx.xxx.xxx.xxx** is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
   scp -r $HOME/samples/cplusplus/contrib/cartoonGAN_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx
   cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/scripts   
   ```

2. <a name="step_2"></a>Execute the script to run the sample.           
   ```
   bash sample_run.sh
   ```

### Result Viewing
After the running is complete, the inference result is displayed in the CLI of the operating environment, and the inferred image is generated in the **$HOME/cartoonGAN_picture/out/output** directory.
![Input Image Description](https://images.gitee.com/uploads/images/2021/1115/162454_566f1288_7647177.png "image-20211109140820003.png")

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
