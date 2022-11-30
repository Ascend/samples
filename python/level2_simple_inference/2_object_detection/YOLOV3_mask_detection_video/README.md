**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%20running%20sample%20guide/Running%20Video%20Samples%20in%20MindStudio).**

## Video Input Sample for Mask Recognition
Function: detects the face and face mask in a video, and recognizes the masked face.  
Input: video to be detected.   
Output: inference results displayed on the Presenter Server WebUI.

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item      | Requirement                                                        | Remarks                                                        |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CANN version  | ≥ 5.0.4                                                    | Install the CANN by referring to [Installation](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the *About Ascend Samples Repository*. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md).|
| Hardware  | Atlas200DK/Atlas300 ([Ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| opencv,python-acllite                                        | Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://github.com/Ascend/samples/tree/master/python/environment).|

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

2. Obtain the source model required by the application.
    | **Model**     | **Description**                         | **How to Obtain**                                            |
    | ----------------- | ------------------------------------- | ------------------------------------------------------------ |
    | yolo3_resnet18.pb | TensorFlow-YOLOV3-based mask detection model.| Download the model file and configuration file by referring to [README.md](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov3_resnet18/ATC_yolo3_resnet18_tf_AE). |

    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
    cd ${HOME}/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_video/model    
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3-RESNET18%20/yolo3_resnet18.pb    
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/YOLOV3-RESNET18%20/insert_op.cfg
    atc --model="./yolo3_resnet18.pb" --output_type=FP32 --input_shape="images:1,352,640,3" --insert_op_conf=./insert_op.cfg --input_format=NHWC --output="./yolo3_resnet18_yuv" --framework=3  --soc_version=Ascend310
    ```

3. Obtain the test videos required by the sample.
    ```
    Run the following commands to go to the data folder of the sample and download the corresponding test image:
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_video/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/mask_detection_video/mask.h264
    cd ../scripts
    ```
### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**  

1. Run the following commands to upload the **YOLOV3_mask_detection_video** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
    ```
    # In the following information, xxx.xxx.xxx.xxx is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_video HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/YOLOV3_mask_detection_video/scripts    
    ```

2. Run the sample.
   ```
   bash samples_run.sh
   ```

### Result Viewing

1. Open the Presenter Server WebUI.   
   - For Atlas 200 DK   
      Open the URL that is displayed when Presenter Server is started.    
   - For Atlas 300 AI accelerator card (AI1s cloud inference environment)   
      **The following assumes that the intranet IP address of the Atlas 300 AI accelerator card (AI1s) is 192.168.0.194 and the public IP address is 124.70.8.192.**     
      The message "Please visit http://192.168.0.194:7009 for display server" is displayed when Presenter Server is started.    
      Replace the intranet IP address **192.168.0.194** in the URL with the public IP address **124.70.8.192**. That is, change the URL to http://124.70.8.192:7009.      
      Open the URL in the browser.     
2. Wait for Presenter Agent to transmit data to the server and click **Refresh**. When data arrives, the icon in the **Status** column for the corresponding **Channel** changes to green.    
3. Click a link in the **View Name** column to view the result.     
![输入图片说明](https://images.gitee.com/uploads/images/2021/1029/162650_0e2aca00_8083019.png "微信图片_20211029162628.png")

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
