English|[中文](README_CN.md)
  
**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## Video Object Detection Sample
Function: Detect objects from two video stream inputs at the same time by using an object detection model.   
Input: RTSP video stream or MP4 video file of a network camera.  
Output: detection result displayed on the Presenter Server WebUI.  

**Note: This sample supports RTSP video streams output by Hikvision network cameras. The support for video streams pushed by streaming tools will be added.**

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item      | Requirement                                                        | Remarks                                                        |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CANN version  | ≥ 5.0.4                                                     | Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the *About Ascend Samples Repository*. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md).|
| Hardware  | Atlas 200 DK/Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))| Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| presentagent, ffmpeg+acllite                                 | For details, see [Third-Party Dependency Installation Guide (C++ Sample)](../../../../environment).|

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
        # 1. Click **Clone** or **Download** in the upper right corner of the samples repository and click **Download ZIP**.   
        # 2. Upload the .zip package to the home directory of a common user in the development environment, for example, **${HOME}/ascend-samples-master.zip**.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
    ```
   
2. Convert the model.    
    | **Model**  | **Description**                                   | **How to Obtain**                                            |
    | -------------- | ----------------------------------------------- | ------------------------------------------------------------ |
    | face_detection | Image classification inference model. It is a ResNet SSD model based on Caffe.| Download the model and weight file by referring to the links in **README.md** in the [ATC_resnet10-SSD_caffe_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/facedetection/ATC_resnet10-SSD_caffe_AE) directory of the ModelZoo repository.|
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
    
    cd ${HOME}/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp/model  
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/insert_op.cfg
    atc --model=./face_detection.prototxt --weight=./face_detection_fp32.caffemodel --framework=0 --output=./face_detection --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg
    ```
### Sample Deployment
1. Modify the configuration file **multi_channels_rtsp.conf**.
   ``` 
   vi ${HOME}/multi_channels_rtsp/scripts
   ```
   Set **rtspaddr** and **display** as required. By default, there is only one **rtsp_0** stream, which is not displayed.
   ```
   [rtspaddr]
   #rtsp_0=rtsp://192.168.23.1.030/test0.sdp
   rtsp_0=../data/person.mp4
   #rtsp_1=xxxxx

   [display]
   display_channel=0
   ```
   - **rtsp_x**: input in the *x*-th channel. To configure more channels, add input addresses such as **rtsp_1** and **rtsp_2** to the configuration file.    
   - **display_channel**: channel number of the video to be displayed. If **display_channel** is commented out, the corresponding video will not be displayed on the Presenter Server WebUI. To display the video, delete the comment tag (#) before **display_channel** and specify the channel number of the video to be displayed, for example, **display_channel=0**.  

2. Run the following commands to execute the compilation script to start sample compilation: By default, the preset data in **person.mp4** is downloaded. 
   ```
   cd ${HOME}/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp/scripts    
   bash sample_build.sh
   ```

### Sample Running
**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**      

1. Run the following commands to upload the **multi_channels_rtsp** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):   
    ```
    # In the following information, *xxx.xxx.xxx.xxx* is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r ${HOME}/samples/cplusplus/level2_simple_inference/n_performance/1_multi_process_thread/multi_channels_rtsp HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser    
    ssh HwHiAiUser@xxx.xxx.xxx.xxx     
    cd ${HOME}/multi_channels_rtsp/scripts
    ```

2. <a name="step_2"></a>Execute the script to run the sample.        
    ```
    bash sample_run.sh
    ```

### Result Viewing
1. Open the Presenter Server WebUI.   
   - For Atlas 200 DK:   
      Open the URL that is displayed when Presenter Server is started.   
   - For Atlas 300 AI accelerator card (AI1s cloud inference environment):   
      **The following assumes that the intranet IP address of the Atlas 300 AI accelerator card (AI1s) is 192.168.0.194 and the public IP address is 124.70.8.192.**   
      The message "Please visit http://192.168.0.194:7009 for display server" is displayed when Presenter Server is started.   
      Replace the intranet IP address **192.168.0.194** in the URL with the public IP address **124.70.8.192**. That is, change the URL to http://124.70.8.192:7009.   
      Open the URL in the browser.   
2. Wait for Presenter Agent to transmit data to the server and click **Refresh**. When data arrives, the icon in the **Status** column for the corresponding **Channel** changes to green.   
3. Click a link in the **View Name** column to view the result.   

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
