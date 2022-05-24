English|[中文](README_CN.md)
  
**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## Face Detection Example of RTSP Video Streams
Function: uses the face detection model to detect faces in real-time videos of a network camera.   
Input: RTSP video streams of a network camera.   
Output: semantic segmentation results displayed on the Presenter Server WebUI.   

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item| Requirement| Remarks|
|---|---|---|
| CANN version| ≥ 5.0.4| Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the About Ascend Samples Repository. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md).|
| Hardware| Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| python-acllite | Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://github.com/Ascend/samples/tree/master/python/environment).|

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE:** 
> - If you use a local video file for test, the video file must be a raw H.264 stream file in annex-b format. To convert an MP4 file to the H.264 file, run the following command:   
>   ffmpeg -i aaa.mp4 -codec copy -bsf: h264_mp4toannexb -f h264 aaa.h264    
> - Ensure that the MP4 file meet the requirements for using PyAV to read cut video frames.

### Sample Preparation

1. Obtain the source package.

   You can download the source code in either of the following ways:  
    - Command line (The download takes a long time, but the procedure is simple.)
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```
       **To switch to another tag (for example, v0.5.0), run the following command:**
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

2. Obtain the source network model required by the application.
    |  **Model** |  **Description** |  **How to Obtain** |
    |---|---|---|
    |  face_detection| Caffe-based face detection model.|  Download the model and weight file by referring to the links in **README.md** in the [ATC_resnet10-SSD_caffe_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/facedetection/ATC_resnet10-SSD_caffe_AE) directory of the ModelZoo repository.|
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp/model/    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection_fp32.caffemodel  
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/facedection/face_detection.prototxt
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection-python/insert_op.cfg
    atc --input_shape="data:1,3,300,300" --weight="./face_detection_fp32.caffemodel" --input_format=NCHW --output="./face_detection" --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --framework=0 --model="./face_detection.prototxt"
    ```

3. Obtain the test H.264 file required by the sample. 
    ```
    # Run the following commands to go to the **data** folder of the sample and download the corresponding test H.264 file:
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp/data
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face_detection_rtsp/person.h264
    cd ../src  
    ```

### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 3](#step_3) directly.**  

1. Modify the configuration file.

   Add the video file or RTSP address to be detected to the **[videostream]** field. In this example, a maximum of 6-channel video streams are supported.

2. Run the following commands to upload the **face_detection_rtsp** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
   ```
   # In the following information, *xxx.xxx.xxx.xxx* is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
   scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx
   cd $HOME/samples/python/level2_simple_inference/2_object_detection/face_detection_rtsp/script
   ```

3. <a name="step_3"></a>Run the executable file.
   ```
   bash sample_run.sh
   ```

### Result Viewing

1. Open the Presenter Server WebUI.   
   - For Atlas 200 DK   
     Open the URL that is displayed when Presenter Server is started.   
   - For Atlas 300 AI accelerator card (AI1s cloud inference environment)    
     **The following assumes that the intranet IP address of the Atlas 300 AI accelerator card (AI1s) is 192.168.0.194 and the public IP address is 124.70.8.192.**   
     When Presenter Server is started, a message "Please visit [http://192.168.0.194:7009](http://192.168.0.194:7009/) for display server" is displayed.    
     Replace the intranet IP address **192.168.0.194** in the URL with the public IP address **124.70.8.192**. That is, change the URL to http://124.70.8.192:7009.      
     Open the URL in the browser.   
2. Wait for Presenter Agent to transmit data to the server and click **Refresh**. When data arrives, the icon in the **Status** column for the corresponding **Channel** changes to green.  
3. Click a link in the **View Name** column to view the result.  

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
