English|[中文](README_CN.md)
   
**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## RTSP Video Stream Object Detection Example
Function: uses the YOLOv3 object detection model to detect objects in the input video. 
Input: RTSP video stream or H.264 file of an IP camera (IPC).  
Output: The test result is directly displayed on the terminal.

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item| Requirement| Remarks|
|---|---|---|
| CANN version| ≥ 5.0.4| Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the About Ascend Samples Repository. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md). |
| Hardware| Atlas 200 DK/Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
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

2. Obtain the source model required by the application.
    | **Model**| **Description**                     | **How to Obtain**                                            |
    | ------------ | --------------------------------- | ------------------------------------------------------------ |
    | YOLOv3      | An object detection model based on Caffe.| Download the model and weight files by referring to the links in **README.md** in the [ATC_yolov3_caffe_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov3/ATC_yolov3_caffe_AE) directory of the ModelZoo repository.|
    
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/coco_detection_rtsp/model/    
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/aipp_nv12.cfg    
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt
    atc --model=yolov3.prototxt --weight=yolov3.caffemodel --framework=0 --output=yolov3_yuv --soc_version=Ascend310 --insert_op_conf=aipp_nv12.cfg
    ```

3. Obtain the test H.264 file required by the sample.
    ```
    # Run the following commands to go to the **data** folder of the sample and download the corresponding test H.264 file:
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/coco_detection_rtsp/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/face_detection_rtsp/person.h264
    cd ../src
    ```

### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 3](#step_3) directly.**  

1. Modify the configuration file.   
   Add the video file or RTSP address to be detected to the **[videostream]** field. In this example, a maximum of 6-channel video streams are supported.

2. Run the following commands to upload the **coco_detection_rtsp** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
   ```
   # In the following information, *xxx.xxx.xxx.xxx* is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
   scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/coco_detection_rtsp HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx
   cd $HOME/samples/python/level2_simple_inference/2_object_detection/coco_detection_rtsp/src
   ```

3. <a name="step_3"></a>Run the executable file.
   ```
   python3.6 main.py ../data
   ```

### Result Viewing

The terminal displays the detection result of each channel.

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
