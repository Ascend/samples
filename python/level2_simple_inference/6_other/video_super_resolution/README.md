English|[中文](README_CN.md)


**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in the command line. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Running%20Image%20Samples%20in%20MindStudio?sort_id=3736297).**

## EDVR Video Super-Resolution Sample
Function: Use the Video Restoration with Enhanced Deformable Convolutional Networks (EDVR) model to perform super-resolution inference on the input video.  
Sample input: low-resolution video to be inferred   
Sample output: super-resolution video after inference  

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)  
| Item      | Requirements                                                        | Remarks                                                        |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CANN version  | ≥ 5.0.4                                                     | Install the CANN by referring to [Installation](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the **About Ascend Samples Repository**. If the CANN version is earlier than the required version, switch to the sample repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md). |
| Hardware  | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| opencv_python, ffmpeg_python, numpy, imageio                 | Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://gitee.com/ascend/samples/tree/master/python/environment).|

**Note: In addition to setting up the basic environment, you must run the following commands to install additional third-party dependencies:** 

```
apt-get install ffmpeg
pip install imageio
pip install opencv_python
pip install ffmpeg_python
```

### Sample Preparation

1. Obtain the source package.

   You can download the source code in either of the following ways:  
    - - Command line (The download takes a long time, but the procedure is simple.)
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
      ```
       **To switch to another tag (for example, v0.5.0), run the following command:**
       ```
       git checkout v0.5.0
       ```
    - - Compressed package (The download takes a short time, but the procedure is complex.)  
       **Note: If you want to download the code of another version, switch the branch of the samples repository according to the prerequisites.**   
       ``` 
        # 1. Click Clone or Download in the upper right corner of the samples repository and click Download ZIP.   
        # 2. Upload the .zip package to the home directory of a common user in the development environment, for example, $HOME/ascend-samples-master.zip.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
       ```

2. Obtain the source model required by the application.
    |  **Model Name** |  **Model Description** |  **Download Link** |
    |---|---|---|
    |  EDVR | Super-resolution inference model for videos. It is an EDVR model based on TensorFlow. |  [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/EDVR_180_320.pb](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/EDVR_180_320.pb)|
    
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands.
    cd ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution/model     
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/EDVR_180_320.pb        
    atc --input_shape="L_input:1,5,180,320,3" --input_format=ND --output="EDVR_180_320" --soc_version=Ascend310 --framework=3 --model="./EDVR_180_320.pb" --output_type=FP32
    ```
```
    
3. Obtain the test videos required by the sample.
```
    mkdir -p ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution/data
    cd ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/video_super_resolution/low_resolution.mp4  
    ```

### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **video_super_resolution** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
    ```
    # In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r ${HOME}/samples/python/level2_simple_inference/6_other/video_super_resolution  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/video_super_resolution/src
    ```

2. Run the project.
    ```
    python3.6 video_super_resolution.py
    ```
### Result Viewing

After the execution is complete, a super-resolution video is generated in the **outputs** directory.

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue in the **samples** repository.
