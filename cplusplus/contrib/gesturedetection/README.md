[中文](README_CN.md) | English

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## Sample of Human Gesture Detection
Function: Uses continuous images as the input to detect human gestures in real time. It detects key human feature points based on AI algorithms and obtains the angles of the body joints. Then converts the detection data into recognized gestures and print them on the screen.
So the robot can imitate the gestures of the human body.     
Input: Source JPG image.   
Output: Inference result.    

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)

| Item| Requirement| Remarks|
|---|---|---|
| CANN version| >=5.0.4 | Install the CANN by referring to [Installation](/README.md#installation) in the *About Ascend Samples Repository*. If the CANN version is earlier than the required version, switch to the **samples** repository specific to the CANN version. See [Release Notes](/README.md#release-notes).|
| Hardware| Atlas 200 DK/Atlas 300 (ai1s)  | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| presentagent, opencv, ffmpeg+acllite | For details, see [Third-Party Dependency Installation Guide (C++ Sample)](../../environment/README.md).|

### Installing libeigen3-dev
This dependency is used only in this sample, so you need to install libeigen3-dev before running this sample.
- The development environment architecture is the same as that of the running environment.   
  Run the following command to install libeigen3-dev **in both the development and running environments**:
  ```
  sudo apt-get install libeigen3-dev
  ```
- The development environment is x86, and the running environment is ARM.
  1. Connect the **operating environment** to the network and run the following command to install freetype:
      ```
      sudo apt-get install libeigen3-dev
      ```
  2. In the **development environment**, run the following commands to copy the corresponding .so files:
      ```
      # Copy the related .so files in the ARM architecture to the aarch64-linux-gnu directory of the x86 architecture. This does not cause any problems in the local x86 environment.
      cd /usr/lib/aarch64-linux-gnu
      # Copy related .so files. X.X.X.X indicates the IP address of the operating environment.
      sudo scp -r HwHiAiUser@X.X.X.X:/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/*.so.* ./
      # Copy the related header file.
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/include/eigen3 /usr/include
      ```

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
   | **Model**| **Description**            | **How to Obtain**                                            |
   | ------------ | ------------------------ | ------------------------------------------------------------ |
   | openpose     | Human skeleton point detection model| https://github.com/Ascend/modelzoo/blob/9bf8402aca694dd602be536b5d7ff782c5e8c4e4/contrib/TensorFlow/Research/cv/%20gesturedetection/ATC_OpenPose_caffe_AE |
   | STGCN        | Human gesture detection model      | https://github.com/Ascend/modelzoo/tree/9bf8402aca694dd602be536b5d7ff782c5e8c4e4/contrib/TensorFlow/Research/cv/%20gesturedetection/ATC_STGCN_tf_AE |

   ```  
   #To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.    
   
   cd $HOME/samples/cplusplus/contrib/gesturedetection/model    
   wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_iter_440000.caffemodel 
   wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_deploy.prototxt
   wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/insert_op.cfg
   wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/stgcn_fps30_sta_ho_ki4.pb
   atc --input_shape="input_features:1,2,30,14" --input_format=NCHW --output="./stgcn_fps30_sta_ho_ki4" --soc_version=Ascend310 --framework=3 --model=./stgcn_fps30_sta_ho_ki4.pb
   atc --input_shape="data:1,3,128,128" --weight="./pose_iter_440000.caffemodel" --input_format=NCHW --output="./pose_deploy" --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --framework=0 --model=./pose_deploy.prototxt 
   ```

### Sample Deployment
Run the following commands to execute the compilation script to start sample compilation:     
   ```
   cd  $HOME/samples/cplusplus/contrib/gesturedetection/scripts    
   bash sample_build.sh
   ```

### Sample Running
1. Run the following commands to upload the **gesturedetection** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):   
   ```
   # In the following information, <xxx.xxx.xxx.xxx> is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
   scp -r $HOME/samples/cplusplus/contrib/gesturedetection/ HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser    
   ssh HwHiAiUser@xxx.xxx.xxx.xxx     
   cd $HOME/samples/cplusplus/contrib/gesturedetection/scripts
   ```

2. <a name="step_2"></a>Execute the script to run the sample.        
   ```
   bash sample_run.sh
   ```

### Result Viewing
After the running is complete, the inference result is displayed in the CLI of the operating environment.

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
