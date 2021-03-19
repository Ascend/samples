English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This readme file provides only guidance for running the sample in command line (CLI) mode.**

## 3D Action Sorting Sample

Function: classifies action by using the 3DCNN model.

Input: video data to be inferred

Output: 10 action confidence after inference


### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](https://gitee.com/ascend/samples/tree/master/python/environment).

- The development environment and operating environment of the corresponding product have been installed.

### Software Preparation

1. Obtain the source code package.

   You can download the source code in either of the following ways:

    - Command line (The download takes a long time, but the procedure is simple.)

        In the development environment, run the following commands as a non-root user to download the source code repository:

       **cd $HOME**

       **git clone https://gitee.com/ascend/samples.git**

    - Compressed package (The download takes a short time, but the procedure is complex.)

        1. Click **Clone or download** in the upper right corner of the samples repository and click **Download ZIP**.

        2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.

        3. In the development environment, run the following commands to unzip the package:

            **cd $HOME**

            **unzip ascend-samples-master.zip**

2. Obtain the source model required by the application.

    Obtain the original model and its weight files used in the application by referring to the following table and save them to any directory of a common user in the development environment, for example, **$HOME/models/3Dgesture_recognition**.

    | **Model Name** | **Description**                          | **How to Obtain**                        |
    | -------------- | ---------------------------------------- | ---------------------------------------- |
    | 3DCNN    | 3D action inference model. It is a 3DCNN model based on tensorflow. | Download the model and weight files by referring to the **README.md** file in [https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/3dcnn/ATC_3DCNN_tensorflow_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/3dcnn/ATC_3DCNN_tensorflow_AE). |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  

    > - The converted OM model provided by ModelZoo does not match the current sample. Therefore, you need to download the original model and weight files, and convert the model by yourself.

3. Convert the original model to a Da Vinci model.

    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](https://gitee.com/ascend/samples/tree/dev/python/environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following commands to download the AIPP configuration file and convert the model:

        **cd $HOME/models/3Dgesture_recognition**  

        **atc --model=3d_gesture_recognition.pb  --framework=3 --output=3d_gesture_recognition --soc_version=Ascend310 --input_shape="X:1,16,112,112,3" --input_format=NDHWC**

    3. Run the following command to copy the converted model to the **model** folder of the sample:

     **cp ./3d_gesture_recognition.om $HOME/samples/python/contrib/3Dgesture_recognition/model/**

4. Obtain the test images required by the sample.

    Run the following commands to go to the **data** folder of the sample and download the corresponding test images:

    
    **cd $HOME/samples/python/contrib/3Dgesture_recognition/data**

    **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/3D_gesture_recognition/testdata/test_float32_actiontype7.bin**    



### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **3Dgesture_recognition** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/samples/python/contrib/3Dgesture_recognition HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  

    > - In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.

2. Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variables and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/python/contrib/3Dgesture_recognition/**     

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:

      **cd $HOME/3Dgesture_recognition/**      

    Run the following command to run the sample:

    **python3.6 src/3Dgesture_recognition.py ./data/**
### Result Checking

After the execution is complete, find the JPG images with inference results in the **outputs** directory.
