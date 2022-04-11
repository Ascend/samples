English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample works with CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300.**

**This readme file provides only guidance for running the sample in the command line. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Running%20Image%20Samples%20in%20MindStudio?sort_id=3736297).**

## Vessel Segmentation Sample

Function: Vessel Segmentation in an input image by using the vessel model.

Input: a source JPG image

Output: the result JPG image

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](../../environment).

- The development environment and operating environment of the corresponding product have been set up.

### Software Preparation

1. Obtain the source code package.

   You can download the source code in either of the following ways:

    - Command line (The download takes a long time, but the procedure is simple.)

        In the development environment, run the following commands as a non-root user to download the source repository:

        **cd $HOME**

        **git clone https://github.com/Ascend/samples.git**

    - Compressed package (The download takes a short time, but the procedure is complex.)

        1. Click **Clone or download** in the upper right corner of the samples repository and click **Download ZIP**.

        2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.

        3. In the development environment, run the following commands to unzip the package:

            **cd $HOME**

            **unzip ascend-samples-master.zip**

2. Obtain the original model required by the application.

    Obtain the original model file and the corresponding weight file used by the application by referring to the following table and save them to any directory of a common user in the development environment, for example, **$HOME/models/gesture_recognition_picture**.


| **Model Name**      | **Description**                          | **How to Obtain**                        |
| ------------------- | ---------------------------------------- | ---------------------------------------- |
| vessel | Image segmentation model, vessel segmentation model based on Caffe | Download the model and weight files by referring to the **[README.md](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/vessel-segmentation/ATC_retina-unet_caffe_AE)** file. |

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
> - The converted OM model provided by ModelZoo does not match the current sample. Therefore, you need to download the original model and weight files, and convert the model by yourself.

3. Convert the original model to a DaVinci model.

    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](../../../environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when Ascend Tensor Compiler (ATC) is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Convert the model:

         **atc --model=./deploy_vel_ascend.prototxt --weight=./vel_hw_iter_5000.caffemodel --framework=0 --output=./vessel --soc_version=Ascend310 --input_format=NCHW --input_fp16_nodes=data -output_type=FP32 **

    3. Run the following command to copy the converted model to the **model** folder of the sample:

        **cp ./vessel.om $HOME/samples/python/contrib/vessel_segmentation/model/**

4. Obtain the test images required by the sample.

    Run the following commands to go to the **data** folder of the sample and download the corresponding test images:

    **cd $HOME/samples/python/contrib/vessel_segmentation/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vessel_segmentation/test_image/test1.png**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vessel_segmentation/test_image/test2.png**       

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vessel_segmentation/test_image/test3.png** 


### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **vessel_segmentation** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):

    **scp -r $HOME/samples/python/contrib/vessel_segmentation HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  

    > - Replace ***xxx.xxx.xxx.xxx*** with the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 is the corresponding public network IP address.

2. <a name="step_2"></a>Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variable and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/python/contrib/vessel_segmentation/src**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:

      **cd $HOME/vessel_segmentation/src**      

    Run the following command to run the sample:

    **python3.6 main.py ../data/**
### Result Checking

After the execution is complete, find the JPG images with inference results in the **outputs** directory.
