English|[中文](README_CN.md)

**The following sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**The sample applies to CANN 20.0 and later versions and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This README provides only guidance for running samples in command line mode. For details about how to run samples in MindStudio, see [Running Image Samples in MindStudio_wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## faste_RCNN_VOC_detection_dynamic_resolution Sample

Function: Use the faster_rcnn model to perform prediction and inference on the input image and display the result on the output image.

Input: JPG image

Output: JPG image with the inference result

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been prepared according to [Environment Preparation and Dependency Installation](../../../environment).

- The development environment and operating environment of the corresponding product have been installed.

### Software Preparation

1. Obtain the source code package.

   You can use either of the following methods to download the source code:

    - Command line (The download takes a long time, but the procedure is simple.)

        In the development environment, run the following command as a non-root user to download the source code repository:

       **cd $HOME**

       **git clone https://gitee.com/ascend/samples.git**

    - Compressed package (The download time is short, but the procedure is complex.)

        1. Click **Clone or download** in the upper right corner of the samples repository and click **Download ZIP**.

        2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.

        3. In the development environment, run the following commands to decompress the .zip package:

            **cd $HOME**

            **unzip ascend-samples-master.zip**

2. Obtain the source network model required by the application.

    Obtain the original network model and its weight file used in the application by referring to the following table and store them in any directory of a common user in the development environment, for example, **$HOME/models/faste_RCNN_VOC_detection_dynamic_resolution**.

    | **Model Name** | **Description** | **How to Obtain** |
    |---|---|---|
    |  faster_rcnn| Image classification inference model. It is a YOLOv3 model based on Caffe.   | Download the original model and weight file by referring to [https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/faster_rcnn/ATC_faster_rcnn_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/faster_rcnn/ATC_faster_rcnn_caffe_AE).  |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

    > - The converted OM model provided by ModelZoo does not match the current sample. Therefore, you need to download the original model and weight file to convert the model by yourself.

3. Convert the original model to a Da Vinci model.

    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](../../../environment).**

    1. Set the **LD_LIBRARY_PATH** environment variable.

        The **LD_LIBRARY_PATH** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable in the command line to facilitate modification.

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following command to download the AIPP configuration file and run the ATC command to convert the model:

        **cd $HOME/models/faste_RCNN_VOC_detection_dynamic_resolution**  

        **atc --model=faster_rcnn.prototxt --weight=faster_rcnn.caffemodel --framework=0 --output=faster_rcnn --soc_version=Ascend310 --input_shape="data:1,3,-1,-1;im_info:1,3" --dynamic_image_size="512,512;600,600;800,800"**

    3. Run the following command to copy the converted model to the **model** folder of the sample.

        **cp ./faster_rcnn.om $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution/model/**

4. Obtain the test images required by the sample.

    Run the following command to go to the **data** folder of the sample and download the corresponding test image:

    **cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/faste_RCNN_VOC_detection_dynamic_resolution/bus.jpg**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/faste_RCNN_VOC_detection_dynamic_resolution/boat.jpg**

### Sample Deployment

1. Set the environment variables on which compilation depends in the command line of the development environment.

   Perform the following step based on the actual situation:

   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables:

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

        > - If the version is 20.0, change **x86_64-linux** in the **DDK_PATH** environment variable to **x86_64-linux_gcc7.3.0**.
        > - You can run the **uname -a** command in the command line to view the CPU architecture of the development environment and operating environment. If x86_64 is displayed in the command output, the architecture is x86. If arm64 is displayed in the command output, the architecture is ARM.

   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM Toolkit needs to be called during app build time because the Toolkits of both the x86 and ARM architectures are deployed in the development environment. Therefore, you need to import the path of the ARM ACLlib.

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

        > - If the version is 20.0, change **arm64-linux** in the **DDK_PATH** environment variable to **arm64-linux_gcc7.3.0**.
        > - You can run the **uname -a** command in the command line to view the CPU architecture of the development environment and operating environment. If x86_64 is displayed in the command output, the architecture is x86. If arm64 is displayed in the command output, the architecture is ARM.

2. Go to the **faste_RCNN_VOC_detection_dynamic_resolution** directory and create a directory for storing build outputs. For example, the directory created in this sample is **build/intermediates/host**.

    **cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution**

    **mkdir -p build/intermediates/host**

3. Go to the **build/intermediates/host** directory and run the cmake command.

    - If the OS architecture of the development environment is the same as that of the operating environment, run the following command to perform compilation.

      **cd build/intermediates/host**

      **make clean**

      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

    - If the OS architecture of the development environment is different from that of the operating environment, use the cross compiler to perform compilation. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, run the following command to perform cross compilation:

      **cd build/intermediates/host**

      **make clean**

      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

4. Run the make command to generate an executable file **main** in the **faste_RCNN_VOC_detection_dynamic_resolution/out** directory.

    **make**

### Sample Running

**Note: If the development environment and operating environment are deployed on the same server, skip step 1 and go to [step 2](#step_2).**

1. Run the following command to upload the **faste_RCNN_VOC_detection_dynamic_resolution** directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

> - **xxx.xxx.xxx.xxx** indicates the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.

2. <a name="step_2"></a>Run the executable file.

    - If the development environment and operating environment are deployed on the same server, run the following commands to set the operating environment variables and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/faste_RCNN_VOC_detection_dynamic_resolution/out**

    - If the development environment and operating environment are deployed on separate servers, run the following command to switch the directory:

      **cd $HOME/faste_RCNN_VOC_detection_dynamic_resolution/out**

    - Create a result file.

      **mkdir output**

    Switch to the directory and run the following command to run the sample:

    **./main**

### Checking the Result

After the running is complete, the inference result is displayed in the command line of the operating environment.
