English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and is not for commercial purposes.**

**This sample applies to Ascend camera 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This document provides only guidance for running the sample in CLI mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## Video Object Detection Sample

Function: detect objects that appear in the video and provide prediction results in the video.

Input: original MP4 video

Output: prediction results displayed on the Presenter page

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been prepared based on [Preparing Environment and Installing Dependencies](../../../environment).

- The development environment and operating environment of the corresponding product have been installed.

### Preparing Software

1. Obtain the source code package.
   
   You can use either of the following methods to download the source code:
   
   - Command line (The download takes a long time, but the procedure is simple.)
In the development environment, run the following commands as a non-root user to download the source code repository:   
**cd $HOME**  
**git clone https://github.com/Ascend/samples.git**
   
   - Compressed package (The download time is short, but the procedure is complex.)
     
     1. Click **Clone or download** in the upper right corner of the samples repository and select **Download ZIP**.
     2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.
     3. In the development environment, run the following commands to decompress the .zip package:   
**cd $HOME**  
**unzip ascend-samples-master.zip**

2. Obtain the source model required by the application.
   
   Obtain the original network model and its weight file used in the application by referring to the following table and store them in any directory of a common user in the development environment, for example, **$HOME/models/YOLOV3\_coco\_detection\_video**.
   
   | **Model Name**| **Description**| **How to Obtain**|
   |----------|----------|----------|
   | yolov3| Applies to image classification. It is a YOLOv3 model based on Caffe.| Download the model and weight file by referring to the section about downloading the original model in the **README.md** file in [https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3/ATC\_yolov3\_caffe\_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3/ATC_yolov3_caffe_AE).|

   ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
   
   > - The converted OM model is provided in the ModelZoo. However, the model does not match the current sample. Therefore, you need to download the original model and weight file and convert the model again.

3. Convert the original model to a Da Vinci model.
   
   **Note: Ensure that the environment variables have been configured based on [Preparing Environment and Installing Dependencies](../../../environment).**
   
   1. Set the **LD\_LIBRARY\_PATH** environment variable.
      
      The **LD\_LIBRARY\_PATH** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to separately set this environment variable in the CLI to facilitate modification.        
      **export install\_path=$HOME/Ascend/ascend-toolkit/latest**     
      
      **export LD\_LIBRARY\_PATH=\\${install\_path}/atc/lib64**
   
   2. Run the following commands to download the AIPP configuration file and convert the model:
      
      **cd $HOME/models/YOLOV3\_coco\_detection\_video**
      
      **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_video/aipp_bgr.cfg**
      
      **atc --model=./yolov3.prototxt --weight=./yolov3.caffemodel --framework=0 --output=yolov3 --soc\_version=Ascend310 --insert\_op\_conf=./aipp\_bgr.cfg**
   
   3. Run the following command to copy the converted model to the **model** folder of the sample:
      
      **cp ./yolov3.om $HOME/samples/cplusplus/level2\_simple\_inference/2\_object\_detection/YOLOV3\_coco\_detection\_video/model/**

4. Obtain the test file required by the sample.
   
   Run the following commands to go to the **data** folder of the sample, download the test file, and return to the sample folder:
   
   **cd $HOME/samples/cplusplus/level2\_simple\_inference/2\_object\_detection/YOLOV3\_coco\_detection\_video/data**
   
   **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_coco_detection_video/detection.mp4**
   
   **cd ..**

### Deploying the Sample

1. Modify the Presenter-related configuration file.
   
   Change **presenter\_server\_ip** and **presenter\_view\_ip** in the **scripts/param.conf** file in the sample directory to the IP addresses that can be pinged in the development environment. The following two cases are used as examples:
   
   - Atlas 200 DK
     
     1. In the development environment, run the **ifconfig** command to view available IP addresses.
     2. In the development environment, change **presenter\_server\_ip** and **presenter\_view\_ip** in the **scripts/param.conf** file to the new IP addresses.  
![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - 1\. If the development environment and operating environment are deployed on separate servers, the configured virtual NIC IP address is used, for example, **192.168.1.223**.
     > - 2\. If the development environment and operating environment are deployed on the same server, the fixed IP address of Atlas 200 DK is used, for example, **192.168.1.2**.
   
   - Atlas 300 AI accelerator card (AI1s cloud inference environment)
     
     1. On the ECS console, check the available intranet IP address in the cloud environment of AI1s, for example, **192.168.0.198**.
     2. In the development environment, change **presenter\_server\_ip** and **presenter\_view\_ip** in the **scripts/param.conf** file to the new IP addresses.  
![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - You can also run the **ifconfig** command in the AI1s cloud environment to view the internal IP address.
     > - The IP address for logging in to the AI1s cloud environment is the public IP address of the environment. The IP address obtained by running the **ifconfig** command in the AI1s cloud environment is the internal IP address of the environment.

2. Set the environment variables for compiling the dependencies in the command line of the development environment.
   
   You can run the **uname -a** command in the command line to view the CPU architecture of the development environment and operating environment. If **x86\_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used. Perform the following step based on the actual situation:
   
   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables:
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/x86\_64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**
     
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - If the version is 3.0.0, change **x86\_64-linux** in the **DDK\_PATH** environment variable to **x86\_64-linux\_gcc7.3.0**.
   
   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM toolkit needs to be called during app build time because the toolkits of both the x86 and ARM architectures are deployed in the development environment. Therefore, you need to import the path of the ARM ACLlib.
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif")**Note**:
     
     > - If the version is 3.0.0, change **arm64-linux** in the **DDK\_PATH** environment variable to **arm64-linux\_gcc7.3.0**.

3. Go to the **YOLOV3\_coco\_detection\_video** directory and create a directory for storing compilation files. For example, the created directory in this document is **build/intermediates/host**.
   
   **cd $HOME/samples/cplusplus/level2\_simple\_inference/2\_object\_detection/YOLOV3\_coco\_detection\_video**
   
   **mkdir -p build/intermediates/host**

4. Go to the **build/intermediates/host** directory and run the **cmake** command to generate a compilation file.
   
   - If the development environment and operating environment have the same OS architecture, run the following commands to perform compilation.   
**cd build/intermediates/host**  
**make clean**  
**cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**
   
   - When the OS architecture of the development environment is different from that of the operating environment, you need to use the cross compiler for compilation. For example, if the development environment uses the x86 architecture and the operating environment uses the ARM architecture, run the following commands to perform cross compilation:   
**cd build/intermediates/host**  
**make clean**  
**cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

5. Run the **make** command. The generated executable file **main** is stored in the **YOLOV3\_coco\_detection\_video/out** directory.
   
   **make**

### Running the Sample

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**

> - In the following command, *xxx.xxx.xxx.xxx* indicates the IP address of the operating environment, which is **192.168.1.2** when Atlas 200 DK is connected using the USB, and is the IP address of the corresponding public network for Atlas 300 (AI1s).

1. Run the following command to upload the **YOLOV3\_coco\_detection\_video** directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**.
   
   **If the development environment and operating environment are deployed on the same server, skip this step.**
   
   **scp -r $HOME/samples/cplusplus/level2\_simple\_inference/2\_object\_detection/YOLOV3\_coco\_detection\_video HwHiAiUser@*xxx.xxx.xxx.xxx*:/home/HwHiAiUser**

2. Start the Presenter Server and log in to the operating environment.
   
   - Atlas 200 DK
     
     1. Run the following commands in the development environment to start the Presenter Server:   
  **cd $HOME/samples/common/**
  **bash run_presenter_server.sh $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video/scripts/param.conf**
     2. Run the following command to log in to the operating environment:   
**If the development environment and operating environment are deployed on the same server, skip this step.**   
**ssh HwHiAiUser@*xxx.xxx.xxx.xxx***
   
   - Atlas 300 AI accelerator card (AI1s cloud inference environment)
     
     1. Run the following command to log in to the operating environment:   
**If the development environment and operating environment are deployed on the same server, skip this step.**   
**ssh HwHiAiUser@*xxx.xxx.xxx.xxx***
     2. Start the Presenter Server in the operating environment.   
Go to the directory where the project is located (for example, **$HOME/samples/common/**)     
and run the **bash run_presenter_server.sh $HOME/samples/cplusplus/level2_simple_inference/2_object_detection/YOLOV3_coco_detection_video/scripts/param.conf** command.

3. Run the executable file.
   
   - If the development environment and operating environment are deployed on the same server, run the following commands to set the operating environment variables and change the directory:   
**export LD\_LIBRARY\_PATH=**  
**source ~/.bashrc**  
**cd $HOME/samples/cplusplus/level2\_simple\_inference/2\_object\_detection/YOLOV3\_coco\_detection\_video/out**
   
   - If the development environment and operating environment are deployed on separate servers, run the following command to change the directory:
     
     **cd $HOME/YOLOV3\_coco\_detection\_video/out**
   
   - Create the result folder.
     
     **mkdir output**
   
   Run the following command to run the sample:
   
   **./main ../data/detection.mp4**

### Checking the Result

1. Open the Presenter Server WebUI.
   
   - Atlas 200 DK
     
     Open the URL that is displayed when the Presenter Server is started.
   
   - Atlas 300 AI accelerator card (AI1s cloud inference environment)
     
     **For example, the intranet IP address of the Atlas 300 AI accelerator card (AI1s) is 192.168.0.194 and the public IP address is 124.70.8.192.**
     
     **Please visit http://192.168.0.194:7009 for display server** is displayed when the Presenter Server is started.
     
     You only need to replace the internal IP address **192.168.0.194** in the URL with the public IP address **124.70.8.192**. Then the URL is **http://124.70.8.192:7009**.
     
     Open the URL in the browser.

2. Wait for Presenter Agent to transmit data to the server and click **Refresh**. When there is data, the icon in the **Status** column for the corresponding channel turns green.

3. Click a link in the **View Name** column to view the result.