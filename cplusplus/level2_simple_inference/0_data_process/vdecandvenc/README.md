English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and is not for commercial purposes.**

**This sample applies to Ascend camera 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This document provides only guidance for running the sample on the command line. For details about how to run the sample in MindStudio, see the [Wiki of Running Video Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## vdecandvenc Sample

Function: Encode a video by calling the **venc** and **vdec** APIs of the DVPP.

Input: original MP4 file, which is an H.264 or H.265 file.

Output: encoded H.264 or H.265 file

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been prepared based on [Preparing Environment and Installing Dependencies](../../../environment).

- The development environment and operating environment of the corresponding product have been installed.

### Preparing Software

1. Obtain the source code package.
   
   You can download the source code in either of the following ways:
   
   - Command line (The download takes a long time, but the procedure is simple.)
     
     In the development environment, run the following commands as a non-root user to download the source code repository:
     
     **cd $HOME**
     
     **git clone https://gitee.com/ascend/samples.git**
   
   - Compressed package (The download time is short, but the procedure is complex.)
     
     1. Click **Clone or download** in the upper right corner of the samples repository and select **Download ZIP**.
     
     2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.
     
     3. In the development environment, run the following commands to decompress the **.zip** package:
        
        **cd $HOME**
        
        **unzip ascend-samples-master.zip**

2. Obtain the original model required by the application.
   
   Obtain the original model and its weight files used in the application by referring to the following table and save them to any directory of a common user in the development environment, for example, **$HOME/models/vdecandvenc**.
   
   | **Model Name**| **Description**| **How to Obtain**|
   |----------|----------|----------|
   | face\_detection| Applies to image classification. It is a YOLOv3 model based on Caffe.| Download the model and weight file by referring to the section about downloading the original model in the **README.md** file in [https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC\_resnet10-SSD\_caffe\_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/facedetection/ATC_resnet10-SSD_caffe_AE).|

   ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
   
   > - The converted OM model is provided in the ModelZoo. However, the model does not match the current sample. Therefore, you need to download the original model and weight file and convert the model again.

3. Convert the original model to a Da Vinci model.
   
   **Note: Ensure that the environment variables have been configured based on [Preparing Environment and Installing Dependencies](../../../environment).**
   
   1. Set the **LD\_LIBRARY\_PATH** environment variable.
      
      The **LD\_LIBRARY\_PATH** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.
      
      **export LD\_LIBRARY\_PATH=\\${install\_path}/atc/lib64**
   
   2. Run the following commands to download the AIPP configuration file and convert the model:
      
      **cd $HOME/models/face\_detection\_camera**
      
      **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/face\_detection\_camera/insert\_op.cfg**
      
      **atc --output\_type=FP32 --input\_shape="data:1,3,300,300" --weight=./face\_detection\_fp32.caffemodel --input\_format=NCHW --output=./face\_detection --soc\_version=Ascend310 --insert\_op\_conf=./insert\_op.cfg --framework=0 --save\_original\_model=false --model=./face\_detection.prototxt**
   
   3. Run the following command to copy the converted model to the **model** folder of the sample:
      
      **cp ./face\_detection.om $HOME/samples/level1\_single\_api/1\_acl/4\_dvpp/vdecandvenc/model/**

4. Obtain the test video required by the sample.
   
   Run the following commands to go to the **data** folder of the sample and download the corresponding test video:
   
   **cd /home/ascend/samples/cplusplus/level1\_single\_api/1\_acl/4\_dvpp/vdecandvenc/data**
   
   **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdecandvenc/out\_video.h264**  
**wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/vdecandvenc/person.mp4**

### Deploying the Sample

1. Set the environment variables for compiling the dependencies on the command line of the development environment.
   
   Perform the following step based on the actual situation:
   
   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables:
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/x86\_64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**
     
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - If the version is 3.0.0, change **x86\_64-linux** in the **DDK\_PATH** environment variable to **x86\_64-linux\_gcc7.3.0**.
     > - You can run the **uname -a** command on the command line to view the CPU architecture of the development environment and operating environment. If **x86\_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.
   
   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM toolkit needs to be called during application build time because the toolkits of both the x86 and ARM architectures are installed in the development environment. Therefore, you need to import the path of the ARM ACLlib.
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**
     
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - If the version is 3.0.0, change **arm64-linux** in the **DDK\_PATH** environment variable to **arm64-linux\_gcc7.3.0**.
     > - You can run the **uname -a** command on the command line to view the CPU architecture of the development environment and operating environment. If **x86\_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.

2. Go to the **vdecandvenc** directory and create a directory for storing compilation files. For example, the created directory in this document is **build/intermediates/host**.
   
   **cd $HOME/samples/cplusplus/level1\_single\_api/1\_acl/4\_dvpp/vdecandvenc**
   
   **mkdir -p build/intermediates/host**

3. Go to the **build/intermediates/host** directory and run the **cmake** command.
   
   - If the development environment and operating environment have the same OS architecture, run the following commands to perform compilation.
     
     **cd build/intermediates/host**
     
     **make clean**
     
     **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**
   
   - When the OS architecture of the development environment is different from that of the operating environment, you need to use the cross compiler for compilation. For example, if the development environment uses the x86 architecture and the operating environment uses the ARM architecture, run the following commands to perform cross compilation:
     
     **cd build/intermediates/host**
     
     **make clean**
     
     **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

4. Run the **make** command. The generated executable file **main** is stored in the **venc/out** directory.
   
   **make**

### Running the Sample

**Note: If the development environment and operating environment are deployed on the same server, skip step 1 and go to [step 2](#step_2).**

1. Run the following commands to upload the **vdecandvenc** directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:
   
   **scp -r $HOME/samples/cplusplus/level1\_single\_api/1\_acl/4\_dvpp/vdecandvenc HwHiAiUser@*xxx.xxx.xxx.xxx*:/home/HwHiAiUser**
   
   **ssh HwHiAiUser@*xxx.xxx.xxx.xxx***
   
   ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
   
   > - *xxx.xxx.xxx.xxx* is the IP address of the operating environment, which is **192.168.1.2** when the Atlas 200 DK is connected using the USB, and is the IP address of the corresponding public network for Atlas 300 (AI1s).

2. <a name="step_2"></a>Run the executable file.
   
   - If the development environment and operating environment are deployed on the same server, run the following commands to set the operating environment variables and change the directory:
     
     **export LD\_LIBRARY\_PATH=**
     
     **source ~/.bashrc**
     
     **cd $HOME/samples/cplusplus/level1\_single\_api/1\_acl/4\_dvpp/vdecandvenc/out**
   
   - If the development environment and operating environment are deployed on separate servers, run the following command to change the directory:
     
     **cd $HOME/vdecandvenc/out**
   
   Run the following command to run the sample:
   
   **mkdir output**
   
   **./main ../data/out\_video.h264**

### Checking the Result

After the execution is complete, the running result is printed on the command line of the operating environment, and the video after inference is saved in the **$HOME/vdecandvenc/out/output** directory.