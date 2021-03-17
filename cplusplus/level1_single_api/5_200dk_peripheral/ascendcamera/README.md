English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and is not for commercial purposes.**

**This sample applies to Ascend camera 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This document provides only guidance for running the sample in CLI mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## Ascend Camera Sample

Function: use a camera to shoot photos or videos.

Input: Camera

Output: The inference results are displayed on the Presenter Server WebUI, or the data is saved to the local host.

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
**git clone https://gitee.com/ascend/samples.git**
   
   - Compressed package (The download time is short, but the procedure is complex.)
     
     1. Click **Clone or download** in the upper right corner of the samples repository and select **Download ZIP**.
     2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.
     3. In the development environment, run the following commands to decompress the .zip package:   
**cd $HOME**  
**unzip ascend-samples-master.zip**

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
     2. In the development environment, change **presenter\_server\_ip** and **presenter\_view\_ip** in the **script/param.conf** file to the new IP addresses.  
![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - You can also run the **ifconfig** command in the AI1s cloud environment to view the internal IP address.
     > - The IP address for logging in to the AI1s cloud environment is the public IP address of the environment. The IP address obtained by running the **ifconfig** command in the AI1s cloud environment is the internal IP address of the environment.

2. Set the environment variables for compiling the dependencies in the command line of the development environment.
   
   You can run the **uname -a** command in the command line to view the CPU architecture of the development environment and operating environment. If **x86\_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used. Perform the following step based on the actual situation:
   
   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables:
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/x86\_64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**
     
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") Note:
     
     > - If the version is 3.0.0, change **x86\_64-linux** in the **DDK\_PATH** environment variable to **x86\_64-linux\_gcc7.3.0**.
     > - You can run the **uname -a** command in the command line to view the CPU architecture of the development environment and operating environment. If **x86\_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.
   
   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM toolkit needs to be called during app build time because the toolkits of both the x86 and ARM architectures are deployed in the development environment. Therefore, you need to import the path of the ARM ACLlib.
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**  
![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - If the version is 3.0.0, change **arm64-linux** in the **DDK\_PATH** environment variable to **arm64-linux\_gcc7.3.0**.

3. Go to the **ascendcamera** directory and create a directory for storing compilation files. For example, the created directory in this document is **build/intermediates/host**.
   
   **cd $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcamera**
   
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

5. Run the **make** command. The generated executable file **main** is stored in the **ascendcamera/out** directory.
   
   **make**

### Running the Sample (Saving the Image to the Local Host)

**Note: If the development environment and operating environment are deployed on the same server, skip step 1 and go to [step 2](#step_2).**

1. Run the following commands to upload the **ascendcamera** directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:
   
   **scp -r $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcamera HwHiAiUser@***xxx.xxx.xxx.xxx***:/home/HwHiAiUser**
   
   **ssh HwHiAiUser@**_xxx.xxx.xxx.xxx_
   
   ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") Note:
   
   > - *xxx.xxx.xxx.xxx* is the IP address of the operating environment, which is **192.168.1.2** when Atlas 200 DK is connected using the USB, and is the IP address of the corresponding public network for Atlas 300 (AI1s).

2. <a name="step_2"></a>Run the executable file.
   
   - If the development environment and operating environment are deployed on the same server, run the following commands to set the operating environment variables and change the directory:
     
     **export LD\_LIBRARY\_PATH=**
     
     **source ~/.bashrc**
     
     **cd $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcamera/out**
   
   - If the development environment and operating environment are deployed on separate servers, run the following command to change the directory:
     
     **cd $HOME/ascendcamera/out**
   
   Run the following command to run the sample:
   
   **./main -i -c 1 -o ./output/filename.jpg --overwrite**

Parameters

- **-i**: indicates that a JPG image is obtained.

- ****-c****: indicates the channel the camera uses. This parameter can be set to **0** or **1**. The value **0** corresponds to **Camera1** , and the value **1** corresponds to **Camera2**. The default value is **0**.

- **-o**: indicates the file storage location. localDirectory is the name of a local folder. filename.jpg is the name of a saved image, which can be user-defined.

- **--overwrite**: overwrites the existing file with the same name.

### Checking the Result

After the execution is complete, the running results are printed in the CLI of the operating environment and saved in **$HOME/ascendcamera/out/output**.

### Running the Sample(Saving the Video to the Local Host)

**Note: If the development environment and operating environment are deployed on the same server, skip step 1 and go to [step 2](#step2_2).**

1. Run the following commands to upload the **ascendcamera** directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:
   
   **scp -r $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcamera HwHiAiUser@***xxx.xxx.xxx.xxx***:/home/HwHiAiUser**
   
   **ssh HwHiAiUser@*******xxx.xxx.xxx.xxx*********
   
   ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") Note:
   
   > - *xxx.xxx.xxx.xxx* is the IP address of the operating environment, which is **192.168.1.2** when Atlas 200 DK is connected using the USB, and is the IP address of the corresponding public network for Atlas 300 (AI1s).

2. <a name="step2_2"></a>Run the executable file.
   
   - If the development environment and operating environment are deployed on the same server, run the following commands to set the operating environment variables and change the directory:
     
     **export LD\_LIBRARY\_PATH=**
     
     **source ~/.bashrc**
     
     **cd $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcamera/out**
   
   - If the development environment and operating environment are deployed on separate servers, run the following command to change the directory:
     
     **cd $HOME/ascendcamera/out**
   
   Before running the sample, create the **output** folder in the **out** folder.
   
   **cd $HOME/ascendcamera/out**
   
   **mkdir output**
   
   Run the following command to run the sample:
   
   **./main**

### Checking the Result

After the execution is complete, the running results are printed in the CLI of the operating environment and saved in **$HOME/ascendcamera/out/output**.

### Running the Sample (Presenter Server)

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") Note:

> - In the following command, *xxx.xxx.xxx.xxx* indicates the IP address of the operating environment, which is **192.168.1.2** when Atlas 200 DK is connected using the USB, and is the IP address of the corresponding public network for Atlas 300 (AI1s).

1. Run the following command to upload the **ascendcamera** directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**.
   
   **If the development environment and operating environment are deployed on the same server, skip this step.**
   
   **scp -r $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcamera HwHiAiUser@***xxx.xxx.xxx.xxx***:/home/HwHiAiUser**

2. Start the Presenter Server and log in to the operating environment.
   
   - Atlas 200 DK
     
     1. Run the following commands in the development environment to start the Presenter Server:   
**cd $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcamera**  
**bash scripts/run\_presenter\_server.sh**
     2. Run the following command to log in to the operating environment:   
**If the development environment and operating environment are deployed on the same server, skip this step.**   
**ssh HwHiAiUser@*******xxx.xxx.xxx.xxx*********
   
   - Atlas 300 AI accelerator card (AI1s cloud inference environment)
     
     1. Run the following command to log in to the operating environment:   
**If the development environment and operating environment are deployed on the same server, skip this step.**   
**ssh HwHiAiUser@*******xxx.xxx.xxx.xxx*********
     2. Run the following commands in the operating environment to start the Presenter Server:   
**cd $HOME/ascendcamera**  
**bash script/run\_presenter\_server.sh**

3. Run the executable file.
   
   - If the development environment and operating environment are deployed on the same server, run the following commands to set the operating environment variables and change the directory:   
**export LD\_LIBRARY\_PATH=**  
**source ~/.bashrc**  
**cd $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/ascendcameraout**
   
   - If the development environment and operating environment are deployed on separate servers, run the following command to change the directory:   
**cd $HOME/ascendcamera/out**
   
   Run the following command to run the sample. Change *ip* and *xxxx* to the actual IP address and port number.
   
   **./main -v -c 1 -t 60 --fps 20 -w 704 -h 576 -s *ip*:*xxxx*/*presentername***
   
   Parameters
   
   - **-v**: indicates that the video of the camera is obtained and displayed on the Presenter Server.
   - **-c**: indicates the channel to which the camera belongs. The value can be **0** or **1**. The value **0** corresponds to **Camera0**, and the value **1** corresponds to **Camera1**. The default value is **0**.
   - **-t**: indicates that a video file lasting 60 seconds is obtained. If this parameter is not specified, the video file is obtained until the application exits.
   - **--fps**: indicates the frame rate of a saved video. The value range is 1–20. The default video frame rate is 10 fps.
   - **-w**: indicates the width of a saved video.
   - **-h**: indicates the height of a saved video.
   - **-s**: The IP address following **-s** is the IP address of the operating environment. *xxxx* indicates the port number of the Presenter Server corresponding to the Ascend camera application.
   - *presentername*: indicates the view name displayed on Presenter Server. The value is user-defined and must be unique. It is a string of 3 to 20 characters consisting of letters, digits, and underscores (\_).

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