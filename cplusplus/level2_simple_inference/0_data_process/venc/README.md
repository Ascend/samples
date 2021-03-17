English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and is not for commercial purposes.**

**This sample applies to CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This readme file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## VENC Sample

Function: encodes a video.

Input: original MP4 file

Output: encoded H.264 file


### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Preparing Environment and Installing Dependencies](../../../environment).

- The development environment and operating environment of the corresponding product have been installed.

### Preparing Software

1. Obtain the source code package.

   You can use either of the following methods to download the source code:

    - Command line (The download takes a long time, but the procedure is simple.)

        In the development environment, run the following commands as a non-root user to download the source code repository:

       **cd $HOME**

       **git clone https://gitee.com/ascend/samples.git**

    - Compressed package (The download takes a short time, but the procedure is complex.)

        1. Click **Clone or download** in the upper right corner of the samples repository and click **Download ZIP**.

        2. Upload the ZIP package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.

        3. In the development environment, run the following commands to unzip the ZIP package:

            **cd $HOME**

            **unzip ascend-samples-master.zip**

2. Obtain the required test image.

    Run the following commands to go to the **data** folder of the sample and download the corresponding test image:

    **cd /home/ascend/samples/cplusplus/level1_single_api/1_acl/4_dvpp/venc/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/venc/detection.mp4**


### Deploying the Sample

1. Set the environment variables for building the dependencies on the CLI of the development environment.

   Perform the following step based on the actual situation:

   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables:

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - If the CANN version is 3.0.0, change **x86_64-linux** in the ***DDK_PATH*** environment variable to **x86_64-linux_gcc7.3.0**.
        > - You can run the **uname -a** command on the CLI to view the CPU architecture of the development environment and operating environment. If **x86_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.

   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM Toolkit needs to be called during application build time because the Toolkits of both the x86 and ARM architectures are installed in the development environment. Therefore, you need to import the path of the ARM ACLlib.

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - If the CANN version is 3.0.0, change **arm64-linux** in the ***DDK_PATH*** environment variable to **arm64-linux_gcc7.3.0**.
        > - You can run the **uname -a** command on the CLI to view the CPU architecture of the development environment and operating environment. If **x86_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.

2. Switch to the **venc** directory and create a directory for storing build outputs, for example, **build/intermediates/host** in this sample.

    **cd $HOME/samples/cplusplus/level1_single_api/1_acl/4_dvpp/venc**

    **mkdir -p build/intermediates/host**

3. Go to the **build/intermediates/host** directory and run the **cmake** command.

    - If the OS architecture of the development environment is the same as that of the operating environment, run the following commands to perform compilation:

      **cd build/intermediates/host**   

      **make clean**

      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

    - If the OS architecture of the development environment is different from that of the operating environment, you need to use the cross compiler for compilation. For example, if the development environment uses the x86 architecture and the operating environment uses the ARM architecture, run the following commands to perform cross compilation:

      **cd build/intermediates/host**

      **make clean**

      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

4. Run the **make** command and find the generated executable file **main** in the **venc/out** directory.

    **make**

### Running the Sample

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **venc** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/samples/cplusplus/level1_single_api/1_acl/4_dvpp/venc HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.

2. <a name="step_2"></a>Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variables and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/cplusplus/level1_single_api/1_acl/4_dvpp/venc/out**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:

      **cd $HOME/venc/out**

    Run the following command to run the sample:

    **mkdir output**

    **./main ../data/detection.mp4**

### Viewing the Result

After the execution is complete, the running result is printed on the CLI of the operating environment, and the video after inference is saved in the **$HOME/venc/out/output** directory.
