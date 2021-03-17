English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend software stack and is not for commercial purposes.**

**This sample applies to Ascend camera 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This document provides only guidance for running the sample in CLI mode. For details about how to run the sample in MindStudio, see the [Wiki of Running Image Sample in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## GPIO Sample

Function: Configure the GPIO pins.

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been prepared based on [Preparing Environment and Installing Dependencies](https://gitee.com/ascend/samples/tree/dev/cplusplus/environment).

- The development environment and operating environment of the corresponding product have been installed.

### Preparing Software

1. Obtain the source code package.
   
   You can use either of the following methods to download the source code:
   
   - Use the CLI. This method takes a long time, but the procedure is simple.
     
     In the development environment, run the following commands as a non-root user to download the source code repository:
     
     **cd $HOME**
     
     **git clone https://gitee.com/ascend/samples.git**
   
   - Download the source code as a compressed package. This method takes a short time, but the procedure is complex.
     
     1. Click **Clone or download** in the upper right corner of the samples repository and select **Download ZIP**.
     
     2. Upload the ZIP package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.
     
     3. In the development environment, run the following commands to decompress the **.zip** package:
        
        **cd $HOME**
        
        **unzip ascend-samples-master.zip**

### Deploying the Sample

1. Set the environment variables for compiling the dependencies on the CLI of the development environment.
   
   Perform the following step based on the actual situation:
   
   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables:
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/x86\_64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**
     
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - If the version is 3.0.0, change **x86\_64-linux** in the **DDK\_PATH** environment variable to **x86\_64-linux\_gcc7.3.0**.
     > - You can run the **uname -a** command on the CLI to view the CPU architecture of the development environment and operating environment. If **x86\_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.
   
   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM toolkit needs to be called during app build time because the toolkits of both the x86 and ARM architectures are deployed in the development environment. Therefore, you need to import the path of the ARM ACLlib.
     
     **export DDK\_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**
     
     **export NPU\_HOST\_LIB=$DDK\_PATH/acllib/lib64/stub**
     
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
     
     > - If the version is 3.0.0, change **arm64-linux** in the **DDK\_PATH** environment variable to **arm64-linux\_gcc7.3.0**.
     > - You can run the **uname -a** command on the CLI to view the CPU architecture of the development environment and operating environment. If **x86\_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.

2. Go to the GPIO directory and create a directory for storing compilation files. For example, the created directory in this document is **build/intermediates/host**.
   
   **cd $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/gpio**
   
   **mkdir -p build/intermediates/host**

3. Go to the **build/intermediates/host** directory and run the **cmake** command to generate a compilation file.
   
   - If the development environment and operating environment have the same OS architecture, run the following commands to perform compilation.
     
     **cd build/intermediates/host**
     
     **make clean**
     
     **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**
   
   - When the OS architecture of the development environment is different from that of the operating environment, you need to use the cross compiler for compilation. For example, if the development environment uses the x86 architecture and the operating environment uses the ARM architecture, run the following commands to perform cross compilation:
     
     **cd build/intermediates/host**
     
     **make clean**
     
     **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

4. Run the **make** command. The generated executable file **main** is stored in the **gpio/out** directory.
   
   **make**

### Running the Sample

**Note: If the development environment and operating environment are deployed together, skip step 1 and go to [step 3](#step_3).**

1. Run the following commands to upload the GPIO directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:
   
   **scp -r $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/gpio HwHiAiUser@*xxx.xxx.xxx.xxx*:/home/HwHiAiUser**
   
   **ssh HwHiAiUser@*xxx.xxx.xxx.xxx***
   
   ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note:**
   
   > - *xxx.xxx.xxx.xxx* is the IP address of the operating environment, which is **192.168.1.2** when the Atlas 200 DK is connected using the USB, and is the IP address of the corresponding public network for Atlas 300 (AI1s).

2. Obtain the operation permissions of GPIO, I2C, and UART.
   
   Log in to the operating environment and edit the **/etc/rc.local** file.
   
   **ssh HwHiAiUser@*xxx.xxx.xxx.xxx***
   
   **Run the su root command and enter the password to switch to the root user.**
   
   Run the **vim /etc/rc.local** command to add the following commands before **exit0**:
   
   ```
   echo 504 >/sys/class/gpio/export
   echo 444 >/sys/class/gpio/export
   chown -R HwHiAiUser /sys/class/gpio/gpio444
   chown -R HwHiAiUser /sys/class/gpio/gpio504
   chown -R HwHiAiUser /sys/class/gpio/gpio444/direction
   chown -R HwHiAiUser /sys/class/gpio/gpio504/direction
   chown -R HwHiAiUser /sys/class/gpio/gpio444/value
   chown -R HwHiAiUser /sys/class/gpio/gpio504/value
   chown -R HwHiAiUser /dev/i2c-1
   chown -R HwHiAiUser /dev/i2c-2
   chown -R HwHiAiUser /dev/ttyAMA0
   chown -R HwHiAiUser /dev/ttyAMA1
   usermod -aG HwHiAiUser HwHiAiUser
   ```
   
   **Restart the operating environment.**

3. <a name="step_2"></a>Run the executable file.
   
   - If the development environment and operating environment are deployed together, run the following commands to set the operating environment variables and change the directory:
     
     **export LD\_LIBRARY\_PATH=**
     
     **source ~/.bashrc**
     
     **cd $HOME/samples/cplusplus/level1\_single\_api/5\_200dk\_peripheral/gpio/out**
   
   - If the development environment and operating environment are deployed separately, run the following command to change the directory:
     
     **cd $HOME/gpio/out**
   
   Run the following command to run the sample:
   
   **./main**

### Viewing the Results

After the running is complete, the results are displayed in the CLI of the operating environment.