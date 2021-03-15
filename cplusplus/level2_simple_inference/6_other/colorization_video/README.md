English|[中文](README_CN.md)

**The following sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**The sample applies to CANN 20.0 and later versions and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This README provides only guidance for running samples in command line mode. For details about how to run samples in MindStudio, see [Running Video Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## Video Colorization Sample

Function: Perform inference on the input black and white video by using the image colorization model.

Input: Black and white video (.mp4)

Output: Inference result displayed on Presenter

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
        1. Click **Clone or download** in the upper right corner of the samples repository and select **Download ZIP**.
        2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.
        3. In the development environment, run the following commands to decompress the .zip package:

            **cd $HOME**  
            **unzip ascend-samples-master.zip**

2. Obtain the source network model required by the application.

    Obtain the original network model and its weight file used in the application by referring to the following table and store them in any directory of a common user in the development environment, for example, **$HOME/models/colorization_video**.

    | **Model Name** | **Description** | **How to Obtain** |
    |---|---|---|
    | colorization | Inference model for black and white video colorization. It is a colorization model based on Caffe.   | Download the original model and weight file by referring to [https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/colorization/ATC_colorization_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/colorization/ATC_colorization_caffe_AE).  |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

    > - The converted OM model provided by ModelZoo does not match the current sample. Therefore, you need to download the original model and weight file to convert the model by yourself.

3. Convert the original model to a Da Vinci model.

    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](../../../environment).**

    1. Set the **LD_LIBRARY_PATH** environment variable.

        The **LD_LIBRARY_PATH** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable in the command line to facilitate modification.

        **export install_path=$HOME/Ascend/ascend-toolkit/latest**
    
        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following ATC command to convert the model.

        **cd $HOME/models/colorization_video**  

        **atc --input_shape="data_l:1,1,224,224" --weight="/home/ascend/models/colorization_video/colorization.caffemodel" --input_format=NCHW --output=colorization --soc_version=Ascend310 --framework=0 --model="/home/ascend/models/colorization_video/colorization.prototxt"**

    3. Run the following command to copy the converted model to the **model** folder of the sample.

        **cp ./colorization.om /home/ascend/samples/cplusplus/level2_simple_inference/6_other/colorization_video/model/**

4. Obtain the test files required by the sample.

    Run the following command to go to the **data** folder of the sample, download the corresponding test file, and return to the sample folder:

    **cd $/home/ascend/samples/cplusplus/level2_simple_inference/6_other/colorization_video/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/colorization_video/black-white_video.mp4**

### Sample Deployment

1. Modify Presenter-related configuration files.

    Change **presenter_server_ip** and **presenter_view_ip** in **script/param.conf** in the sample directory to the IP addresses that can ping the operating environment in the development environment. The following are two examples:

     - Atlas 200 DK
        1. In the development environment, run the ifconfig command to view available IP addresses.
        2. In the development environment, change **presenter_server_ip** and **presenter_view_ip** in **script/param.conf** to the available IP addresses.

        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

        > - 1. If the development environment and operating environment are deployed on separate servers, the configured virtual NIC IP address is used, for example, 192.168.1.223.
        > - 2. If the development environment and operating environment are deployed on the same server, the fixed IP address of Atlas 200 DK is used, for example, 192.168.1.2.

    - Atlas 300 AI accelerator card (AI1s cloud-based inference environment)
        1. On the ECS console, check the available intranet IP addresses in the AI1s cloud environment, for example, 192.168.0.198.
        2. In the development environment, change **presenter_server_ip** and **presenter_view_ip** in **script/param.conf** to the available IP addresses.

        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**  
        > - You can also run the ifconfig command in the AI1s cloud-basedenvironment to view the intranet IP addresses.
        > - The IP address for logging in to the AI1s cloud environment is the public IP address of the environment. The IP addresses queried by running the ifconfig command in the AI1s cloud environment are the internal IP addresses of the environment.

2. Set the environment variables on which compilation depends in the command line of the development environment.

   You can run the **uname -a** command in the command line to view the CPU architecture of the development environment and operating environment. If x86_64 is displayed in the command output, the architecture is x86. If arm64 is displayed in the command output, the architecture is ARM Perform the following step based on the actual situation:

   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables:
  
     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM Toolkit needs to be called during app build time because the Toolkits of both the x86 and ARM architectures are deployed in the development environment. Therefore, you need to import the path of the ARM ACLlib.
  
     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**  

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

        > - If the version is 20.0, change **arm64-linux** in the **DDK_PATH** environment variable to **arm64-linux_gcc7.3.0**.

3. Go to the **colorization_video** directory and create a directory for storing build outputs. For example, the directory created in this sample is **build/intermediates/host**.

    **cd $HOME/samples/cplusplus/level2_simple_inference/6_other/colorization_video**

    **mkdir -p build/intermediates/host**

4. Go to the **build/intermediates/host** directory and run the **cmake** command.

    - If the OS architecture of the development environment is the same as that of the operating environment, run the following command to perform compilation.
      **cd build/intermediates/host**  
      **make clean**  
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

    - If the OS architecture of the development environment is different from that of the operating environment, use the cross compiler to perform compilation. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, run the following commands to perform cross compilation:
      **cd build/intermediates/host**  
      **make clean**  
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

5. Run the make command to generate an executable file **main** in the **colorization_video/out** directory.

    **make**

### Sample Running

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **Note**

> - In the following information, **xxx.xxx.xxx.xxx** indicates the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when the USB is connected, and that of Atlas 300 (AI1s) is the corresponding public IP address.

1. Run the following command to upload the **colorization_video** directory in the development environment to the operating environment, for example, **/home/HwHiAiUser**:

    **If the development environment and operating environment are deployed on the same server, skip this step.**  

    **scp -r $HOME/samples/cplusplus/level2_simple_inference/6_other/colorization_video HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. Start the Presenter Server and log in to the operating environment.

    - Atlas 200 DK
        1. Run the following command in the development environment to start the Presenter Server:
            **cd $HOME/samples/cplusplus/level2_simple_inference/6_other/colorization_video**  
            **bash script/run_presenter_server.sh**  
        2. Run the following command to log in to the operating environment:
            **If the development environment and operating environment are deployed on the same server, skip this step.**  
            **ssh HwHiAiUser@xxx.xxx.xxx.xxx**

    - Atlas 300 AI accelerator card (AI1s cloud-based inference environment)
        1. Run the following command to log in to the operating environment:

           **If the development environment and operating environment are deployed on the same server, skip this step.**  
           **ssh HwHiAiUser@xxx.xxx.xxx.xxx**
        2. Start the Presenter Server in the operating environment.
Go to the directory where the project is located (for example, **$HOME/colorization_video**) and run the following command:
            **bash script/run_presenter_server.sh**

3. <a name="step_2"></a>Run the executable file.

    - If the development environment and operating environment are deployed on the same server, run the following commands to set the operating environment variables and switch the directory:

      **export LD_LIBRARY_PATH=**  
      **source ~/.bashrc**  
      **cd $HOME/samples/cplusplus/level2_simple_inference/6_other/colorization_video/out**

    - If the development environment and operating environment are deployed on separate servers, run the following command to switch the directory:

      **cd $HOME/colorization_video/out**

    Switch to the directory and run the following command to run the sample:

    **./main ../data/black-white_video.mp4**

### Checking the Result

1. Open the Presenter Server WebUI.

   - Atlas 200 DK

      Open the URL that is displayed when the Presenter Server service is started.

   - Atlas 300 AI accelerator card (AI1s cloud-based inference environment)

      **Use the Atlas 300 AI accelerator card (AI1s) whose internal IP address is 192.168.0.194 and public IP address is 124.70.8.192 as an example.**

      A message "Please visit http://192.168.0.194:7009 for display server" is displayed when the Presenter Server is started.

      Replace the internal IP address 192.168.0.194 in the URL with the public IP address 124.70.8.192. Then the URL is http://124.70.8.192:7009.

      Open the URL in the Windows browser.

2. Wait for Presenter Agent to transmit data to the server. Click Refresh. When there is data, the icon in the Status column for the corresponding channel changes to green.

3. Click the View Name link on the right to view the result.
