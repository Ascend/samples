English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 20.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This readme file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## Sample of Landscape Painting Generation with AI

Function: generates landscape paintings based on the input category and layout with the AI_painting model.

Input: vectors that record the category and tensors that record the layout

Output: inference results displayed on the Presenter Server WebUI

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](../../environment).

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

    Obtain the original model and its weight files used in the application by referring to the following table and save them to any directory of a common user in the development environment, for example, **$HOME/models/AI_painting**.

    | **Model Name** | **Description**                   | **How to Obtain**                        |
    | -------------- | --------------------------------- | ---------------------------------------- |
    | AIPainting     | Image generation inference model. | Download the model by referring to the **README.md** file in [https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/painting/ATC_painting_tf_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/painting/ATC_painting_tf_AE). |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - You can use the converted OM model provided by ModelZoo directly, or download the original model and convert the model by yourself.

3. Convert the original model to a Da Vinci model. 

    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](.../../environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the ATC command to convert the model.

        **cd $HOME/models/AI_painting**  

        **atc --output_type=FP32 --input_shape="objs:9;coarse_layout:1,256,256,17"  --input_format=NHWC --output="AIPainting_v2" --soc_version=Ascend310 --framework=3  --model="./AIPainting_v2.pb"** 

    3. Run the following command to copy the converted model to the **model** folder of the sample:

        **cp ./AIPainting_v2.om $HOME/samples/cplusplus/contrib/AI_painting/model/**




### Sample Deployment

1. Modify the Presenter-related configuration file.

    Change **presenter_server_ip** and **presenter_view_ip** in **script/param.conf** in the sample directory to the IP address that can ping the operating environment in the development environment. The following are two examples:

     - For Atlas 200 DK   
        1. In the development environment, run the **ifconfig** command to view the available IP address.   
        2. In the development environment, change **presenter_server_ip** and **presenter_view_ip** in **script/param.conf** to the available IP address.   
          ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - 1. If the development environment and operating environment are set up on separate servers, the configured virtual NIC IP address is used, for example, **192.168.1.223**.
        > - 2. If the development environment and operating environment are set up on the same server, the fixed IP address of the Atlas 200 DK is used, for example, **192.168.1.2**.

    - For Atlas 300 AI accelerator card (AI1s cloud inference environment)   
        1. On the Elastic Cloud Server (ECS) console, check the available intranet IP address in the cloud environment of AI1s, for example, **192.168.0.198**.   
        2. In the development environment, change **presenter_server_ip** and **presenter_view_ip** in **script/param.conf** to the available IP address.   
          ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - You can also run the **ifconfig** command in the AI1s cloud environment to view the intranet IP address.
        > - The IP address for logging in to the AI1s cloud environment is the public IP address of the environment. The IP address obtained by running the **ifconfig** command in the AI1s cloud environment is the intranet IP address of the environment.

2. Set the environment variables for building the dependencies on the CLI of the development environment.

   You can run the **uname -a** command on the CLI to view the CPU architecture of the development environment and operating environment. If **x86_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used. Perform the following step based on the actual situation:

   - If the CPU architecture of the development environment is the same as that of the operating environment, run the following commands to import environment variables: 

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - If the CANN version is 20.0, change **x86_64-linux** in the ***DDK_PATH*** environment variable to **x86_64-linux_gcc7.3.0**.

   - If the CPU architecture of the development environment is different from that of the operating environment, run the following commands to import environment variables. If the development environment uses the x86 architecture and the operating environment uses the ARM architecture, the ACLlib of the ARM Toolkit needs to be called during application build time because the Toolkits of both the x86 and ARM architectures are installed in the development environment. Therefore, you need to import the path of the ARM ACLlib. 

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**  

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**   
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - If the CANN version is 20.0, change **arm64-linux** in the ***DDK_PATH*** environment variable to **arm64-linux_gcc7.3.0**.

3. Switch to the **AI_painting** directory and create a directory for storing build outputs, for example, **build/intermediates/host** in this sample.

    **cd $HOME/samples/cplusplus/contrib/AI_painting/**


    **mkdir -p build/intermediates/host**

4. Go to the **build/intermediates/host** directory and run the **cmake** command.

    - If the development environment and operating environment have the same OS architecture, run the following commands to perform compilation:   
      **cd build/intermediates/host**  
      **make clean**   
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

    - If the OS architecture of the development environment is different from that of the operating environment, use the cross compiler for compilation. For example, if the development environment uses the x86 architecture and the operating environment uses the ARM architecture, run the following commands to perform cross compilation:   
      **cd build/intermediates/host**   
      **make clean**   
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

5. Run the **make** command and find the generated executable file **main** in the **AI_painting/out** directory.

    **make**


### Sample Running

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
> - In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.

1. Run the following commands to upload the **AI_painting** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**.   

    **If the development environment and operating environment are set up on the same server, skip this step.**   

    **scp -r $HOME/samples/cplusplus/contrib/AI_painting HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. Start Presenter Server and log in to the operating environment.

    - For Atlas 200 DK   
        1. Run the following commands in the development environment to start Presenter Server:   
            **cd $HOME/samples/cplusplus/contrib/AI_painting**   
            **bash script/run_presenter_server.sh**   
        2. Run the following command to log in to the operating environment.   
            **If the development environment and operating environment are set up on the same server, skip this step.**   
            **ssh HwHiAiUser@xxx.xxx.xxx.xxx** 

    - For Atlas 300 AI accelerator card (AI1s cloud inference environment)   
        1. Run the following command to log in to the operating environment.   
           **If the development environment and operating environment are set up on the same server, skip this step.**   
           **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    
        2. Start Presenter Server in the operating environment.   
           Run the following command in the directory where the project is located (for example, **\$HOME/AI_painting**):  
           **bash script/run_presenter_server.sh**   

3. Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variables and switch the directory:   
      **export LD_LIBRARY_PATH=**   
      **source ~/.bashrc**     
      **cd $HOME/samples/cplusplus/contrib/AI_painting/out**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:   
      **cd $HOME/AI_painting/out**

    Run the following command to run the sample:

    **./main**

### Result Checking

1. Open the Presenter Server WebUI.

   - For Atlas 200 DK

      Open the URL that is displayed when Presenter Server is started.

   - For Atlas 300 AI accelerator card (AI1s cloud inference environment)

      **The following assumes that the intranet IP address of the Atlas 300 AI accelerator card (AI1s) is 192.168.0.194 and the public IP address is 124.70.8.192.**

      The message "Please visit http://192.168.0.194:7009 for display server" is displayed when Presenter Server is started.

      Replace the intranet IP address **192.168.0.194** in the URL with the public IP address **124.70.8.192**. That is, change the URL to http://124.70.8.192:7009.

      Open the URL in the browser.

2. Wait for Presenter Agent to transmit data to the server and click **Refresh**. When data arrives, the icon in the **Status** column for the corresponding **Channel** turns green.

3. Click a link in the **View Name** column to view the result.
