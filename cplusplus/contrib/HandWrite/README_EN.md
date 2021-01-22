English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 20.0 and later versions. The supported product is Atlas 200 DK.**

**This document provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Video Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138).**

## Handwritten Chinese Character Recognition

Function: This sample recognizes the Chinese characters captured by the camera and displays the recognition result on the Presenter Server WebUI.

Input: Raspberry Pi camera.

Output: recognition result displayed on the Presenter Server WebUI.


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
        1. Click **Clone or download** in the upper right corner of the samples repository and select **Download ZIP**.   
        2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.   
        3. In the development environment, run the following commands to unzip the package:   
            **cd $HOME**   
            **unzip ascend-samples-master.zip**

2. Obtain the source model required by the application.

    Obtain the original model and its weight file used in the application by referring to the following table and save them to any directory of a common user in the development environment, for example, **$HOME/models/HandWrite**.
    
    |  **Model Name**  |  **Description**  |  **How to Obtain**  |
    |---|---|---|
    | ResNet-18 | Handwritten Chinese character recognition model  | Download the model and weight file by referring to the **README.md** file in [https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/resnet18/%20ATC_resnet18_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/resnet18/%20ATC_resnet18_caffe_AE). |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - The converted OM model provided by ModelZoo does not match the current sample. Therefore, you need to download the original model and weight file, and convert the model by yourself.

3. Convert the original model to a Da Vinci model.
    
    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](../../environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following commands to download the AIPP configuration file and convert the model:

        **cd $HOME/models/HandWrite**  

        **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/handwrite/insert_op.cfg**

        **atc --model=./resnet.prototxt --weight=./resnet.caffemodel --framework=0 --output=resnet --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --input_shape="data:1,3,112,112" --input_format=NCHW**

    3. Run the following command to copy the converted model to the **model** folder of the sample:

        **cp ./resnet.om $HOME/samples/cplusplus/contrib/HandWrite/model/**

### Sample Deployment

1. Modify the Presenter-related configuration file.

    Set **presenter_server_ip** and **presenter_view_ip** in **scripts/param.conf** in the sample directory to the IP addresses that can be pinged in the development environment.   
        1. In the development environment, run the **ifconfig** command to view available IP address.   
        2. In the development environment, change **presenter_server_ip** and **presenter_view_ip** in **scripts/param.conf** to the available IP address.   
        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - 1. If the development environment and operating environment are set up on separate servers, the configured virtual NIC IP address is used, for example, **192.168.1.223**.   
        > - 2. If the development environment and operating environment are set up on the same server, the fixed IP address of the Atlas 200 DK is used, for example, **192.168.1.2**.

  
 
2. Set the environment variables for building the dependencies on the command line of the development environment.

  
     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**  
 
     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**   
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - If the CANN version is 20.0, change **arm64-linux** in the ***DDK_PATH*** environment variable to **arm64-linux_gcc7.3.0**.

3. Switch to the **HandWrite** directory and create a directory for storing build outputs, for example, **build/intermediates/host** in this sample.

    **cd $HOME/samples/cplusplus/contrib/HandWrite**

    **mkdir -p build/intermediates/host**

4. Go to the **build/intermediates/host** directory and run the **cmake** command.


      **cd build/intermediates/host**   
      **make clean**   
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

5. Run the **make** command and find the generated executable file **main** in the **HandWrite/out** directory.

    **make**


### Sample Running

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
> - ***xxx.xxx.xxx.xxx*** indicates the IP address of the operating environment, which is generally 192.168.1.2 for Atlas 200 DK when it is connected over the USB port.

1. Run the following commands to upload the **HandWrite** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**:   

    **If the development environment and operating environment are set up on the same server, skip this step.**   

    **scp -r $HOME/samples/cplusplus/contrib/HandWrite HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. Start the Presenter Server and log in to the operating environment.    
        1. Run the following commands in the development environment to start Presenter Server:   
            **cd $HOME/samples/cplusplus/contrib/HandWrite**   
            **bash scripts/run_presenter_server.sh**   
        2. Run the following command to log in to the operating environment:   
            **If the development environment and operating environment are set up on the same server, skip this step.**   
            **ssh HwHiAiUser@xxx.xxx.xxx.xxx** 


3. Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variables and switch the directory:   
      **export LD_LIBRARY_PATH=**   
      **source ~/.bashrc**     
      **cd $HOME/samples/cplusplus/contrib/HandWrite/out**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory: 
  
      **cd $HOME/HandWrite/out**

    Run the following command to run the sample:

    **./main**

### Result Checking

1. Open the Presenter Server WebUI.

      Open the URL that is displayed when the Presenter Server is started.
      

2. Wait for Presenter Agent to transmit data to the server and click **Refresh**. When data arrives, the icon in the **Status** column for the corresponding **Channel** turns green.

3. Click a link in the **View Name** column to view the result.
