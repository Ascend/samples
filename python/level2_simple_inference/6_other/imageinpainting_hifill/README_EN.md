English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 20.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This document provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## HD Image Repair (Python)

Function: repairs ultra-HD images.

Input: JPG image to be repaired and the corresponding mask.

Output: repaired image.

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

2. Obtain the single-operator JSON file and .om offline model file required by the application.  

     **wget -P ~/imageinpainting_hifill https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json**   
     **wget -P ~/imageinpainting_hifill https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.om**
    

3. Convert the single-operator JSON file into a Da Vinci model.
    
    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](.../../environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following commands to convert the model:  

        **cd ~/imageinpainting_hifill** 

        **atc --singleop=./matmul_27648.json --output=./0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648 --soc_version=Ascend310**   

    3. Run the following commands to copy the converted model to the **model** folder of the sample:

        **cp ./hifill.om \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/model/** 
 
        **cp ./0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648/*.om \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/model/**

4. Obtain the test images required by the sample.

    Run the following commands to download the test images:

    **cd \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/data**

    **wget https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/data/test.jpg**
    
    **cd \$HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/mask**

    **wget https://c7xcode.obs.myhuaweicloud.com/models/imageinpainting_hifill/mask/test.jpg** 



### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **imageinpainting_hifill** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - ***xxx.xxx.xxx.xxx*** indicates the IP address of the operating environment, which is generally 192.168.1.2 for Atlas 200 DK when it is connected over the USB port. For Atlas 300 (ai1s), it is the corresponding public IP address.

2. <a name="step_2"></a>Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variables and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/python/level2_simple_inference/6_other/imageinpainting_hifill/src**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:
    
      **cd $HOME/imageinpainting_hifill/src**      

    Run the following command to run the sample:

    **python3.6 main.py**
### Result Checking

After the execution is complete, find the result JPG image in the **out** directory.
