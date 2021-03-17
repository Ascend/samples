English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This readme file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## Sample of image_HDR_enhance

Function: realize HDR enhance for image.

Input: PNG image

Output: PNG image after HDR enhance

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

2. Obtain the source model required by the application。      
    
   Refer to the link below for the original network model used in this application。
        
   ```bash
   cd $HOME/samples/python/contrib/image_HDR_enhance/model
   
   wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/image_HDR_enhance/image_HDR_enhance.pb
   ``` 

   The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.

   ```bash
   export LD_LIBRARY_PATH=\\${install_path}/atc/lib64
   ```

   Run the following commands to convert the model:     
   
   ```bash         
   atc --model=./image_HDR_enhance.pb --framework=3 --output=image_HDR_enhance --soc_version=Ascend310  --input_shape="input:1,512,512,3" --input_format=NHWC --output_type=FP32
   ```

3. Obtain the test image required by the sample.

    Run the following commands to go to the **data** folder of the sample and download the corresponding test image:

    **cd $HOME/samples/python/contrib/image_HDR_enhance/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/image_HDR_enhance/data1.png**



### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **cartoonGAN_picture** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/samples/python/contrib/image_HDR_enhance/  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**
    **scp -r $HOME/samples/python/common/atlas_utils/   HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.

2. Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variables and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/python/contrib/image_HDR_enhance/src**    
      **python3.6 main.py**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:

      **cd $HOME/samples/python/contrib/image_HDR_enhance/src**

    Run the following command to run the sample:

    **python3.6 main.py**

### Result Checking

After the execution is complete, find the PNG image with inference results in the **outputs** directory.
