English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 20.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This readme file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## Sample of Image Cartoonization with CartoonGAN

Function: cartoonizes an input image by using the CartoonGAN model.

Input: JPG image

Output: JPG image with inference results

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
    - For CANN 20.1   
    Due to versioning issues, this model was not converted correctly in version 20.1.So version 20.1 gets the OM model directly.      
     
        **cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/model**    
        **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/cartoonGAN_picture/cplus/cartoonization.om** 
    
    - For CANN 20.0    

        1. Refer to the link below for the original network model used in this application。
        
            **cd $HOME/samples/cplusplus/contrib/cartoonGAN_picture/model** 。   
            **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/cartoon/cartoonization.pb**  

        2. Set the ***LD_LIBRARY_PATH*** environment variable.
        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.
           
             **export install_path=$HOME/Ascend/ascend-toolkit/latest**

             **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

        3. Run the following commands to convert the model:     
            
            **atc --output_type=FP32 --input_shape="train_real_A:1,256,256,3" --input_format=NHWC --output="./cartoonization" --soc_version=Ascend310 --framework=3 --save_original_model=false --model="./cartoonization.pb" --precision_mode=allow_fp32_to_fp16**
3. Obtain the test image required by the sample.

    Run the following commands to go to the **data** folder of the sample and download the corresponding test image:

    **cd $HOME/samples/python/contrib/cartoonGAN_picture/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/cartoonGAN_picture/scenery.jpg**



### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **cartoonGAN_picture** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/samples/python/contrib/cartoonGAN_picture/ HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.

2. Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variables and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/python/contrib/cartoonGAN_picture/src**    
      **python3.6 cartoonization.py ../data/**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:

      **cd $HOME/YOLOV3_coco_detection_picture/src**

    Run the following command to run the sample:

    **python3.6 cartoonization.py ../data/**

### Result Checking

After the execution is complete, find the JPG image with inference results in the **outputs** directory.
