English|[中文](README_CN.md) 

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This README file provides only guidance for running the sample in command line (CLI) mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

## style_transfer_picture Example
Function: uses the style transfer model to infer the input image and generate a styled image.   
Input: JPG images.   
Output: JPG images with inference results.   

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item| Requirement| Remarks|
|---|---|---|
| CANN version| ≥ 5.0.4| Install the CANN by referring to [Sample Deployment](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the About Ascend Samples Repository. If the CANN version is earlier than the required version, switch to the samples repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md).|
| Hardware| Atlas 200 DK/Atlas 300 ([AI1s](https://support.huaweicloud.com/en-us/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| OpenCV and Python-acllite| Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://github.com/Ascend/samples/tree/master/python/environment).|

### Sample Preparation

1. Obtain the source package.

   You can download the source code in either of the following ways:  
    - Command line (The download takes a long time, but the procedure is simple.)
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```
       **To switch to another tag (for example, v0.5.0), run the following command:**
       ```
       git checkout v0.5.0
       ```
    - Compressed package (The download takes a short time, but the procedure is complex.)  
       **Note: If you want to download the code of another version, switch the branch of the samples repository according to the prerequisites.**  
       ``` 
        # 1. Click **Clone** or **Download** in the upper right corner of the samples repository and click **Download ZIP**.   
        # 2. Upload the .zip package to the home directory of a common user in the development environment, for example, **${HOME}/ascend-samples-master.zip**.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
       ```

2. Obtain the source model required by the application.
    |  **Description** |  **How to Obtain** |
    |---|---|
    | GAN-based model. |  Download the model and weight file by referring to the links in **README.md** in the [TensorFlow_contrib_cv_style_transfer](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/style_transfer) directory of the ModelZoo repository.|

    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download the model from ModelZoo and manually convert it.
    #In this example, only the inference effect of the starry sky style is displayed. Modify the inference effect based on the site requirements.
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer_picture/model    
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/style_transfer_picture/xingkong1.pb
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/tangguo.pb
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/bijiasuo.pb
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/work_soldiers.pb    
    atc --model=./xingkong1.pb --framework=3 --output=xingkong1_fp32_nchw_no_aipp --soc_version=Ascend310
    atc --model=./bijiasuo.pb --framework=3 --output=bijiasuo_fp32_nchw_no_aipp --soc_version=Ascend310
    atc --model=./tangguo.pb --framework=3 --output=tangguo_fp32_nchw_no_aipp --soc_version=Ascend310
    atc --model=./work_soldiers.pb --framework=3 --output=work_soldiers_fp32_nchw_no_aipp --soc_version=Ascend310
    ```

3. Obtain the test images required by the sample.
    ```
    # Run the following commands to go to the **data** folder of the sample and download the corresponding test images:
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer_picture/data
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/data/test.jpg
    cd ../src
    ```


### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**  

1. Run the following commands to upload the **samples** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
    ```
    # In the following information, *xxx.xxx.xxx.xxx* is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer/src
    ```

2. <a name="step_2"></a>Run the executable file.
    ```
    (1) Starry style
    python3.6 main.py ../data xingkong
    (2) Candy style
    python3.6 main.py ../data tangguo
    (3) Picasso style
    python3.6 main.py ../data bijiasuo
    (4) Worker-Peasant-Soldier style
    python3.6 main.py ../data worksoldiers
    ```

### Result Viewing

After the execution is complete, the inferred images are saved in the **outputs** folder in the project directory of the operating environment, as shown in the following figures.  
![Input Image Description](https://images.gitee.com/uploads/images/2021/1103/094011_426f5b30_7647177.png "image-20211102165506515.png")

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue to the **samples** repository.
