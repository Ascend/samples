English|[中文](README_CN.md)


**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This readme file provides only guidance for running the sample in the command line. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Running%20Image%20Samples%20in%20MindStudio?sort_id=3736297).**

## Sample of Removing a Specified Foreground Object from an Image

Purpose: Use Mask R-CNN and ImageInpainting models to perform inference on removing a specified foreground object in an input image.
Sample input: image from which you want to remove a specified foreground object
Sample output: mask image of the object to be removed, split image of the instance, and image after the object is removed

### Prerequisites
Check whether the following requirements are met. If not, perform operations according to the remarks. If the CANN version is upgraded, check whether the third-party dependencies need to be reinstalled. (The third-party dependencies for 5.0.4 and later versions are different from those for earlier versions.)
| Item      | Requirement                                                        | Remarks                                                        |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CANN version  | ≥ 5.0.4                                                     | Install the CANN by referring to [Installation](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85) in the **About Ascend Samples Repository**. If the CANN version is earlier than the required version, switch to the sample repository specific to the CANN version. See [Release Notes](https://github.com/Ascend/samples/blob/master/README.md). |
| Hardware  | Atlas 200 DK/Atlas 300 ([AI1S](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | Currently, the Atlas 200 DK and Atlas 300 have passed the test. For details about the product description, see [Hardware Platform](https://ascend.huawei.com/en/#/hardware/product). For other products, adaptation may be required.|
| Third-party dependency| opencv, pillow, numpy, mmcv, easydict, matplotlib, python-acllite| Select required dependencies. See [Third-Party Dependency Installation Guide (Python Sample)](https://gitee.com/ascend/samples/tree/master/python/environment).|

**Note: To run this sample, you must run the following commands to install the required Python dependencies:**

```
python3.6 -m pip install mmcv --user
python3.6 -m pip install easydict==1.9 --user
python3.6 -m pip install matplotlib==3.3.4 --user
```

### Sample Preparation

1. Obtain the source package.

   You can download the source code in either of the following ways:  
    - - Command line (The download takes a long time, but the procedure is simple.)
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
      ```
       **To switch to another tag (for example, v0.5.0), run the following command:**
       ```
       git checkout v0.5.0
       ```
    - - Compressed package (The download takes a short time, but the procedure is complex.)  
       **Note: If you want to download the code of another version, switch the branch of the samples repository according to the prerequisites.**   
       ``` 
        # 1. Click **Clone or Download** in the upper right corner of the samples repository and click **Download ZIP**.   
        # 2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
       ```

2. Obtain the source model required by the application.
    | **Model Name**              | **Model Description**                                                | **Download Link**                                            |
    | -------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
    | mask_rcnn+image_inpainting | Models specified for removing a specified foreground object from an input image.| Mask R-CNN: [https://www.hiascend.com/software/modelzoo/detail/C/8364bc31cfe144f9a3e18e8543ffd314](https://www.hiascend.com/software/modelzoo/detail/C/8364bc31cfe144f9a3e18e8543ffd314)<br>ImageInpainting: [https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/imageinpainting_HiFill](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/imageinpainting_HiFill)|
    ```
    # To facilitate download, the commands for downloading the original model and converting the model are provided here. You can directly copy and run the commands. You can also refer to the above table to download and manually convert the model.    
    cd ${HOME}/samples/python/level3_multi_model/mask_rcnn_image_inpainting/model    
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/maskrcnn_mindspore.air
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/aipp_rgb.cfg     
    atc --input_format=NCHW --framework=1 --model="maskrcnn_mindspore.air" --input_shape="x:1, 3, 768, 1280; im_info: 1, 4" --output="maskrcnn_mindspore_rgb" --insert_op_conf="aipp_rgb.cfg" --precision_mode=allow_fp32_to_fp16 --soc_version=Ascend310 --op_select_implmode=high_precision
    
    wget https://obs-9be7.obs.myhuaweicloud.com/models/imageinpainting_hifill/hifill.pb
    atc --output_type=FP32 --input_shape="img:1,512,512,3;mask:1,512,512,1" --input_format=NHWC --output="./hifill" --soc_version=Ascend310 --framework=3 --save_original_model=false --model="./hifill.pb"
    
    wget https://obs-9be7.obs.myhuaweicloud.com/models/imageinpainting_hifill/matmul_27648.json
    atc --singleop=./matmul_27648.json --output=./0_BatchMatMul_0_0_1_1_1024_1024_0_0_1_1_1024_27648_0_0_1_1_1024_27648 --soc_version=Ascend310
    ```

3. Obtain the test images required by the sample.
    ```
    # Run the following commands to go to the data folder of the sample and download the corresponding test images:
    cd $HOME/samples/python/level3_multi_model/mask_rcnn_image_inpainting/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mask_rcnn/photo1.jpg
    ```
### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **mask_rcnn_image_inpainting** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
    ```
    # In the following information, xxx.xxx.xxx.xxx is the IP address of the operating environment. The IP address of Atlas 200 DK is 192.168.1.2 when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.
    scp -r ${HOME}/samples/python/level3_multi_model/mask_rcnn_image_inpainting HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/mask_rcnn_image_inpainting/src
    ```

2. Run the project.
    ```
    python3.6 mask_rcnn.py 410 664
    ```
### Result Viewing

After the running is complete, the **mask** and **output** folders are created in the sample directory. The instance split image is stored in the **mask** folder, and the image from which the object has been removed is stored in the **output** folder.

Before inference:
![Image description](https://images.gitee.com/uploads/images/2021/1110/141432_23bde59f_8083019.jpeg "photo1.jpg")

After inference:
![Image description](https://images.gitee.com/uploads/images/2021/1110/141623_eee7745f_8083019.jpeg "photo1_out.jpg")
![Image description](https://images.gitee.com/uploads/images/2021/1110/141612_746406cb_8083019.jpeg "photo1_mask.jpg")
![Image description](https://images.gitee.com/uploads/images/2021/1110/141637_737d9472_8083019.jpeg "outpaint_photo1.jpg")

### Common Errors
For details about how to rectify the errors, see [Troubleshooting](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D). If an error is not included in Wiki, submit an issue in the **samples** repository.
