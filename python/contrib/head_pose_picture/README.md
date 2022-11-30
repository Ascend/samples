**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

This sample works with CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300.

# WHENet: Real-time Fine-Grained Estimation for Wide Range Head Pose
**Function**: tracks multiple pedestrians in a scene with the **mot_v2.om** model.

**Input**: a source image

**Output**: the result image

## Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](https://github.com/Ascend/samples/blob/master/python/environment/README.md).
- The development environment and operating environment of the corresponding product have been set up.

## Software Preparation
* Make sure you log in to the operating environment (**HwHiAiUser**)
    ```
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    ```
    ![Icon-note.gif](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif) **NOTE**

    > Replace ***xxx.xxx.xxx.xxx*** with the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 is the corresponding public network IP address.

### 1. Obtain the source package.
```
cd $HOME
git clone https://github.com/Ascend/samples.git
```

### 2. Obtain the Offline Model (**om**) or Convert **pb** to **om** in [Step 3](#3-(OPTIONAL)-Convert-the-original-pb-model-to-a-DaVinci-offline-model.).

   Ensure you are in the project directory (`head_pose_picture/`) and run one of the following commands in the table to obtain the pedestrian tracking model used in the application.

	cd $HOME/samples/python/contrib/head_pose_picture/

| **Model**  |  **How to Obtain** |
| ---------- |  ----------------- |
| WHENet_b2_a1_modified.om | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/head_pose_picture/moWHENet_b2_a1_modifiedt_v2.om' -O model/moWHENet_b2_a1_modifiedt_v2.om`  |
| yolo_model.om | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/head_pose_picture/yolo_model.om' -O model/yolo_model.om`  |
| WHENet_b2_a1_modified.pb | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/head_pose_picture/WHENet_b2_a1_modified.pb' -O model/WHENet_b2_a1_modified.pb`  |
| yolo_model.pb | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/head_pose_picture/yolo_model.pb' -O model/yolo_model.pb`  |

   ![Icon-note.gif](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif) **NOTE**
   >- `WHENet_b2_a1_modified.om` and `yolo_model.om` offline models you can use out-of-the-box without model conversion. If you use this then you can skip the next step on model conversion.
   >- `WHENet_b2_a1_modified.pb` and `yolo_model.pb` pb models for those that want to configure the model conversion process.

### 3. **(OPTIONAL)** Convert the original pb model to a DaVinci offline model.

   **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](https://gitee.com/ascend/samples/tree/master/python/environment).**

   1. Set the ***LD_LIBRARY_PATH*** environment variable.

      The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when Ascend Tensor Compiler (ATC) is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.
      
          export LD_LIBRARY_PATH=${install_path}/compiler/lib64

   For **CANN 3.0.0 and later**: <br/>

   2. Go to the project directory (head_pose_picture) and run the model conversion command to convert the model:

          cd $HOME/samples/python/contrib/head_pose_picture/
          atc --framework=3 --model=model/yolo_model.pb --input_shape="input_1:1,416,416,3" --input_format=NHWC --output=model/yolo_model --output_type=FP32 --soc_version=Ascend310
          atc --framework=3 --model=model/WHENet_b2_a1_modified.pb --input_shape="input_1:1,224,224,3" --input_format=NHWC --output=model/WHENet_b2_a1_modified --output_type=FP32 --soc_version=Ascend310


## Sample Running
   - ### Download Sample Image
     ```
     cd $HOME/samples/python/contrib/head_pose_picture/
     wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/head_pose_picture/test.jpg -O data/test.jpg
     ```

   - ### Test Sample Image
     ```
     cd $HOME/samples/python/contrib/head_pose_picture/src
     python3 main.py
     ```
