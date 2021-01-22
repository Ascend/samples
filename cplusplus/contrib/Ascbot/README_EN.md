English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 20.0 and later versions. The supported product is AscendBot.**

# Introduction to AscendBot
AscendBot is an open-source smart robotic car designed for AI and robot enthusiasts. It is also an open AI and robot development platform with the following features:
- High performance: Based on Huawei Atlas 200 DK, AscendBot enables 8 TOPS@FP16 compute power.
- Easy to learn: Complete development tutorials and sample code are available ranging from the AI algorithm level to the application level.
- Easy to use: A hardware list and a setup tutorial are provided for developers to assemble the hardware by themselves.

## Sample Function

AscendBot is remotely controlled by the APK on the mobile phone to implement object tracing, trajectory tracing, and fall protection functions.

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](../../environment).

- The development environment and operating environment of the corresponding product have been installed.
- A complete AscendBot car, with the components, battery, router, display, and camera correctly installed, is available.

### Software Preparation

1. Obtain the source code package.

   You can download the source code in either of the following ways:

    - Command line (The download takes a long time, but the procedure is simple.)

        In the development environment, run the following commands as a non-root user to download the source code repository:

       **cd $HOME**

       **git clone https://gitee.com/ascend-incubator/car.git**

    - Compressed package (The download takes a short time, but the procedure is complex.)

        1. Click **Clone or download** in the upper right corner of the samples repository and select **Download ZIP**.

        2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-incubator-car-master.zip**.

        3. In the development environment, run the following commands to unzip the package:

            **cd $HOME**

            **unzip ascend-incubator-car-master.zip**

2. Obtain the source model required by the application.

    Obtain the original model and its weight file used in the application by referring to the following table and save them to any directory of a common user in the development environment, for example, **$HOME/models/ascbot**.
    
    |  **Model Name**  |  **Description**  |  **How to Obtain**  |
    |---|---|---|
    | collision_avoidance_model | Detects risks in the forward direction to implement fall avoidance.  | Download the model and weight files by referring to the **README.md** file in [https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/ascbot/ATC_CollisionAntiDrop_caffe_AE/](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/ascbot/ATC_CollisionAntiDrop_caffe_AE/). |
    | road_following_model | Detects lane lines to implement trajectory tracing. | Download the model and weight file by referring to the **README.md** file in [https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/ascbot/ATC_LaneDetection_caffe_AE/](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/ascbot/ATC_LaneDetection_caffe_AE/).  |
    | road_object_detection_deploy| Selects the run mode from the free mode, trajectory tracing mode, and object tracing mode. | Download the model and weight file by referring to the **README.md** file in [https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/ascbot/ATC_Object_detection_caffe_AE/](https://gitee.com/ascend/modelzoo/tree/master/contrib/Research/cv/ascbot/ATC_Object_detection_caffe_AE/).  |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - The converted OM model provided by ModelZoo does not match the current sample. Therefore, you need to download the original model and weight file, and convert the model by yourself.

3. Convert the original model to a Da Vinci model.
    
    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](.../../environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following commands to download the AIPP configuration file and convert the model:

        **cd $HOME/models/ascbot**  

        **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/car/collision/insert_op_collision_avoidance.cfg**

        **atc --model="collision_avoidance_model.prototxt" --weight="collision_avoidance_model.caffemodel" --soc_version=Ascend310 --framework=0 --output="collision_avoidance_model" --insert_op_conf=insert_op_collision_avoidance.cfg**
         
        **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/car/following/insert_op_road_following.cfg**

        **atc --model="road_following_model.prototxt" --weight="road_following_model.caffemodel" --soc_version=Ascend310 --framework=0 --output="road_following_model" --insert_op_conf=insert_op_road_following.cfg**

        **wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/car/object_detection/insert_op_road_object_detection_deploy.cfg**

        **atc --model="road_object_detection_deploy.prototxt" --weight="road_object_detection_deploy.caffemodel" --soc_version=Ascend310 --framework=0 --output="road_object_detection_deploy" --insert_op_conf=insert_op_road_object_detection_deploy.cfg**


    3. Run the following commands to copy the converted model to the **model** folder of the sample:

        **cp ./collision_avoidance_model.om $HOME/car/ascbot_c75/model/**
    
        **cp ./road_following_model.om $HOME/car/ascbot_c75/model/**

        **cp ./road_object_detection_deploy.om $HOME/car/ascbot_c75/model/**


### Sample Deployment
 
1. Set the environment variables for building the dependencies on the command line of the development environment.



     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
        > - If the CANN version is 20.0, change **arm64-linux** in the ***DDK_PATH*** environment variable to **arm64-linux_gcc7.3.0**.    
        > - You can run the **uname -a** command on the command line to view the CPU architecture of the development environment and operating environment. If **x86_64** is displayed in the command output, the x86 architecture is used. If **arm64** is displayed in the command output, the ARM architecture is used.

2. Switch to the **ascbot_c75** directory and create a directory for storing build outputs, for example, **build/intermediates/host** in this sample.

    **cd $HOME/car/ascbot_c75**

    **mkdir -p build/intermediates/host**

3. Go to the **build/intermediates/host** directory and run the **cmake** command.
      **cd build/intermediates/host**

      **make clean**
    
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

4. Run the **make** command and find the generated executable file **main** in the **ascbot_c75/out** directory.

    **make**

### Sample Running

1. Run the following commands to upload the **ascend_bot** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/car/ascbot_c75 HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  
    > - ***xxx.xxx.xxx.xxx*** indicates the IP address of the operating environment, which is generally 192.168.1.2 for Atlas 200 DK when it is connected over the USB port.


2. Set up the environment.

    Access **/etc/rc.local**.
    
    **vim /etc/rc.local**

    Add the following lines:
    
"""

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

    usermod -aG HwHiAiUser HwHiAiUser
    
"""

3. Run the executable file.

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:
    
      **cd $HOME/ascbot_c75/out**

    Run the following command to run the sample:

    **./main**

### Result Checking

After the execution is complete, you can download the APK on the mobile phone to control the status of the car.
[APK download link](https://share.weiyun.com/5lsbfzF)



