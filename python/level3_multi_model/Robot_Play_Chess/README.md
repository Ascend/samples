English|[中文](README_CN.md)



**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 3.1.0 and later versions. The supported product is Atlas 200 DK.**

## Sample Description
Use a camera to capture checkerboard patterns, use Atlas 200 DK to run the VGG16 model for inference, run the game engine on the main control platform to calculate the moves in chess, and control the robotic arm to play chess.
The user uses the web server to play games with the robotic arm based on the UI on the mobile device.

## Material List
| **Material Type**| **Description**                                                |
| ------------ | ------------------------------------------------------------ |
| Inference platform    | [Atlas 200 DK](https://www.hiascend.com/document/detail/en/Atlas200DKDeveloperKit/1013/environment/atlased_04_0001.html)|
| Camera      | 480 x 640 USB camera                                             |
| Robotic arm      | [Mirobot 6DoF mini robotic arm](https://www.wlkata.com/)|
| Main control platform    | Ubuntu18.04 LTS-x64-Huawei Matebook X Pro                    |
| Chess    | Custom acrylic plate                                                |

## Environment Preparation
### Ubuntu Main Control Platform
- Deploy the development environment. For details, see the [Atlas 200 DK documentation](https://www.hiascend.com/document/detail/en/Atlas200DKDeveloperKit/1013/environment/atlased_04_0001.html).
- Prepare the environment by referring to the README files of the **game engine** and **center control** modules in the **doc** directory.
### Atlas 200 DK Inference Platform
- Deploy the operating environment. For details, see the [Atlas 200 DK documentation](https://www.hiascend.com/document/detail/en/Atlas200DKDeveloperKit/1013/environment/atlased_04_0001.html).
- Check whether the inference platform is successfully set up. See the Ascend sample [vgg16_cat_dog_picture](https://gitee.com/ascend/samples/tree/master/python/level2_simple_inference/1_classification/vgg16_cat_dog_picture).
### Checkerboard Comprehension
- Fix a camera at a proper location above the checkerboard. You can use the camera capture software to view the effect. Ensure that the checkerboard is in the center of the images,
and the chessboard is as large as possible but does not exceed the boundary. (For details, see the images in the **data** directory.)
- Configure the checkerboard comprehension module. For details, see the README file of the checkerboard comprehension module in the **doc** directory.
### Wlkata Mirobot Robotic Arm
- Configure and calibrate the robotic arm. See the [Wlkata Mirobot User Guide](https://lin-nice.github.io/mirobot_gitbook/).
### Web Server
- Configure the web server. For details, see the README file of the web server module in the **doc** directory.

## Sample Description
This sample consists of five modules: center control, checkerboard comprehension, game engine, robotic arm, and web server. For details about the development and design documents, see the README files in the **doc** directory.

## Sample Deployment
### Code Obtaining
- Obtain the source package.

   On the Ubuntu main control platform, run the following commands as a non-root user to download the source repository:
    ```shell
    cd $HOME
    git clone https://github.com/Ascend/samples.git
    ```

- Obtain the network model required by the sample.

    Obtain the model used in the sample by referring to the following table and save it to the project directory of a common user in the development environment.
    ```shell
    cd $HOME/samples/python/level3_multi_model/Robot_Play_Chess/model
    ```
    
    | **Model Name**             | **Model Description**           | **Download Link**                                            |
    | ------------------------- | ----------------------- | ------------------------------------------------------------ |
    | chess_ckpt_0804_vgg_99.om | VGG16 chess classification model | https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/robot_play_chess/chess_ckpt_0804_vgg_99.om |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE** 
    
   > - The OM model has been provided in ModelZoo and can be used directly.
   
- Run the following commands to upload the **samples** directory of the main control platform to a directory on the inference platform, for example, **/home/HwHiAiUser**, and log in to the Atlas 200 DK inference platform as the **HwHiAiUser** user (running user):
  
    ```shell
    scp -r $HOME/samples/ HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx 
    ```
   
    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**   
    
    > - **xxx.xxx.xxx.xxx** indicates the IP address of Atlas 200 DK. When Atlas 200 DK is connected to the Ubuntu main control platform over the USB port, the IP address is 192.168.1.2.

### Sample Running

**Ensure that all devices are in the same LAN.**

- Start the checkerboard comprehension module on Atlas 200 DK.

- Start the game engine on the PC.
  
- Start the robotic arm.

- Start the center control on the PC.

- Start the web server on the PC.

For details, see the README.md file of each module.


### Expected Result
Effect picture 1:
![IMG_20210916_111621](./doc/IMG_20210916_111735.jpg)
Effect picture 2:
![IMG_20210916_111735](./doc/IMG_20210916_111621.jpg)