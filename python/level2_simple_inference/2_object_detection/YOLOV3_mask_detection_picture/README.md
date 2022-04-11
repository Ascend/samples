English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300.**

**This readme file provides only guidance for running the sample in command line mode. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874).**

### Face Mask Detection Sample

Function: detects the face and face mask in an image, and recognizes the masked face.

Input: a source JPG image

Output: the result JPG image     

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](../../../environment).

- The development environment and operating environment of the corresponding product have been installed.

1. Obtain the source code package.

   You can download the source code in either of the following ways:

    - Command line (The download takes a long time, but the procedure is simple.)

        In the development environment, run the following commands as a non-root user to download the source code repository:

       **cd $HOME**

       **git clone https://github.com/Ascend/samples.git**

    - Compressed package (The download takes a short time, but the procedure is complex.)

        1. Click **Clone or download** in the upper right corner of the samples repository and click **Download ZIP**.

        2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.

        3. In the development environment, run the following commands to unzip the package:

            **cd $HOME**

            **unzip ascend-samples-master.zip**         

2. Obtain the original model required by the application.

    Obtain the original model by referring to the following table and save it to any directory of a common user in the development environment, for example, **$HOME/models/YOLOV3_mask_detection_picture**.

    | **Model Name**              | **Description**&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;          | **How to Obtain**                                            |
    | --------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
    | mask_detection.pb           | Face mask detection model based on TensorFlow YOLOv3.        | Download the original model by referring to the **README.md** file in [https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov3_resnet18/ATC_yolo3_resnet18_tf_AE](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov3_resnet18/ATC_yolo3_resnet18_tf_AE). |
    | mask_detection_quanzited.pb | Quantized model of mask detection based on tensorflow-yolov3 | Download address:https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com:443/003_Atc_Models/AE/ATC%20Model/YOLOV3-RESNET18%20/yolo3_resnet18_quantized.pb (The operation of converting om offline model is the same as that of unquantized PB model. The specific operation of quantization can be referred to：https://github.com/Ascend/samples/wikis/%E4%BD%BF%E7%94%A8AMCT%E5%B7%A5%E5%85%B7%E9%87%8F%E5%8C%96YOLOV3%E6%A8%A1%E5%9E%8B?sort_id=4402780 ,If you choose to use the quantized model, note the replacement of the model name in the example) |

3. Convert the original model to a Da Vinci model.

    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](../../../environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.

        **export install_path=\\$HOME/Ascend/ascend-toolkit/latest**

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following commands to convert the model:

        **cd $HOME/models/YOLOV3_mask_detection_picture**  

        **atc --input_shape="images:1,352,640,3" --input_format=NHWC --output="./mask_detection" --soc_version=Ascend310 --framework=3 --model="./yolo3_resnet18.pb"**

    3. Run the following command to copy the converted model to the **model** folder of the sample:

        **cp ./mask_detection.om $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_picture/model/**

4. Obtain the test image required by the sample.

    Run the following commands to go to the **data** folder of the sample and download the corresponding test image:

    **cd $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_picture/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/YOLOV3_mask_detection_picture-python/mask.jpg**


### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **YOLOV3_mask_detection_picture** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user (running user):

    **scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  

    > - In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 is the corresponding public network IP address.

2. <a name="step_2"></a>Run the executable file.

    - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variable and switch the directory:

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/python/level2_simple_inference/2_object_detection/YOLOV3_mask_detection_picture/src**     
      **python3.6 mask_detect.py**

    - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory before you run the executable file:

      **cd $HOME/YOLOV3_mask_detection_picture/src**

      **python3.6 mask_detect.py** 
      ​       

### Result Checking

After the execution is complete, find the result JPG image in the **out** directory.
