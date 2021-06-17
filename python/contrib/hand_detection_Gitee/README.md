**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

This sample works with CANN 3.3.0 and later versions, and supports Atlas 200 DK and Atlas 300.

# Sample of Hand Detection in Image
**Function**: detect the presence of a hand in a scene with the **Hand_detection.om** model.

**Input**: an image

**Output**: an images with bounding box for the detected hand in the scene


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

### 2. Install Dependencies 
Only if required libraries are not installed
```
pip3 install Pillow
pip3 install opencv-python
```

### 3. Obtain the Protobuf model and test images

   Ensure you are in the project directory (`hand_detection_picture/`) and run the following commands in the table to obtain the hand detection model used in the application.

	cd $HOME/samples/python/contrib/hand_detection_picture/

| **Model**  |  **How to Obtain** |
| ---------- |  ----------------- |
| Hand_detection.pb | `wget --no-check-certificate 'https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hand_detection_picture/Hand_detection.pb' -O model/Hand_detection.pb`  |
| hand.jpg | `wget --no-check-certificate 'https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hand_detection_picture/hand.jpg' -O data/hand.jpg` |
| verify.jpg | `wget --no-check-certificate 'https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hand_detection_picture/verify.jpg' -O data/verify.jpg` |
 
  

### 4. Convert the original model to a DaVinci model

   **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](https://github.com/Ascend/samples/tree/master/python/environment).**

   1. Set the ***LD_LIBRARY_PATH*** environment variable.

      The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when Ascend Tensor Compiler (ATC) is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.
      
          export LD_LIBRARY_PATH=${install_path}/atc/lib64


   2. Go to the project directory (hand_detection) and run the model conversion command to convert the model:

          atc --model=model/Hand_detection.pb --framework=3 --output=model/Hand_detection --soc_version=Ascend310 --input_shape="image_tensor:1,300,300,3" --input_format=NHWC --output_type=FP32



## Sample Running

  ### Run on single test image (hand.jpg)
     
    cd $HOME/samples/python/contrib/hand_detection_picture/src
    python3 hand_detection.py
     
   See `output/output.jpg` to see detection result

## Result Checking
After the execution is complete, the inference result is displayed on the CLI of the operating environment.



<!-- # Hand Detection
This model detects the hand of a person in given image.

## Model Description:

#### Original Model

The original model is MobileNet V1 + SSD, trained with Tensorflow framework.

Orignal model repository: https://github.com/victordibia/handtracking 

License: Apache-2.0

#### Offline Model Conversion

To prepare the Tensorflow model for inference on Ascend 310 processor (product: Atlas 200 DK/Atlas 300), the following steps are taken:

- Modify the original model: 
  - change dynamic-shape input to fixed-shape input
  - remove/replace unsupported operators
  
  The modified Tensorflow frozen-graph model (.pb file) is available: https://drive.google.com/file/d/1Ls-28mkmKq5e6bQsK6iA9p9vqBBxqVFM/view?usp=sharing

- Conversion with ATC tool (or MindStudio GUI):

  Download the .pb file, then run following cmd in the same directory:
  ```
  atc --framework=3 --model="Hand_detection.pb" --input_shape="image_tensor:1,300,300,3" --input_format=NHWC --output="Hand_detection" --output_type=FP32 --soc_version=Ascend310 
  ```


#### Input
- **Input Shape**: [1,300,300, 3]
- **Input Format** : NHWC
- **Input Type**: UINT8

#### Output
- The pre-trained model will detect 2 types: hand and others.
- Output is a list of 4 numpy arrays
  - Output 0: one number, Number of detections, shape (1,)
  - Output 1: detected classes, shape (1,10), **Note**: maximum 10 objects (bounding boxes) are detected
  - Output 2: confidence score for the detected classes, shape (1,10)
  - Output 3: bounding box coordinates, shape (1,10,4): bounding box coordinates for the 10 boxes
    - **0 position**: top left y coordinate
    - **1 position**: top left x coordinate
    - **2 position**: bottom right y coordinate
    - **3 position**: bottom right x coordinate
  
## Sample Code:
  - Codes in https://github.com/Ascend-Huawei/OfflineModelSamples/tree/main/hand_detection/src create a sample to quickly understand how the model works, preprocessing, inference, postprocessing are already included.
  
  - Dependencies: opencv, numpy, Pillow

  - Preprocessing: 
    - **Image Resize**: 300*300
    - **Image Type**: UINT8, RGB
  - Postprocessing:
    - filter bboxes with score higher than the threshold
    - plot bounding boxes on image
    
  - To run code， simply use commands below in the terminal:
  
    ``` 
    cd src
    python3 hand_detection.py 
    ``` 

-->