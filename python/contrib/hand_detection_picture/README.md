# Hand Detection
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

