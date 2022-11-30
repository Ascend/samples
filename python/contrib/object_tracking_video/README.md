**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

This sample works with CANN 3.3.0 and later versions, and supports Atlas 200 DK and Atlas 300.

# Sample of Multi-Object Tracking in Video
**Function**: tracks multiple pedestrians in a scene with the **mot_v2.om** model.

**Input**: a crowd video or image

**Output**: images or video with bounding box and ID for each person in the scene

**Preformance and Result**: **~8 fps** depending on how crowd the video is; see result at https://github.com/HardysJin/atlas-track 

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
```
cd $HOME/samples/python/contrib/object_tracking_video/
pip3 install -r requirements.txt
```

### 3. Obtain the Offline Model (om) or Convert ONNX to om in [Step 4](#4-Convert-the-original-model-to-a-DaVinci-model).

   Ensure you are in the project directory (`object_tracking_video/`) and run one of the following commands in the table to obtain the pedestrian tracking model used in the application.

	cd $HOME/samples/python/contrib/object_tracking_video/

| **Model**  |  **How to Obtain** |
| ---------- |  ----------------- |
| mot_v2.om | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/object_tracking_video/mot_v2.om' -O model/mot_v2.om`  |
| mot_v2.onnx | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/object_tracking_video/mot_v2.om' -O model/mot_v2.onnx`  |

   ![Icon-note.gif](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif) **NOTE**
   >- `mot_v2.om` offline model you can use out-of-the-box without model conversion. If you use this then you can skip the next step on model conversion.
   >- `mot_v2.onnx` ONNX model for those that want to configure the model conversion process.
   
   From the project directory, navigate to the `scripts/` directory and run `get_sample_data.sh` to download sample images for testing the application later in section 4. The sample images will be saved in `data/`.
   
   ```
   cd $HOME/samples/python/contrib/object_tracking_video/script/
   bash get_sample_data.sh
   ```
### 4. Convert the original model to a DaVinci model. (OPTIONAL)

   **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](https://github.com/Ascend/samples/tree/master/python/environment).**

   1. Set the ***LD_LIBRARY_PATH*** environment variable.

      The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when Ascend Tensor Compiler (ATC) is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.
      
          export LD_LIBRARY_PATH=${install_path}/compiler/lib64

   For **CANN 3.3.0-alpha006**: <br/>

   2. Go to the project directory (object_tracking_video) and run the model conversion command to convert the model:

          atc --input_shape="input.1:1,3,608,1088" --check_report=./network_analysis.report --input_format=NCHW --output=model/mot_v2 --soc_version=Ascend310 --framework=5 --model=model/mot_v2.onnx


## Sample Running

   - ### Simple & Quick Run on test video ([london.mp4](https://drive.google.com/file/d/1ntbudc1JB8HzEw38pwZKPXukrgADiKdS/view))
     ```
     cd $HOME/samples/python/contrib/object_tracking_video/scripts
     bash run_demo.sh
     ```
     See result in `object_tracking_video/outputs/london`

   - ### Run on your own video
     ```
     cd $HOME/samples/python/contrib/object_tracking_video/src
     python3 main.py --input_video "\Path to video"
     ```
     See result in `object_tracking_video/outputs/VIDEO_NAME`

   - ### Run on single test image (test.jpg)
     ```
     cd $HOME/samples/python/contrib/object_tracking_video/src
     python3 test.py --test_img ../data/test.jpg --verify_img ../data/verify.jpg
     ```
     See `data/test_output.jpg`


## Train
This model is a dlav0 version of [FairMOT](https://github.com/ifzhang/FairMOT), you can follow their guide to setup the training environment, then use this [script](https://github.com/HardysJin/FairMOT-dlav0/blob/master/src/convert.py) to convert to ONNX.

<!-- Pedestrian Detection and Tracking on Atlas 200DK, a dlav0 version of [FairMOT](https://github.com/ifzhang/FairMOT).

## Introduction
Multi Object Tracking (MOT) is a challenging topic as it often has two seperate tasks for detection and tracking. Recent attention focus on accomplishing the two tasks in a single network to improve the inference speed. [FairMOT](https://github.com/ifzhang/FairMOT), compared to [JDE](https://github.com/Zhongdao/Towards-Realtime-MOT), uses anchor-free CenterNet as the backbone to balance the detection and re-id branches and Kalman Filter for bounding box state prediction, resulting state-of-the-arts accuracy and near real-time speed (30 fps) using good GPUs. The dlav0 version has slightly lower accuracy but ~2x faster. The speed on Atlas 200DK is ~8 FPS depending on number of detections.

## Tracking performance
### Sample Comparison for Unseen Video
<img src="assets/london_compare.gif" width="1000"/> 
Or <a href="https://www.youtube.com/watch?v=ndSdGqUV0cg">Youtube</a>

### Quantitative Comparison on [MOT Challenge](https://motchallenge.net/) using GTX1080
<img src="assets/quantitative_compare.png" width="400"/> 

### Important Notes
As the tracking/association part uses CPU and cannot be benefitted by HPU, the number of detection impacts the speed a lot.

## Installation
Python 3.6.9
### Download Model
```
cd model
./download.sh
cd ..
```

### Install Dependencies
```
pip3 install -r requirements.txt
```

### Run
```
python3 main.py --input_video "\Path to video"
```

### Acknowledgement
A large part of the code is borrowed from [FairMOT](https://github.com/ifzhang/FairMOT), [JDE](https://github.com/Zhongdao/Towards-Realtime-MOT), and [CenterNet](https://github.com/xingyizhou/CenterNet). Thanks for their wonderful works.
