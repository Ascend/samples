This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.

This sample works with CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300.

# Sample of Real-time Body Pose Detection
**Function**: Infer human body poses from a live-video footage with the **OpenPose_light.om** model.

**Input**: JPG image containing a whole person

**Output**: JPG image with body pose esitmation

**Preformance**: inference time time of running the model on Atlas 200 DK is about 17 ms per image/frame

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

### 1. Obtain the source packages.

```
cd $HOME
git clone https://github.com/Ascend/samples.git
```

### 2. Obtain the original model required by the application.
   Ensure you are in the project directory (`body_pose_picture/`) and run one of the following commands in the table to obtain the pedestrian tracking model used in the application.

	cd $HOME/samples/python/contrib/body_pose_picture/

| **Model**  |  **How to Obtain** |
| ---------- |  ----------------- |
| OpenPose_light.om | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/body_pose_picture/OpenPose_light.om' -O model/OpenPose_light.om`  |
| OpenPose_light.pb | `wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/body_pose_picture/OpenPose_light.pb' -O model/OpenPose_light.pb`  |

![Icon-note.gif](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif) **NOTE**
>
>- `OpenPose_light.om` offline model you can use out-of-the-box without model conversion. If you use this then you can skip the next step on model conversion.
>- `OpenPose_light.pb` tensorflow pb model for those that want to configure the model conversion process.


### 3. **(Optional)** Convert the original model to a Da Vinci model.

**Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](../../../environment).**

1. Set the ***LD_LIBRARY_PATH*** environment variable.

    The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when Ascend Tensor Compiler (ATC) is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.

    **export LD_LIBRARY_PATH=\\${install_path}/compiler/lib64**  

2. Run following ATC command to convert the `.pb` to `.om` model in the project directory:

    ```
    atc --framework=3 --model=model/OpenPose_light.pb --input_shape="input001:1,368,368,3" --input_format=NHWC --output=model/OpenPose_light --output_type=FP32 --out_nodes="light_openpose/stage_1/ArgMax:0" --soc_version=Ascend310
    ```

## Sample Running

> Codes in this repo create a sample to quickly understand how the model works, preprocessing, inference, postprocessing are already included. This application can be run on various input formats, namely image input, video input as well as live camera input.
>
> **Dependencies**: opencv, numpy, Pillow
>
> **Preprocessing**
>
> - **Image Resize**: 368*368
> - **Image Type**: Float32, BGR
> - **Normalization**: 1/255
>
> **Postprocessing**:
> - figure out the body keypoint coordinates from heatmaps

### Download Sample Inputs
```
cd $HOME/samples/python/contrib/body_pose_picture/
wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/body_pose_picture/test.jpg' -O data/test.jpg
wget -nc --no-check-certificate 'https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/body_pose_picture/yoga.mp4' -O data/yoga.mp4
```

### Test single image

```
cd $HOME/samples/python/contrib/body_pose_picture/src
python3 run_image.py
```
See result in `output/test_output.jpg`

### Test a sample video (mp4)
  
```
cd $HOME/samples/python/contrib/body_pose_picture/src
python3 run_video.py
```
See result in `output/demo-yoga.mp4-*****.mp4`


### Test live-camera input

1. Connect Atlas 200DK board with Raspberry Pi Camera, see [Guide](https://www.hiascend.com/document/detail/en/Atlas200DKDeveloperKit/1013/environment/atlased_04_0006.html)

    - Test: `cv2.VideoCapture("CAMERA0")` where `CAMERA0` can be found on the boardl and `0` is the camera_id
    - Make sure id match in `src/run_live.py::56` e.g. `cap = Camera(camera_id=0, fps=10)`

2. Start Presenter Server

    Execute the following command to start the Presenter Server in the background.
    ```
    cd $HOME/samples/common/
    bash run_presenter_server.sh ../python/contrib/body_pose_picture/src/body_pose.conf
    ```

3. Run Live Application:
    ``` 
    cd $HOME/samples/python/contrib/body_pose_picture/src
    python3 run_live.py
    ```

4. Visualize at `127.0.0.1:7007`

## Model Training and Description

This model is based on [Lightweight OpenPose](https://arxiv.org/pdf/1811.12004.pdf), you can refer to their [guide](https://github.com/Daniil-Osokin/lightweight-human-pose-estimation.pytorch) for training

### Description

The original model is the Lightweight OpenPose model, please refer to [link](https://github.com/Daniil-Osokin/lightweight-human-pose-estimation.pytorch), an open-source pose detection network. 

License: Apache-2.0 License

In this repository, the model is retrained wiht Tensorflow implementation and simplied for edge computing, and it directly outputs the predicted locations of the human body joints. The set of 14 detected joints are shown in the diagram below:

                     12                     0-right shoulder, 1-right elbow, 2-right wrist, 3-left shoulder
                     |                      4-left elbow, 5-left wrist, 6-right hip, 7-right knee, 8-right ankle
                     |                      9-left hip, 10-left knee, 11-left ankle, 12-top of the head, 13-neck
               0-----13-----3
              /     / \      \
             1     /   \      4
            /     /     \      \
           2     6       9      5
                 |       |
                 7       10
                 |       |
                 8       11
