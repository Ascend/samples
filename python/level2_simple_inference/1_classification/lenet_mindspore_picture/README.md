English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample applies to CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)).**

**This readme file provides only guidance for running the sample in command line (CLI) mode.**

## Garbage Sorting Sample

Function: classifies input images by using the lenet model.

Input: JPG images to be inferred

Output: the number of JPG images after inference

For details about the training, see [mnist Sorting with lenet](https://github.com/Ascend/modelzoo/tree/master/built-in/MindSpore/Official/cv/image_classification/LeNet_for_MindSpore).
In Ascend910 ， use scripts/convert.py convert checkpoint_lenet-1_1875.ckpt to mnist.air


### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](https://github.com/Ascend/samples/tree/master/python/environment).

- The development environment and operating environment of the corresponding product have been installed.

### Software Preparation

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

2. Obtain the source model required by the application.

    Obtain the original model and its weight files used in the application by referring to the following table and save them to any directory of a common user in the development environment, for example, **$HOME/models/garbage_picture**.

    | **Model Name** | **Description**                          | **How to Obtain**                        |
    | -------------- | ---------------------------------------- | ---------------------------------------- |
    | mnist    | Image classification inference model. It is a lenet model based on MindSpore. | https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air |



3. Convert the original model to a Da Vinci model.

    **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](https://github.com/Ascend/samples/tree/dev/python/environment).**

    1. Set the ***LD_LIBRARY_PATH*** environment variable.

        The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when the ATC tool is used. Therefore, you need to set this environment variable separately in the CLI to facilitate modification.

        **export install_path=$HOME/Ascend/ascend-toolkit/latest**

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. Run the following commands to  convert the model:

        **cd $HOME/models/lenet_mindspore**

        **atc --framework=1 --model=mnist.air  --output=mnist --soc_version=Ascend310**

    3. Run the following command to copy the converted model to the **model** folder of the sample:

        **cp ./mnist.om $HOME/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/model/**

4. Obtain the test images required by the sample.

    Run the following commands to go to the **data** folder of the sample and download the corresponding test images:

    **cd $HOME/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/data**

    **https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test1.png**

    **https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test2.png** 
    
    **https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test3.png**      



### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to [step 2](#step_2) directly.**   

1. Run the following commands to upload the **samples** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the **HwHiAiUser** user:

    **scp -r $HOME/samples/  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **NOTE**  

    > - In the following information, ***xxx.xxx.xxx.xxx*** is the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 (AI1s) is the corresponding public IP address.

2. Run the executable file.

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**

      **cd $HOME/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/**     

    Run the following command to run the sample:

    **python3.6 src/classify.py ./data/**
### Result Checking

After the execution is complete, it will display:

```
init resource stage:
Init resource success
Init model resource start...
[Model] create model output dataset:
malloc output 0, size 40
Create model output dataset success
Init model resource success
(32, 32)
post process
images:test2.png
======== top5 inference results: =============
label:9  confidence: 0.991472, class: 9
label:7  confidence: 0.003693, class: 7
label:8  confidence: 0.001775, class: 8
label:3  confidence: 0.001515, class: 3
label:4  confidence: 0.000880, class: 4
(32, 32)
post process
images:test3.png
======== top5 inference results: =============
label:7  confidence: 0.958997, class: 7
label:9  confidence: 0.022686, class: 9
label:8  confidence: 0.006465, class: 8
label:3  confidence: 0.005904, class: 3
label:1  confidence: 0.002834, class: 1
(32, 32)
post process
images:test1.png
======== top5 inference results: =============
label:1  confidence: 0.993403, class: 1
label:9  confidence: 0.001830, class: 9
label:8  confidence: 0.001219, class: 8
label:4  confidence: 0.001122, class: 4
label:7  confidence: 0.000977, class: 7
acl resource release all resource
Model release source success
acl resource release stream
acl resource release context
Reset acl device  0
Release acl resource success
run success
```

