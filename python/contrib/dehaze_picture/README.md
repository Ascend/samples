English|[中文](README_CN.md)

**This sample provides reference for you to learn the Ascend AI Software Stack and cannot be used for commercial purposes.**

**This sample works with CANN 3.0.0 and later versions, and supports Atlas 200 DK and Atlas 300.**

**This readme file provides only guidance for running the sample in the command line. For details about how to run the sample in MindStudio, see [Running Image Samples in MindStudio](https://github.com/Ascend/samples/wikis/Running%20Image%20Samples%20in%20MindStudio?sort_id=3736297).**


## SingleImageDehaze Sample

Function: dehazes an input image by using the deploy_vel model.

Input: a JPG image

Output: a result image

### Prerequisites

Before deploying this sample, ensure that:

- The environment has been set up by referring to [Environment Preparation and Dependency Installation](https://github.com/Ascend/samples/blob/master/python/environment/README.md).
- The development environment and operating environment of the corresponding product have been set up.

### Software Preparation

#### 1. Obtain the source package.

  You can download the source code in either of the following ways:

   - Command line (The download takes a long time, but the procedure is simple.)

     In the development environment, run the following commands as a non-root user to download the source repository:
        ```
     cd $HOME
     git clone https://github.com/Ascend/samples.git
        ```
   - Compressed package (The download takes a short time, but the procedure is complex.)

     1. Click **Clone or download** in the upper right corner of the samples repository and click **Download ZIP**.

     2. Upload the .zip package to the home directory of a common user in the development environment, for example, **$HOME/ascend-samples-master.zip**.

     3. In the development environment, run the following commands to unzip the package:

      ```
     cd $HOME
     unzip ascend-samples-master.zip
      ```
#### 2. Obtain the model required by the application.

   Obtain the model used in the application by referring to the following table and save it to the project directory of a common user in the development environment.

	cd $HOME/samples/python/contrib/SingleImageDehaze/model

| **Model Name** | **Description**                  | **How to Obtain**                        |
| -------------- | -------------------------------- | ---------------------------------------- |
| deploy_vel     | Dehaze model based on TensorFlow | Download the model file and the corresponding CFG file by referring to "[Original Model](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/dehaze/ATC_deploy_vel_tf_AE)". |


#### 3. Convert the original model to a Da Vinci model.

   **Note: Ensure that the environment variables have been configured in [Environment Preparation and Dependency Installation](https://github.com/Ascend/samples/tree/master/python/environment).**

   1. Set the ***LD_LIBRARY_PATH*** environment variable.

      The ***LD_LIBRARY_PATH*** environment variable conflicts with the sample when Ascend Tensor Compiler (ATC) is used. Therefore, you need to set this environment variable separately in the command line to facilitate modification.

         ```	
      export LD_LIBRARY_PATH=${install_path}/atc/lib64
         ```

   2. Run the following command to convert the model:     
     ```
     atc --model=output_graph.pb --framework=3 --input_shape="t_image_input_to_DHGAN_generator:1,512,512,3" --output=deploy_vel --soc_version=Ascend310 --input_fp16_nodes="t_image_input_to_DHGAN_generator" --output_type= FP32  
     ```

#### 4. Obtain the test image required by the sample.

Run the following commands to go to the **data** folder of the sample and download the corresponding test image:

    cd $HOME/samples/python/contrib/SingleImageDehaze/data
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/SingleImageDehaze/test_image/10992_04_0.8209.png


### Sample Running

**Note: If the development environment and operating environment are set up on the same server, skip step 1 and go to step 2 directly.**

1. Run the following commands to upload the **SingleImageDehaze** directory in the development environment to any directory in the operating environment, for example, **/home/HwHiAiUser**, and log in to the operating environment (host) as the running user (**HwHiAiUser**):
      ```
         scp -r $HOME/samples/python/contrib/SingleImageDehaze/  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
         scp -r $HOME/samples/python/common/atlas_utils/   HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
         ssh HwHiAiUser@xxx.xxx.xxx.xxx
      ```

   ![Icon-note.gif](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif) **NOTE**

   > - Replace ***xxx.xxx.xxx.xxx*** with the IP address of the operating environment. The IP address of Atlas 200 DK is **192.168.1.2** when it is connected over the USB port, and that of Atlas 300 is the corresponding public network IP address.


2. Run the executable file.

   - If the development environment and operating environment are set up on the same server, run the following commands to set the operating environment variable and switch the directory:

     ```
     export LD_LIBRARY_PATH=
     source ~/.bashrc
     cd $HOME/samples/python/contrib/SingleImageDehaze/src
     python3 main.py ../data/
     ```

   - If the development environment and operating environment are set up on separate servers, run the following command to switch the directory:

     ```
     cd $HOME/python/contrib/SingleImageDehaze/src  
     ```
  Run the following command to run the sample:

```
python3.6 main.py ../data/
```


### Result Checking

After the execution is complete, the inference result is displayed on the CLI of the operating environment.