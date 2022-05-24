English|[中文](README_CN.md)

# Object Detection with Caffe YOLOv3 (Dynamic Batch/Image Size)

## Function Description

This sample implements object detection based on the Caffe YOLOv3 network in the dynamic batch/image size scenario.

In this sample, the Caffe YOLOv3 network is converted into an OM offline model (**\*.om** file) that adapts to the Ascend AI Processor. In the conversion command, you need to set the batch size profiles (for example, **1**, **2**, **4**, and **8**) or set the image size profiles (for example, **416**, **416**, **832**, **832**, **1248**, and **1248**). In your application, load the **\*.om** file, select the batch/image size for inference by passing the corresponding argument, and output the inference result to a file.

## Principles

This sample involves the following key functions:

-   **Initialization**
    -   **aclInit**: initializes AscendCL.
    -   **aclFinalize**: deinitializes AscendCL.

-   **Device management**
    -   **aclrtSetDevice**: sets the compute device.
    -   **aclrtGetRunMode**: obtains the operating mode of the Ascend AI Software Stack. The internal processing process varies according to the mode.
    -   **aclrtResetDevice**: resets the compute device and reclaims resources on the device.

-   **Context management**
    -   **aclrtCreateContext**: creates a context.
    -   **aclrtDestroyContext**: destroys a context.

-   **Stream management**
    -   **aclrtCreateStream**: creates a stream.
    -   **aclrtDestroyStream**: destroys a stream.

-   **Memory management**
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.

-   **Data transfer**

    **aclrtMemcpy**: copies memory.

-   **Model inference**
    -   **aclmdlLoadFromFileWithMem**: loads a model from an **\*.om** file.
    -   **aclmdlSetDynamicBatchSize** or **aclmdlSetDynamicHWSize**: sets the runtime batch or image size.
    -   **aclmdlExecute**: performs synchronous model inference.
    -   **aclmdlUnload**: unloads a model.

-   **Data postprocessing**

    **DumpModelOutputResult**: writes the model inference result to a file. (After the executable file is executed, the inference result file is stored in the same directory in the operating environment as the executable file of the application.)

    ```
    processModel.DumpModelOutputResult();
    ```


## Directory Structure

The directory structure of the sample is as follows:

```
├── data
│   ├── tools_generate_data.py            //Script for generating test data

├── inc
│   ├── model_process.h              //Header file that declares functions related to model processing
│   ├── sample_process.h              //Header file that declares functions related to resource initialization and destruction                  
│   ├── utils.h                       //Header file that declares common functions (such as the file reading function)

├── src
│   ├── acl.json         //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── main.cpp               //Implementation file of the main function, for image classification
│   ├── model_process.cpp      // Implementation file of model processing functions
│   ├── sample_process.cpp     //Implementation file of functions related to resource initialization and destruction                                         
│   ├── utils.cpp              //Implementation file of common functions (such as the file reading function)

├── .project     //Project information file, including the project type, project description, and type of the target device
├── CMakeLists.txt    //Build script that calls the **CMakeLists** file in the **src** directory
```

## Environment Requirements

-   OS and architecture: CentOS 7.6 x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, EulerOS x86, EulerOS AArch64
-   Compiler: g++ or aarch64-linux-gnu-g++
-   Processor: Ascend 310, Ascend 710, or Ascend 910
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed

## Environment Variable Setting

- Set environment variables in the development environment.

  1. The CANN-Toolkit provides a process-level environment variable configuration script for users to reference in processes to automatically set basic CANN environment variables. The following is a configuration example:

     ```
     . ${HOME}/Ascend/ascend-toolkit/set_env.sh
     ```

     Replace **$HOME/Ascend** with the actual Ascend-CANN-Toolkit installation path.

  2. Operator build requires Python installation. The following takes Python 3.7.5 as an example. Run the following commands as a running user to set the environment variables related to Python 3.7.5:

     ```
     #Set the Python 3.7.5 library path.
     export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
     #If multiple Python 3 versions exist in the user environment, use Python 3.7.5.
     export PATH=/usr/local/python3.7.5/bin:$PATH
     ```

     Replace the Python 3.7.5 installation path as required. You can also write the preceding commands to the **~/.bashrc** file and run the **source ~/.bashrc** command to make the modification take effect immediately.
     
  3. In the development environment, set environment variables and configure the paths of the header files and library files required for building the AscendCL single-operator verification program.
  
     The build script searches for the required header files and libraries through the paths specified by environment variables. Replace **$HOME/Ascend** with the actual Ascend-CANN-Toolkit installation path.
  
     -   In the x86 operating environment:
  
         ```
         export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
         export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
         ```
  
     -   In the AArch64 operating environment:
  
         ```
         export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
         export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
         ```
- Set environment variables in the operating environment.
  
  -   If Ascend-CANN-Toolkit is installed in the operating environment, set the environment variable as follows:
  
      ```
      . ${HOME}/Ascend/ascend-toolkit/set_env.sh
      ```
  
  -   If Ascend-CANN-NNRT is installed in the operating environment, set the environment variable as follows:
  
      ```
      . ${HOME}/Ascend/nnrt/set_env.sh
      ```
  
  -   If Ascend-CANN-NNAE is installed in the operating environment, set the environment variable as follows:
  
      ```
      . ${HOME}/Ascend/nnae/set_env.sh
      ```
  
    Replace **$HOME/Ascend** with the actual software installation path.

## Building and Runing

1.  Convert the model.
    1.  Log in to the development environment as the running user.
    2.  Make required preparations, including obtaining the ATC tool and set environment variables. For details, see "Getting Started" in the "ATC Instructions" of the *CANN Auxiliary Development Tool User Guide*.
    3.  Prepare data.

        Download the .prototxt model file and .caffemodel pre-trained model file of the YOLOv3 network and upload the files to **/caffe\_model** under the sample directory in the development environment as the running user. If the directory does not exist, create it.

        -   Obtain the files from Gitee: Click [Link](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/yolov3/ATC_yolov3_caffe_AE), view the **README.md** file, and search for the link for obtaining the source model.
        -   Obtain the files from GitHub: Click [Link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/yolov3/ATC_yolov3_caffe_AE), view the **README.md** file, and search for the link for obtaining the source model.


    4.  Switch to the sample directory and convert the YOLOv3 network into an OM offline model (**\*.om** file) that adapts to the Ascend AI Processor.

        If the input for model inference allows a dynamic batch size, run the following command to convert the model (taking the Ascend 310 AI Processor as an example):

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:-1,3,416,416" --input_format=NCHW --dynamic_batch_size="1,2,4,8" --soc_version=Ascend310 --output=model/yolov3_dynamic_batch 
        ```

        If the input for model inference allows a dynamic image size, run the following command to convert the model (taking the Ascend 310 AI Processor as an example):

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:1,3,-1,-1" --input_format=NCHW --dynamic_image_size="416,416;832,832;1248,1248" --soc_version=Ascend310  --output=model/yolov3_dynamic_hw 
        ```

        -   --**model**: directory of the source model file.
        -   --**weight**: directory of the weight file.
        -   --**framework**: source framework type, selected from **0** (Caffe), **1** (MindSpore), **3** (TensorFlow), and **5** (ONNX).
        -   --**input\_shape**: input shape.
        -   --**input\_format**: input format.
        -   --**dynamic\_batch\_size**: dynamic batch size profiles. Applies to the scenario where image count per inference batch is unfixed.
        -   --**dynamic\_image\_size**: dynamic image size profiles. Applies to the scenario where image size per inference batch is unfixed.
        -   --**soc\_version**:
            -   For Ascend 310 AI Processors, set this parameter to **Ascend310**.
            -   For Ascend 710 AI Processors, set this parameter to **Ascend710**.
            -   For Ascend 910 AI Processors, set this parameter to **Ascend910A**, **Ascend910B**, **Ascend910ProA**, **Ascend910ProB**, or **Ascend910PremiumA** as required. "Pro" and "Premium" indicate the performance improvement levels of processors, and "A" and "B" indicate the PartialGood levels.
        -   --**output**: directory for storing the generated **yolov3\_dynamic\_batch.om** or **yolov3\_dynamic\_hw.om** file, that is, **/model** under the sample directory. The default path in the command example is recommended. To specify another path, change the value of **omModelPath** in **sample\_process.cpp** before building the code.

            ```
            string omModelPath = "../model/yolov3_dynamic_batch.om";
            ......
            string omModelPath = "../model/yolov3_dynamic_hw.om";
            ```



2.  Build the code.
    1.  Log in to the development environment as the running user.
    2.  Switch to **/data** under the sample directory and run the **tools\_generate\_data.py** script. The test .bin files corresponding to different batch sizes or image sizes are generated in **/data** under the sample directory.

        For example, run the following command to generate the **input\_float32\_1x3x416x416.bin.in** file for: batch size = 1, channel number = 3, image size = 416 x 416 pixels, and data type = float32.

        ```
        python3.7 tools_generate_data.py input -s [1,3,416,416] -r [2,3] -d float32
        ```

        Parameters in the **tools\_generate\_data.py** script are described as follows:

        -   **input**: prefix of the .bin file name.
        -   **s**: shape of the input data.
        -   **r**: pixel value range of each channel. The value range is [0, 255]. After the **tools\_generate\_data.py** script is executed, the pixel value of each channel is randomly generated within the value range specified by the **-r** argument.
        -   **d**: data format, selected from **int8**, **uint8**, **float16**, **float32**, **int32**, and **uint32**.

    3.  Go to the sample directory and create a subdirectory for storing build outputs, for example, **build/intermediates/host**.

        ```
        mkdir -p build/intermediates/host
        ```

    4.  Switch to the **build/intermediates/host** directory and run the **cmake** command.

        Replace **../../../src** with the directory of **CMakeLists.txt**.

        Set **DCMAKE\_SKIP\_RPATH** to **TRUE**, indicating that the rpath information (that is, the path configured by **NPU_HOST_LIB**) will not be added to the executable file generated after build, and the executable file automatically searches for the dynamic link library in the set **LD_LIBRARY_PATH**.

        -   If the OS architecture of the development environment is the same as that of the operating environment, run the following commands to perform compilation:

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
            ```

        -   If the development environment and operating environment have different OS architectures, run the following commands to perform cross compilation.

            For example, if the development environment uses the x86 architecture while the operating environment uses the AArch64 architecture, run the following commands to perform cross compilation:
            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
            ```


    5.  Run the **make **command. The **main** executable file is generated in **/out** under the sample directory.

        ```
        make
        ```


3.  Run your application.
    1.  As the running user, upload the sample directory in the development environment to the operating environment (host), for example, **$HOME/acl\_yolov3\_dynamic\_batch**.
    2.  Log in to the operating environment (host) as the running user.
    3.  Switch to the directory where the executable file **main** is located (for example, **$HOME/acl\_yolov3\_dynamic\_batch/out**) and grant the execute permission on the **main** file in the directory.

        ```
        chmod +x main
        ```

    4.  Switch to the directory where the executable file **main** is located, for example, **$HOME/acl\_yolov3\_dynamic\_batch/out**, and run the executable file.
        -   In the dynamic batch size scenario, run the following command. Replace the input argument with the runtime batch size, which should be among the profiles specified by the **--dynamic\_batch\_size** argument in the model conversion command.

            ```
            ./main 1
            ```

            The following messages indicate that the file is successfully executed.

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_batch.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model output success.
            [INFO]  set dynamic batch size[1] success.
            [INFO]  model input num[3], output num[2].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[16613376], fotmat[0], dataType[0]
            [INFO]  dimcount:[4],dims:[-1][3][416][416]
            [INFO]  index[1]: name[img_info], inputSize[16], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[1][4]
            [INFO]  index[2]: name[ascend_mbatch_shape_data], inputSize[4], fotmat[2], dataType[3]
            [INFO]  dimcount:[1],dims:[1]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[detection_out3:0:box_out], outputSize[196608], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[8][6144]
            [INFO]  index[1]: name[detection_out3:1:box_out_num], outputSize[256], fotmat[0], dataType[3]
            [INFO]  dimcount:[2],dims:[8][8]
            [INFO]  start to print model dynamic batch info:
            [INFO]  dynamic batch count:[4],dims:{[1][2][4][8]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][6144]
            [INFO]  index:1,dims:[1][8]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            [INFO]  end to finalize acl.
            ```


        -   In the dynamic image size scenario, run the following command. Replace the input arguments with the actual image height and width, which should be among the profiles specified by the **--dynamic\_image\_size** argument in the model conversion command.

            ```
            ./main 416 416
            ```

            The following messages indicate that the file is successfully executed.

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_hw.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model input success.
            [INFO]  create model output success.
            [INFO]  set dynamic hw[416, 416] success.
            [INFO]  model input num[3], output num[2].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[18690048], fotmat[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][3][-1][-1]
            [INFO]  index[1]: name[img_info], inputSize[16], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[1][4]
            [INFO]  index[2]: name[ascend_mbatch_shape_data], inputSize[8], fotmat[2], dataType[3]
            [INFO]  dimcount:[1],dims:[2]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[detection_out3:0:box_out], outputSize[24576], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[1][6144]
            [INFO]  index[1]: name[detection_out3:1:box_out_num], outputSize[32], fotmat[0], dataType[3]
            [INFO]  dimcount:[2],dims:[1][8]
            [INFO]  start to print model dynamic hw info:
            [INFO]  dynamic hw count:[3],dims:{[416, 416][832, 832][1248, 1248]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][6144]
            [INFO]  index:1,dims:[1][8]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  destroy model description success.
            [INFO]  destroy model input success.
            [INFO]  destroy model output success.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            ```
