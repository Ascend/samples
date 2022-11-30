# Image Classification with Caffe ResNet-50 \(Synchronous Inference\)<a name="EN-US_TOPIC_0302603648"></a>

## Overview<a name="section340311417417"></a>

This sample shows how to classify images based on the Caffe ResNet-50 network \(single input with batch size = 1\).

Specifically,

1.  Converts two .jpg images to the binary format by calling the  **transferPic.py**  script provided in the sample and resizes the images from 1024 x 683 to 224 x 224.
2.  Loads the .om offline model to perform synchronous inference on the two images, processes the obtained inference results, and outputs the class index with the top 5 confidence value of each image.

    Convert the Caffe ResNet-50 model file into an offline model that adapts to the Ascend AI Processor in advance.



## Directory Structure<a name="section14723181815424"></a>

The sample directory is organized as follows:

```
├── data
│   ├── dog1_1024_683.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.
│   ├── dog2_1024_683.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.

├── inc
│   ├── model_process.h              //Header file that declares functions related to model processing
│   ├── sample_process.h              //Header file that declares functions related to resource initialization and destruction
│   ├── utils.h                       //Header file that declares common functions (such as file reading function)

├── script
│   ├── transferPic.py               //Convert a .jpg image to a .bin file and resize the image from 1024 x 683 to 224 x 224.

├── src
│   ├── acl.json         //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── main.cpp               //Main function, which is the implementation file of image classification
│   ├── model_process.cpp      //Implementation file of model processing functions
│   ├── sample_process.cpp     //Implementation file of functions related to resource initialization and destruction
│   ├── utils.cpp              //Implementation file of common functions (such as the file reading function)

├── .project     //Project information file, including the project type, project description, and type of the target device
├── CMakeLists.txt    //Build script that calls the CMakeLists file in the src directory
```

## Environment Requirements<a name="section3833348101215"></a>

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, Ubuntu 18.04 aarch64,  EulerOS x86, EulerOS AArch64
-   Compiler: g++ or aarch64-linux-gnu-g++
-   SoC: Ascend 310 AI Processor, Ascend 310P AI Processor, Ascend 910 AI Processor
-   Python and its dependency library: Python 3.7.5 and Pillow
-   The Ascend AI software stack has been deployed on the environment and the corresponding environment variables have been configured. Please refer to the corresponding version of the CANN installation guide in [Link](https://www.hiascend.com/document).
    
     In the following steps, the development environment refers to the environment for compiling and developing code, and the operating environment refers to the environment for running programs such as operators, inference, or training. 

     The operating environment must have an Ascend AI processor. The development environment and the running environment can be co-located on the same server, or they can be set up separately. In separate scenarios, the executable files compiled in the development environment are executed in the running environment. If the operating system architecture of the development environment and the running environment is Different, you need to perform cross-compilation in the development environment.


## Prepare models and pictures <a name="section1593012514400">
1.  To configure CANN basic environment variables and Python environment variables, see [Link](../../../environment/environment_variable_configuration.md).

2.  Log in to the  development environment  as the running user.

3.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification" sample directory.
     
     Please note that the sample directories below refer to the "cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification" directory.

4. Prepare the ResNet-50 models.

    1.  Get the ResNet-50 original models.

        Download the .prototxt model file and .caffemodel pre-trained model file of the ResNet-50 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

         - Model file (\*.prototxt) of ResNet-50 network: click [Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.prototxt) to download the file.
         - Weight file for ResNet-50 network (\*.caffemodel): Click [Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.caffemodel) to download the file.

    2.  Convert the ResNet-50 network to an .om  offline model adapted to Ascend AI ProcessorHiSilicon SoC.

        Go to the sample directory and run the following command (take Ascend310 as an example):

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --output=model/resnet50 --soc_version=Ascend310 --input_format=NCHW --input_fp16_nodes=data -output_type=FP32 --out_nodes=prob:0
        ```

        -   **--model**: directory of the source model file.
        -   **--weight**: directory of the weight file.
        -   **--framework**: source framework type, selected from  **0**  \(Caffe\),  **1**  \(MindSpore\),  **3**  \(TensorFlow\), and  **5**  \(ONNX\).
        -   **--soc\_version**: Version of the Ascend AI processor. Go to the CANN software installation directory/compiler/data/platform_config directory. The name of the .ini file is the version of the Ascend AI processor. Select the version as required.
        -   **--input\_format**: input format.
        -   **--input\_fp16\_nodes**: input nodes to specify as FP16 nodes.
        -   **--output\_type**  and  **--out\_nodes**: specify the data type of the first output as float32.
        -   **--output**: directory for storing the generated  **resnet50.om**  file, that is,  **/model**  under the sample directory. The default path in the command example is recommended. To specify another path, you need to change the value of  **omModelPath**  in  **sample\_process.cpp**  before building the code.

            ```
            const char* omModelPath = "../model/resnet50.om";
            ```

5.  Prepare test input images.
    1.  Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1\_1024\_683.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg)

        [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2\_1024\_683.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg)

    2.  Go to the  **/data**  directory under the sample directory, run the  **transferPic.py**  script to convert the two .jpg images into .bin files, and resize the image from 1024 x 683 to 224 x 224. Find the generated .bin files in  **/data**  under the sample directory.

        ```
        python3.7.5 ../script/transferPic.py
        ```

        If the error message "ModuleNotFoundError: No module named'PIL'" is displayed during script execution, the Pillow library does not exist. In this case, run the  **pip3.7.5 install Pillow --user**  command to install the Pillow library.



## Build and Run <a name="section1593012514493"></a>

1.  To configure CANN basic environment variables and Python environment variables, see [Link](../../../environment/environment_variable_configuration.md).

2.  Build the code.

    1.  Log in to the  development environment  as the running user.

    2.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification" sample directory.
     
        Please note that the sample directories below refer to the "cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification" directory.

    3.  Set environment variables and configure the paths of header files and library files that the program depends on for compilation.
  
        After the following environment variables are set, the compilation script will look for the compiled-dependent header files according to the "{DDK_PATH} environment variable value/runtime/include/acl" directory, and the compiled-independent library files according to the directory pointed to by the {NPU_HOST_LIB} environment variable. Replace "$HOME/Ascend" with the actual installation path of the "Ascend-cann-toolkit" package.

       *Note**, when configuring the {NPU_HOST_LIB} environment variable, you need to use the *.so library in the "runtime/lib64/stub" directory to ensure that when compiling an application based on the AscendCL interface, it does not depend on other components (such as Driver ) *.so library, after the compilation is successful, when running the application, the system will search for the *.so library in the "Ascend-cann-toolkit installation directory/runtime/lib64" directory according to the LD_LIBRARY_PATH environment variable, and will automatically link to all *.so libraries of other components that depend on them.
  
        - When the operating system architecture of the development environment and the operating environment are the same, the configuration example is as follows:
  
           ````
           export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
           export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
           ````
  
        - When the OS architecture of the development environment and the runtime environment are different, the configuration example is as follows:
           
           For example, when the development environment is the X86 architecture and the running environment is the AArch64 architecture, cross-compilation is involved, and the AArch64 architecture software package needs to be installed on the development environment, and the path of the {DDK_PATH} environment variable points to the AArch64 architecture software package installation directory ( shown below), which facilitates compiling code using header and library files from packages with the same architecture as the runtime environment.
  
           ````
           export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
           export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
           ````
       
          You can log in to the corresponding environment and run the "uname -a" command to query the architecture of its operating system.

    4.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/host**.

        ```
        mkdir -p build/intermediates/host
        ```

    5.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

        Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.

        Set **DCMAKE_SKIP_RPATH** to  **TRUE**,  **rpath**  (path specified by  **NPU_HOST_LIB**) is not added to the executable generated after build. The executable automatically looks up for dynamic libraries in the path  included in  **LD_LIBRARY_PATH**.

        -   If the operating system architecture of the  development environment is the same as that of the running environment, run the following commands to perform compilation.

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
            ```

        -   If the operating system architecture of the  development environment is different from that of the running environment, run the following commands to perform cross compilation.

            For example, if the development environment is x86 and the running environment is AArch64, run the following command:
            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
            ```


    6.  Run the  **make **command. The  **main**  executable file is generated in  **/out**  under the sample directory.

        ```
        make
        ```

3.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_resnet50**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_resnet50/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_resnet50/out**\) and run the executable file.

        ```
        ./main
        ```

        The following messages indicate that the file is successfully executed. In the displayed information, index indicates the category ID, value indicates the maximum confidence level of the category. These values may vary according to the version and environment.

        ```
        [INFO] acl init success
        [INFO] open device 0 success
        [INFO] create context success
        [INFO] create stream success
        [INFO] load model ../model/resnet50.om success
        [INFO] create model description success
        [INFO] create model output success
        [INFO] start to process file:../data/dog1_1024_683.bin
        [INFO] model execute success
        [INFO] top 1: index[161] value[xxxxxx]
        [INFO] top 2: index[xxx] value[xxxxxx]
        [INFO] top 3: index[xxx] value[xxxxxx]
        [INFO] top 4: index[xxx] value[xxxxxx]
        [INFO] top 5: index[xxx] value[xxxxxx]
        [INFO] output data success
        [INFO] start to process file:../data/dog2_1024_683.bin
        [INFO] model execute success
        [INFO] top 1: index[267] value[xxxxxx]
        [INFO] top 2: index[xxx] value[xxxxxx]
        [INFO] top 3: index[xxx] value[xxxxxx]
        [INFO] top 4: index[xxx] value[xxxxxx]
        [INFO] top 5: index[xxx] value[xxxxxx]
        [INFO] output data success
        [INFO] Unload model success, modelId is 1
        [INFO] execute sample success
        [INFO] end to destroy stream 
        [INFO] end to destroy context
        [INFO] end to reset device is 0
        ```

        >**Description:**
         >The correspondence between category labels and categories is related to the dataset used for training the model. The model used in this example is trained based on the imagenet dataset. You can check the correspondence between the labels and categories of the imagenet dataset on the Internet. For example, click [Link](https://blog.csdn.net/weixin_44676081/article/details/106755135) to view.
         >The corresponding relationship between the category ID and category in the current on-screen information is as follows:
         >"161": \["basset", "basset hound"\],
         >"267": \["standard poodle"\].



## Key Interfaces<a name="section3558105154116"></a>

This sample involves the following key functions and key interfaces:

-   Initialization
    -   **aclInit**: initializes AscendCL.
    -   **aclFinalize**: deinitializes AscendCL.

-   Device management
    -   **aclrtSetDevice**: sets the compute device.
    -   **aclrtGetRunMode**: obtains the run mode of the  Ascend AI Software Stack. The internal processing varies with the run mode.
    -   **aclrtResetDevice**: resets the compute device and cleans up all resources associated with the device.

-   Context management
    -   **aclrtCreateContext**: creates a context.
    -   **aclrtDestroyContext**: destroys a context.

-   Stream management
    -   **aclrtCreateStream**: creates a stream.
    -   **aclrtDestroyStream**: destroys a stream.

-   Memory management
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.

-   Data transfer

    **aclrtMemcpy**: copies memory.

-   Model inference
    -   **aclmdlLoadFromFileWithMem**: loads a model from an .om file.
    -   **aclmdlExecute**: performs synchronous model inference.
    -   **aclmdlUnload**: unloads a model.

-   Data postprocessing

    Provides sample code to process the model inference result and display the class indexes with top 5 confidence values of each image.

    The sample provides a user-defined API  **DumpModelOutputResult**, which is used to write the model inference result to a file \(after the executable file is executed, the inference result file is generated in the directory of the executable file in the  operating environment\). This API is not called by default. To call this API, you need to add the following code before the  **OutputModelResult**  call in  **sample\_process.cpp**  in advance.

    ```
    // Print the top 5 confidence values with indexes using DumpModelOutputResult
    // if want to dump output result to file in the current directory
    modelProcess.DumpModelOutputResult();
    modelProcess.OutputModelResult();
    ```
