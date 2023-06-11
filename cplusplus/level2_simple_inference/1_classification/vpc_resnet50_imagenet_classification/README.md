# Image Classification with Caffe ResNet-50 \(Image Decoding+Resizing+Synchronous Inference\)<a name="EN-US_TOPIC_0302603667"></a>

## Overview<a name="section7940919203810"></a>

This sample shows how to classify images based on the Caffe ResNet-50 network \(single input with batch size = 1\).

Specifically,

1.  Decodes two .jpg images into the YUV420SP format and resizes them to 224 x 224.
2.  Loads the .om offline model to perform inference on the two images, processes the obtained inference results, and outputs the class index with the top confidence value of each image.

    Convert the Caffe ResNet-50 model file into an offline model that adapts to the Ascend AI Processor in advance. During model conversion, you need to set color space conversion \(CSC\) parameters to convert YUV420SP images into RGB images to meet the input requirements of the model.



## Directory Structure<a name="section1394162513386"></a>

The sample directory is organized as follows:

```
├── caffe_model
│   ├── aipp.cfg        //Configuration file with CSC parameters, used for model conversion

├── data
│   ├── dog1_1024_683.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.
│   ├── dog2_1024_683.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.

├── inc
│   ├── dvpp_process.h               //Header file that declares functions related to data preprocessing
│   ├── model_process.h              //Header file that declares functions related to model processing
│   ├── sample_process.h               //Header file that declares functions related to resource initialization and destruction
│   ├── singleOp_process.h              //Header file that declares the functions related to single-operator execution
│   ├── utils.h                       //Header file that declares common functions (such as file reading function)

├── out
│   ├── op_models
│   ├──├──op_list.json              //Description information of operators ArgMaxV2 and Cast

├── src
│   ├── acl.json         //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── dvpp_process.cpp       //Implementation file of functions related to video processing
│   ├── main.cpp               //Main function, which is the implementation file of image classification
│   ├── model_process.cpp      //Implementation file of model processing functions
│   ├── sample_process.cpp     //Implementation file of functions related to resource initialization and destruction
│   ├── singleOp_process.cpp   //Implementation file of functions related to single-operator execution
│   ├── utils.cpp              //Implementation file of common functions (such as the file reading function)

├── .project     //Project information file, including the project type, project description, and type of the target device
├── CMakeLists.txt    //Build script that calls the CMakeLists file in the src directory
```

## Environment Requirements<a name="section3833348101215"></a>

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, Ubuntu 18.04 aarch64,  EulerOS x86, EulerOS AArch64
-   Compiler: g++ or aarch64-linux-gnu-g++
-   SoC: Ascend 310 AI Processor, Ascend 310P AI Processor, Ascend 910 AI Processor
-   Python version and dependency library: Python 3.7.5
-   The Ascend AI software stack has been deployed on the environment and the corresponding environment variables have been configured. Please refer to the corresponding version of the CANN installation guide in [Link](https://www.hiascend.com/document).
    
     In the following steps, the development environment refers to the environment for compiling and developing code, and the operating environment refers to the environment for running programs such as operators, inference, or training. 

     The operating environment must have an Ascend AI processor. The development environment and the running environment can be co-located on the same server, or they can be set up separately. In separate scenarios, the executable files compiled in the development environment are executed in the running environment. If the operating system architecture of the development environment and the running environment is Different, you need to perform cross-compilation in the development environment.

## Prepare models and pictures <a name="section1270920211277"></a>

1.  To configure CANN basic environment variables and Python environment variables, see [Link](../../../environment/environment_variable_configuration.md).

2.  Log in to the  development environment  as the running user.

3.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification" sample directory.
     
     Please note that the sample directories below refer to the "cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification" directory.

4.  Prepare the ResNet-50 models.

    1.  Get the ResNet-50 original models.

        Download the .prototxt model file and .caffemodel pre-trained model file of the ResNet-50 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

         - Model file (\*.prototxt) of ResNet-50 network: click [Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.prototxt) to download the file.
         - Weight file for ResNet-50 network (\*.caffemodel): Click [Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.caffemodel) to download the file.

    2.  Convert the ResNet-50 network into an offline model \(.om file\) that adapts to Ascend AI Processors. During model conversion, you need to set CSC parameters to convert YUV420SP images to RGB images.

        Go to the sample directory and run the following command (take Ascend310 as an example):

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --soc_version=Ascend310 --insert_op_conf=caffe_model/aipp.cfg --output=model/resnet50_aipp 
        ```

        -   **--model**: directory of the source model file.
        -   **--weight**: directory of the weight file.
        -   **--framework**: source framework type, selected from  **0**  \(Caffe\),  **1**  \(MindSpore\),  **3**  \(TensorFlow\), and  **5**  \(ONNX\).
        -   **--soc\_version**: Version of the Ascend AI processor. Go to the CANN software installation directory/compiler/data/platform_config directory. The name of the .ini file is the version of the Ascend AI processor. Select the version as required.  
        -   **--insert\_op\_conf**: path of the configuration file for inserting the AI Pre-Processing \(AIPP\) operator for AI Core–based image preprocessing including image resizing, CSC, and mean subtraction and factor multiplication \(for pixel changing\), prior to model inference.
        -   **--output**: directory for storing the generated  **resnet50\_aipp.om**  file, that is,  **/model**  under the sample directory. The default path in the command example is recommended. To specify another path, you need to change the value of  **omModelPath**  in  **sample\_process.cpp**  before building the code.

            ```
            const char* omModelPath = "../model/resnet50_aipp.om";
            ```

5.  Prepare single operator model file.

    Build the operator description information \(.json files\) of the Cast and ArgMaxV2 operators into offline models \(.om files\) that adapt to the Ascend AI Processor for running the operators.

    Go to the sample directory and run the following command:

    ```
    atc --singleop=out/op_models/op_list.json --soc_version=${soc_version} --output=out/op_models
    ```

    -   **--singleop**: directory of the single-operator definition file \(.json\)
    -   **--soc\_version**: Version of the Ascend AI processor. Go to the CANN software installation directory/compiler/data/platform_config directory. The name of the .ini file is the version of the Ascend AI processor. Select the version as required.  
    -   **--output**: directory for storing the generated .om file, that is, the  **out/op\_models**  directory.

6.  Prepare test input images.

    Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1\_1024\_683.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg)

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2\_1024\_683.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg)


## Build and Run <a name="section13133171616172"></a>

1.  To configure CANN basic environment variables and Python environment variables, see [Link](../../../environment/environment_variable_configuration.md).

2.  Build the code.
    1.  Log in to the  development environment  as the running user.

    2.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification" sample directory.
     
         Please note that the sample directories below refer to the "cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification" directory.

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
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_dvpp\_resnet50**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_dvpp\_resnet50/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_dvpp\_resnet50/out**\) and run the executable file.

        ```
        ./main
        ```

        After the command is executed successfully, the class indexes with the top confidence values are displayed.

        ```
        [INFO] acl init success
        [INFO] open device 0 success
        [INFO] create context success
        [INFO] create stream success
        [INFO] dvpp init resource success
        [INFO] load model ../model/resnet50_aipp.om success
        [INFO] create model description success
        [INFO] create model output success
        [INFO] start to process picture:../data/dog1_1024_683.jpg
        [INFO] Process dvpp success
        [INFO] model execute success
        [INFO] execute singleOp Cast success
        [INFO] execute ArgMaxV2 success
        [INFO] singleOp process success
        [INFO] ---> index of classification result is 161
        [INFO] start to process picture:../data/dog2_1024_683.jpg
        [INFO] Process dvpp success
        [INFO] model execute success
        [INFO] execute singleOp Cast success
        [INFO] execute ArgMaxV2 success
        [INFO] singleOp process success
        [INFO] ---> index of classification result is 267 
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


## Key Interfaces<a name="section6271153719394"></a>

The following lists the key functions and key interfaces involved in this sample.

-   Initialization
    -   **aclInit**: initializes AscendCL.
    -   **aclFinalize**: deinitializes AscendCL.

-   Device management
    -   **aclrtSetDevice**: sets the compute device.
    -   **aclrtGetRunMode**: obtains the run mode of the  Ascend AI Software Stack. The internal processing varies with the run mode.
    -   **aclrtResetDevice**: resets the compute device and cleans up all resources associated with the device.

-   **Context management**
    -   **aclrtCreateContext**: creates a context.
    -   **aclrtDestroyContext**: destroys a context.

-   Stream management
    -   **aclrtCreateStream**: creates a stream.
    -   **aclrtDestroyStream**: destroys a stream.
    -   **aclrtSynchronizeStream**: waits for stream tasks to complete.

-   Memory management
    -   **aclrtMallocHost**: allocates host memory.
    -   **aclrtFreeHost**: frees host memory.
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.
    -   In data preprocessing, if you need to allocate device memory to store the input or output data, call  **acldvppMalloc**  to allocate memory and call  **acldvppFree**  to free memory.

-   Data transfer

    **aclrtMemcpy**: copies memory.

-   **Data preprocessing**
    -   **acldvppJpegDecodeAsync**: decodes .jpg images into YUV420SP images.
    -   **acldvppVpcResizeAsync**: resizes YUV420SP images to 224 x 224.

-   Model inference
    -   **aclmdlLoadFromFileWithMem**: loads a model from an .om file.
    -   **aclmdlExecute**: performs model inference.

        Before inference, use the CSC parameters in the .om file to convert a YUV420SP image into an RGB image.

    -   **aclmdlUnload**: unloads a model.

-   Data postprocessing \(single-operator execution\)

    **Cast**: casts the data type of the inference result from float32 to float16;  **ArgMaxV2**: searches for the class indexes with the top confidence values in the inference result.

    **aclopExecuteV2**: loads and executes an operator.

