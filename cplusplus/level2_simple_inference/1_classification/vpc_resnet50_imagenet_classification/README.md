# Image Classification with Caffe ResNet-50 \(Image Decoding+Resizing+Synchronous Inference\)<a name="EN-US_TOPIC_0302603667"></a>

## Overview<a name="section7940919203810"></a>

This sample shows how to classify images based on the Caffe ResNet-50 network \(single input with batch size = 1\).

Specifically,

1.  Decodes two .jpg images into the YUV420SP format and resizes them to 224 x 224.
2.  Loads the .om offline model to perform inference on the two images, processes the obtained inference results, and outputs the class index with the top confidence value of each image.

    Convert the Caffe ResNet-50 model file into an offline model that adapts to the Ascend AI Processor in advance. During model conversion, you need to set color space conversion \(CSC\) parameters to convert YUV420SP images into RGB images to meet the input requirements of the model.


## Principles<a name="section6271153719394"></a>

The following lists the key functions involved in this sample.

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

    **Cast**: casts the data type of the inference result from float32 to float16;  **ArgMaxD**: searches for the class indexes with the top confidence values in the inference result.

    **aclopExecuteV2**: loads and executes an operator.


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
│   ├──├──op_list.json              //Description information of operators ArgMaxD and Cast

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
-   Ascend AI Software Stack deployed

## Environment Variables<a name="section1270920211277"></a>

- Configuring Environment Variables in the Development Environment

  1. The CANN portfolio provides a process-level environment variable setting script to automatically set environment variables. The following commands are used as examples

     ```
     . ${HOME}/Ascend/ascend-toolkit/set_env.sh
     ```

     Replace  **$HOME/Ascend**  with the actual Ascend-CANN-Toolkit installation path.

  2. Operator building requires Python installation. The following takes Python 3.7.5 as an example. Run the following commands as a running user to set the environment variables related to Python 3.7.5.

     ```
     # Set tje Python3.7.5 library path.
     export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
     # If multiple Python 3 versions exist in the user environment, specify Python 3.7.5.
     export PATH=/usr/local/python3.7.5/bin:$PATH
     ```

     Replace the Python 3.7.5 installation path as required. You can also write the preceding commands to the ~/.bashrc file and run the source ~/.bashrc command to make the modification take effect immediately.

  3. In the development environment, set environment variables and configure the header search path and library search path on which the build of the AscendCL single-operator verification program depends.

     The build script searches for the required header files and libraries through the paths specified by the environment variables. Replace  **$HOME/Ascend**  with the actual Ascend-CANN-Toolkit installation path.

     - If the development environment operating system architecture is x86, the configuration example is as follows:

       ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
        export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
       ```

     - If the running environment operating system architecture is AArch64, the configuration example is as follows:

       ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
       ```

- Configuring Environment Variables in the Running Environment

  - If Ascend-CANN-Toolkit is installed in the running environment, set the environment variable as follows:

    ```
    . ${HOME}/Ascend/ascend-toolkit/set_env.sh
    ```

  - If Ascend-CANN-NNRT is installed in the running environment, set the environment variable as follows:

    ```
    . ${HOME}/Ascend/nnrt/set_env.sh
    ```

  - If Ascend-CANN-NNAE is installed in the running environment, set the environment variable as follows:

    ```
    . ${HOME}/Ascend/nnae/set_env.sh
    ```

    Replace  **$HOME/Ascend**  with the actual component installation path.


## Build and Run <a name="section13133171616172"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.

    2.  Prepare data.

        Download the .prototxt model file and .caffemodel pre-trained model file of the ResNet-50 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        Click [link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE), find the download links in the  **README.md**  file.

    3.  Convert the ResNet-50 network into an offline model \(.om file\) that adapts to Ascend AI Processors. During model conversion, you need to set CSC parameters to convert YUV420SP images to RGB images.

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


    5.  Build the operator description information \(.json files\) of the Cast and ArgMaxD operators into offline models \(.om files\) that adapt to the Ascend AI Processor for running the operators.

        Go to the sample directory and run the following command:

        ```
        atc --singleop=out/op_models/op_list.json --soc_version=${soc_version} --output=out/op_models
        ```

        -   **--singleop**: directory of the single-operator definition file \(.json\)
        -   **--soc\_version**: Version of the Ascend AI processor. Go to the CANN software installation directory/compiler/data/platform_config directory. The name of the .ini file is the version of the Ascend AI processor. Select the version as required.  
        -   **--output**: directory for storing the generated .om file, that is, the  **out/op\_models**  directory.


2.  Build the code.
    1.  Log in to the  development environment  as the running user.
    2.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/host**.

        ```
        mkdir -p build/intermediates/host
        ```

    3.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

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


    4.  Run the  **make **command. The  **main**  executable file is generated in  **/out**  under the sample directory.

        ```
        make
        ```


3.  Prepare input images.

    Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1\_1024\_683.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg)

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog2\_1024\_683.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg)

4.  Run the app.
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
        [INFO] execute ArgMaxD success
        [INFO] singleOp process success
        [INFO] ---> index of classification result is 161
        [INFO] start to process picture:../data/dog2_1024_683.jpg
        [INFO] Process dvpp success
        [INFO] model execute success
        [INFO] execute singleOp Cast success
        [INFO] execute ArgMaxD success
        [INFO] singleOp process success
        [INFO] ---> index of classification result is 267 
        [INFO] Unload model success, modelId is 1
        [INFO] execute sample success
        [INFO] end to destroy stream 
        [INFO] end to destroy context
        [INFO] end to reset device is 0
        ```


