# Image Classification with Caffe ResNet-50 \(Asynchronous Inference\)<a name="EN-US_TOPIC_0302603656"></a>

## Overview<a name="section340311417417"></a>

This sample shows how to classify images based on the Caffe ResNet-50 network \(single input with batch size = 1\).

Specifically,

1.  Converts two .jpg images to the binary format by calling the  **transferPic.py**  script provided in the sample and resizes the images from 1024 x 683 to 224 x 224.
2.  Loads the .om file to perform  _n_  \(an application parameter configured by the user\) times of asynchronous inference on the two images, processes the obtained  _n_  inference results, and outputs the class index with the top 1 confidence value of each image.

    Convert the Caffe ResNet-50 model file into an offline model that adapts to the Ascend AI Processor in advance.


## Principles<a name="section3558105154116"></a>

This sample involves the following key functions:

-   Initialization
    -   **aclInit**: initializes AscendCL.
    -   **aclFinalize**: deinitializes AscendCL.

-   Device management
    -   **aclrtSetDevice**: sets the compute device.
    -   **aclrtGetRunMode**: obtains the run mode of the  Ascend AI Software Stack. The internal processing varies with the run mode.
    -   **aclrtResetDevice**: resets the compute device and cleans up all resources associated with the device.

-   **Context management**
    -   **aclrtCreateContext**: creates a context.
    -   **aclrtSetCurrentContext**: binds a context to the calling thread.
    -   **aclrtDestroyContext**: destroys a context.

-   Stream management
    -   **aclrtCreateStream**: creates a stream.
    -   **aclrtDestroyStream**: destroys a stream.

-   Memory management
    -   **aclrtMallocHost**: allocates host memory.
    -   **aclrtFreeHost**: frees host memory.
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.

-   Data transfer

    **aclrtMemcpy**: copies memory.

-   Model inference
    -   **aclmdlLoadFromFileWithMem**: loads a model from an .om file.
    -   Create a thread \(for example,  **t1**\), call  **aclrtProcessReport**  in the thread function, and trigger the callback function \(for example,  **CallBackFunc**  for processing the model inference result\) after a specified time.
    -   **aclrtSubscribeReport**: specifies the thread \(**t1**\) for processing the callback function \(**CallBackFunc**\) of the stream.
    -   **aclmdlExecuteAsync**: performs asynchronous model inference.
    -   **aclrtLaunchCallback**: adds a callback function \(**CallBackFunc**\) to be executed on the host or device to the stream task queue.
    -   **aclrtSynchronizeStream**: waits for stream tasks to complete.
    -   **aclrtUnSubscribeReport**: deregisters the thread. The callback function \(**CallBackFunc**\) of the stream is not processed by the specified thread \(t1\).
    -   **aclmdlUnload**: unloads a model.

-   Data postprocessing

    Provides sample code to process the model inference result and display the class index with the top confidence value of each image.

    The sample provides a user-defined API  **DumpModelOutputResult**, which is used to write the model inference result to a file \(after the executable file is executed, the inference result file is generated in the directory of the executable file in the  operating environment\). This API is not called by default. To call this API, you need to add the following code before the  **OutputModelResult**  call in  **sample\_process.cpp**  in advance.

    ```
    // OutputModelResult prints the top 1 confidence value with index.
    // If want to dump output result to file in the current directory,
    // use function DumpModelOutputResult.
    ModelProcess::DumpModelOutputResult(data.second);
    ModelProcess::OutputModelResult(data.second);
    ```


## Directory Structure<a name="section14723181815424"></a>

The sample directory is organized as follows:

```
├── data
│   ├── dog1_1024_683.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.
│   ├── dog2_1024_683.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.

├── inc
│   ├── memory_pool.h                 //Header file that declares functions related to memory pool processing
│   ├── model_process.h              //Header file that declares functions related to model processing
│   ├── sample_process.h              //Header file that declares functions related to resource initialization and destruction
│   ├── utils.h                       //Header file that declares common functions (such as file reading function)

├── script
│   ├── transferPic.py               //Convert a .jpg image to a .bin file and resize the image from 1024 x 683 to 224 x 224.

├── src
│   ├── acl.json         //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── main.cpp               //Main function, which is the implementation file of image classification
│   ├── memory_pool.cpp        //Implementation file of functions related to memory pool processing
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
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed

## Environment Variables<a name="section2576153161110"></a>

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



## Build and Run <a name="section7572134019439"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.

    2.  Prepare data.

        Download the .prototxt model file and .caffemodel pre-trained model file of the ResNet-50 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        Click [link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE), find the download links in the  **README.md**  file.

    3.  Convert the ResNet-50 network into an offline model \(.om file\) that adapts to Ascend AI Processors.

        Go to the sample directory and run the following command (take Ascend310 as an example):

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --output=model/resnet50 --soc_version=Ascend310 --input_format=NCHW --input_fp16_nodes=data -output_type=FP32 --out_nodes=prob:0
        ```

        -   **--model**: directory of the source model file.
        -   **--weight**: directory of the weight file.
        -   **--framework**: source framework type, selected from  **0**  \(Caffe\),  **1**  \(MindSpore\),  **3**  \(TensorFlow\), and  **5**  \(ONNX\).
        -   **--soc\_version**:
            -   Ascend 310 AI Processor, set this parameter to **Ascend310**.
            -   Ascend 310P AI Processor, set this parameter to **Ascend310P**.
            -   Ascend 910 AI Processor, set this parameter to **Ascend910A** or **Ascend910B** or **Ascend910ProA** or **Ascend910ProB** or **Ascend910PremiumA**. **Pro** or **Premium** indicate the performance improvement level. **A** or **B** indicate PartialGood level. Select a value based on the site requirements.
        -   **--input\_format**: input format.
        -   **--input\_fp16\_nodes**: input nodes to specify as FP16 nodes.
        -   **--output\_type**  and  **--out\_nodes**: specify the data type of the first output as float32.
        -   **--output**: directory for storing the generated  **resnet50.om**  file, that is,  **/model**  under the sample directory. The default path in the command example is recommended. To specify another path, you need to change the value of  **omModelPath**  in  **sample\_process.cpp**  before building the code.

            ```
            const char* omModelPath = "../model/resnet50.om";
            ```



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
    1.  Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1\_1024\_683.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg)

        [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog2\_1024\_683.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg)

    2.  Log in to the  development environment  as the running user.
    3.  Go to the  **/data**  directory under the sample directory, run the  **transferPic.py**  script to convert the two .jpg images into .bin files, and resize the image from 1024 x 683 to 224 x 224. Find the generated .bin files in  **/data**  under the sample directory.

        ```
        python3.7.5 ../script/transferPic.py
        ```

        If the error message "ModuleNotFoundError: No module named'PIL'" is displayed during script execution, the Pillow library does not exist. In this case, run the  **pip3.7.5 install Pillow --user**  command to install the Pillow library.


4.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_resnet50\_async**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_resnet50\_async/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_resnet50\_async/out**\) and run the executable file.

        -   Run the executable file without parameters:
            -   By default, model asynchronous inference is performed for 100 times.
            -   By default, the callback interval is  **1**, indicating that a callback task is issued every asynchronous inference iteration.
            -   By default, there are 100 memory blocks in a memory pool.

        -   Run the executable file with parameters:
            -   The first parameter indicates the number of model asynchronous inference times.
            -   The second parameter indicates the interval for delivering the callback task.  **0**  indicates the callback task is not delivered. A non-zero value \(for example,  **_m_**\) indicates that the callback task is delivered after  _**m**_  times of asynchronous inference.
            -   The third parameter indicates the number of memory blocks in the memory pool. The argument must be greater than or equal to the number of model asynchronous inference times.


        ```
        ./main
        ```

        The following messages indicate that the file is successfully executed. In the displayed information, index indicates the category ID, value indicates the maximum confidence level of the category. These values may vary according to the version and environment.

        ```
        [INFO]  ./main param1 param2 param3, param1 is execute model times(default 100), param2 is callback interval(default 1), param3 is memory pool size(default 100)
        [INFO]  execute times = 100
        [INFO]  callback interval = 1
        [INFO]  memory pool size = 100
        [INFO]  acl init success
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  get run mode success
        [INFO]  load model ../model/resnet50.om success
        [INFO]  create model description success
        [INFO]  init memory pool success
        [INFO]  subscribe report success
        [INFO]  top 1: index[267] value[xxxxxx]
        [INFO]  top 1: index[161] value[xxxxxx]
        [INFO]  top 1: index[267] value[xxxxxx]
        [INFO]  top 1: index[161] value[xxxxxx]
        [INFO]  top 1: index[161] value[xxxxxx]
        [INFO]  top 1: index[267] value[xxxxxx]
        [INFO]  top 1: index[161] value[xxxxxx]
        [INFO]  top 1: index[267] value[xxxxxx]
        [INFO]  top 1: index[161] value[xxxxxx]
        [INFO]  top 1: index[267] value[xxxxxx]
        ......
        [INFO]  top 1: index[161] value[xxxxxx]
        [INFO]  top 1: index[267] value[xxxxxx]
        [INFO]  model execute success
        [INFO]  unsubscribe report success
        [INFO]  unload model success, modelId is 1
        [INFO]  execute sample success
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  end to reset device is 0
        [INFO]  end to finalize acl
        ```



