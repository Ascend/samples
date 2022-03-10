# Data Preprocessing \(Image Cropping and Pasting\)<a name="EN-US_TOPIC_0302603622"></a>

## Overview<a name="section7940919203810"></a>

In this example, a selected ROI is cropped from the input image \(sized up to 10 x 6\) and pasted to the output canvas.

## Principles<a name="section6271153719394"></a>

The following lists the key functions involved in this sample.

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
    -   **aclrtSynchronizeStream**: waits for stream tasks to complete.

-   Memory management
    -   **aclrtMallocHost**: allocates host memory.
    -   **aclrtFreeHost**: frees host memory.
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.
    -   In data preprocessing, if you need to allocate device memory to store the input or output data, call  **acldvppMalloc**  to allocate memory and call  **acldvppFree**  to free memory.

-   Data transfer

    **aclrtMemcpy**: copies memory.

-   Data preprocessing

    **acldvppVpcCropAndPasteAsync**: crops a selected ROI from the input image according to  **cropArea**  and loads the cropped image to the output buffer.


## Directory Structure<a name="section1394162513386"></a>

The sample directory is organized as follows:

```
├── data
│   ├── dvpp_vpc_1920x1080_nv12.yuv            //Test image. Obtain the test image according to the guide and save it to the data directory.

├── inc
│   ├── dvpp_process.h               //Header file that declares functions related to data preprocessing
│   ├── sample_process.h               //Header file that declares functions related to model processing
│   ├── utils.h                       //Header file that declares common functions (such as file reading function)

├── src
│   ├── acl.json              //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── dvpp_process.cpp       //Implementation file of functions related to video processing
│   ├── main.cpp               //Main function, which is the implementation file of the image cropping and pasting functions
│   ├── sample_process.cpp     //Implementation file of functions related to resource initialization and destruction
│   ├── utils.cpp              //Implementation file of common functions (such as the file reading function)

├── CMakeLists.txt    //Build script that calls the CMakeLists file in the src directory
```

## Environment Requirements<a name="section3833348101215"></a>

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, Ubuntu 18.04 aarch64,  EulerOS x86, EulerOS AArch64
-   Compiler: g++ or aarch64-linux-gnu-g++
-   SoC: Ascend 310 AI Processor, Ascend 710 AI Processor, Ascend 910 AI Processor
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed

## Environment Variables<a name="section1931223812141"></a>

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



## Build and Run <a name="section16011204259"></a>

1.  Build the code.
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


2.  Prepare input images.

    Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp\_vpc\_1920x1080\_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_vpc_1920x1080_nv12.yuv)

3.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_vpc\_smallResolution\_crop**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_smallResolution\_crop/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Switch to the directory where the executable file  **main**  is located, for example,  **$HOME/acl\_vpc\_smallResolution\_crop/out**. The following command calls VPC twice to crop a 6 x 6 ROI from the input image and resize the cropped image to 224 x 224.

        ```
        ./main --inImgName ../data/dvpp_vpc_1920x1080_nv12.yuv --inFormat 1 --inWidth 1920 --inHeight 1080 --cLeftOffset 0 --cRightOffset 5 --cTopOffset 0 --cBottomOffset 5 --outImgName output_224_224.yuv --outFormat 1 --outWidth 224 --outHeight 224 --pLeftOffset 0 --pRightOffset 223 --pTopOffset 0 --pBottomOffset 223
        ```

        The options are described as follows.

        -   **inImgName**: input image file directory, including the file name.
        -   **inFormat**: input format.
        -   **inWidth**: width of the input image.
        -   **inHeight**: height of the input image.
        -   **cLeftOffset**: left offset of the crop ROI. Must be an even number.
        -   **cRightOffset**: right offset of the crop ROI. Must be an odd number.
        -   **cTopOffset**: offset of the cropped image. Must be an even number.
        -   **cBottomOffset**: bottom offset of the crop ROI. Must be an odd number.
        -   **outImgName**: output image file directory, including the file name.
        -   **outFormat**: output format.
        -   **outWidth**: width of the output image.
        -   **outHeight**: height of the output image.
        -   **pLeftOffset**: left offset of the paste ROI. Must be an even number aligned to 16.
        -   **pRightOffset**: right offset of the paste ROI. Must be an odd number.
        -   **pTopOffset**: top offset of the paste ROI. Must be an even number.
        -   **pBottomOffset**: bottom offset of the paste ROI. Must be an odd number.

        The following messages indicate that the file is successfully executed.

        ```
        [INFO]  acl init success
        [INFO]  set device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  get run mode success
        [INFO]  dvpp init resource success
        [INFO]  call SplitProcessCropAndPaste
        [INFO]  vpc crop and paste success
        [INFO]  execute sample success
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  end to reset device 0
        ```

