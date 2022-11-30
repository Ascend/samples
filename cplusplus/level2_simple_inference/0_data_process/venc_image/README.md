# Media Data Processing \(Video Encoding\)<a name="EN-US_TOPIC_0302602980"></a>

## Overview<a name="section7940919203810"></a>

In this sample, a YUV420SP \(NV12\) image is encoded for  _n_  consecutive times to generate a video stream in H.265 format. \(_n_  is default to 16 and is configurable before running the app.\)


## Directory Structure<a name="section1394162513386"></a>

The sample directory is organized as follows:

```
├── data
│   ├── dvpp_venc_128x128_nv12.yuv            //Test image. Obtain the test image according to the guide and save it to the data directory.

├── inc
│   ├── venc_process.h               //Header file that declares functions related to data preprocessing
│   ├── sample_process.h               //Header file that declares functions related to model processing
│   ├── utils.h                       //Header file that declares common functions (such as file reading function)

├── src
│   ├── acl.json              //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── main.cpp               //Main function, which is the implementation file of multi-image, multi-ROI cropping
│   ├── sample_process.cpp     //Implementation file of functions related to resource initialization and destruction
│   ├── utils.cpp              //Implementation file of common functions (such as the file reading function)
│   ├── venc_process.cpp       //Implementation file of functions related to video processing

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

## Prepare pictures <a name="section145174543134"></a>

1.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level2_simple_inference/0_data_process/venc_image" sample directory.
     
     Note that the sample directories below refer to the "cplusplus/level2_simple_inference/0_data_process/venc_image" directory.

2.  Prepare test input images.

    Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dvpp\_venc\_128x128\_nv12.yuv](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dvpp_venc_128x128_nv12.yuv)


## Build and Run <a name="section492582041817"></a>

1.  To configure CANN basic environment variables and Python environment variables, see [Link](../../../environment/environment_variable_configuration.md).

2.  Build the code.
    1.  Log in to the  development environment  as the running user.

    2.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level2_simple_inference/0_data_process/venc_image" sample directory.
     
        Note that the sample directories below refer to the "cplusplus/level2_simple_inference/0_data_process/venc_image" directory.

    3.  Set environment variables and configure the paths of header files and library files that the program depends on for compilation.
  
        After the following environment variables are set, the compilation script will look for the compiled-dependent header files according to the "{DDK_PATH} environment variable value/runtime/include/acl" directory, and the compiled-independent library files according to the directory pointed to by the {NPU_HOST_LIB} environment variable. Replace "$HOME/Ascend" with the actual installation path of the "Ascend-cann-toolkit" package.
  
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
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/venc\_image**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/venc\_image/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/venc\_image/out**\) and run the executable file.

        After the file is executed successfully, the result file  **dvpp\_venc\_128x128.h265**  is generated in the  **output **directory at the same level as the  **main**  file for later query.

        When executing an executable file, you can add an argument to configure the number of encoding times, for example,** ./main 20**. If no argument is added, the image is encoded 16 times by default.

        ```
        ./main
        ```

        The following messages indicate that the file is successfully executed.

        ```
        [INFO]  ./main param, param is execute venc times(default 16)
        [INFO]  acl init success
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  get run mode success
        [INFO]  create process callback thread successfully, threadId = 139926343980800
        [INFO]  start check result fold:output/
        [INFO]  make directory successfully.
        [INFO]  check result success, fold exist
        [INFO]  process callback thread start
        [INFO]  venc init resource success
        [INFO]  success to callback, stream size:5431
        [INFO]  success to callback, stream size:704
        [INFO]  success to callback, stream size:693
        [INFO]  success to callback, stream size:922
        [INFO]  success to callback, stream size:1153
        [INFO]  success to callback, stream size:689
        [INFO]  success to callback, stream size:857
        [INFO]  success to callback, stream size:1273
        [INFO]  success to callback, stream size:785
        [INFO]  success to callback, stream size:976
        [INFO]  success to callback, stream size:869
        [INFO]  success to callback, stream size:1445
        [INFO]  success to callback, stream size:845
        [INFO]  success to callback, stream size:1003
        [INFO]  success to callback, stream size:922
        [INFO]  success to callback, stream size:905
        [INFO]  venc process success
        [INFO]  execute sample success
        [INFO]  destroy process callback thread success
        [INFO]  end to destroy context
        [INFO]  end to reset device is 0
        [INFO]  end to finalize acl
        ```


## Key Interfaces<a name="section6271153719394"></a>

The following lists the key functions and key interfaces involved in this sample.

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

-   Memory management
    -   **aclrtMallocHost**: allocates host memory.
    -   **aclrtFreeHost**: frees host memory.
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.
    -   In data preprocessing, if you need to allocate device memory to store the input or output data, call  **acldvppMalloc**  to allocate memory and call  **acldvppFree**  to free memory.

-   Data transfer

    **aclrtMemcpy**: copies memory.

-   Data preprocessing

    **aclvencSendFrame**: sends the images to be encoded to the encoder.



