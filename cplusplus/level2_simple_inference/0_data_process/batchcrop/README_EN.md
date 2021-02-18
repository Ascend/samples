# Media Data Processing \(Single-Image, Multi-ROI Cropping\)<a name="EN-US_TOPIC_0302603665"></a>

## Overview<a name="section7940919203810"></a>

In this example, eight 224 x 224 child images \(YUV420SP NV12\) are cropped out from the YUV420SP \(NV12\) input based on the specified cropping areas.

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

    **acldvppVpcBatchCropAsync**: crops one or more selected ROIs from each input image and loads the cropped one or more images to the output buffer. 


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
│   ├── main.cpp               //Main function, which is the implementation file of single-image, multi-ROI cropping
│   ├── sample_process.cpp     //Implementation file of functions related to resource initialization and destruction
│   ├── utils.cpp              //Implementation file of common functions (such as the file reading function)

├── CMakeLists.txt    //Build script that calls the CMakeLists file in the src directory
```

## Environment Requirements<a name="section3833348101215"></a>

-   OS and architecture: CentOS 7.6 x86\_64, CentOS AArch64, or Ubuntu 18.04 x86\_64
-   Version: 20.2
-   Compiler:
    -   Ascend 310 EP/Ascend 710: g++
    -   Atlas 200 DK: aarch64-linux-gnu-g++

-   SoC: Ascend 310 AI Processor or Ascend 710 AI Processor
-   Ascend AI Software Stack deployed

## Environment Variables<a name="section137281211130"></a>

-   **Ascend 310 EP/Ascend 710:**
    1.  Set the header file path and library file path environment variables for the  **src/CMakeLists.txt**  build script in the development environment.

        The following is an example. Replace  **$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_**  with the ACLlib path in  Ascend-CANN-Toolkit  of the corresponding architecture.

        -   x86 operating environment:

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/acllib/lib64/stub
            ```

        -   ARM operating environment:

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
            ```


        The .so library files in the  **$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_/acllib/lib64/stub**  directory are required to build the code logic based on the AscendCL APIs, without depending on any .so library files of other components \(such as Driver\). After successful build, when you run an app on the host, the app can be linked to the .so library files in the  **$HOME/Ascend/nnrt/latest/acllib/lib64**  directory on the host by configuring corresponding environment variables. The app is automatically linked to the dependent .so library files of other components during runtime.

    2.  Set the library path environment variable in the operating environment for app execution.

        The following is an example. Replace  **$HOME/Ascend/nnrt/latest**  with the path of ACLlib.

        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```


-   **Atlas 200 DK:**

    You only need to set environment variables in the development environment. Environment variables in the operating environment have been set in the phase of preparing a bootable SD card.

    The following is an example. Replace  **$HOME/Ascend/ascend-toolkit/latest/arm64-linux**  with the ACLlib path in  Ascend-CANN-Toolkit  of the ARM architecture.

    ```
    export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
    export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
    ```

    The .so library files in the  **$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub**  directory are required to build the code logic based on the AscendCL APIs, without depending on any .so library files of other components \(such as Driver\). At run time, the app links to the .so library files in the  **$HOME/Ascend/acllib/lib64**  directory on the board through the configured environment variable and automatically links to the .so library files of other components.


## Build and Run \(Ascend310 EP/Ascend 710\)<a name="section19471849121012"></a>

1.  Build the code.
    1.  Log in to the  development environment  as the running user.
    2.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/host**.

        ```
        mkdir -p build/intermediates/host
        ```

    3.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

        Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.

        -   If the development environment and operating environment have the same OS architecture, run the following commands to perform compilation.

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
            ```

        -   If development environment and operating environment have different OS architectures, run the following commands to perform cross compilation.

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
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_vpc\_batchcrop**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_batchcrop/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_batchcrop/out**\) and run the executable file.

        ```
        ./main
        ```

        The following messages indicate that the file is successfully executed.

        ```
        [INFO]  aclInit success, ret = 0.
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  dvpp init resource success
        [INFO]  open file = ./dvpp_vpc_1920x1080_nv12.yuv success.
        [INFO]  start set inputDesc success.
        [INFO]  write out to file ./cropName0 success.
        [INFO]  write out to file ./cropName1 success.
        [INFO]  write out to file ./cropName2 success.
        [INFO]  write out to file ./cropName3 success.
        [INFO]  write out to file ./cropName4 success.
        [INFO]  write out to file ./cropName5 success.
        [INFO]  write out to file ./cropName6 success.
        [INFO]  write out to file ./cropName7 success.
        [INFO]  ProcessBatchCrop success.
        [INFO]  ProcessBatchCrop success.
        [INFO]  DestroyBatchCropResource start
        [INFO]  DestroyBatchCropResource end
        [INFO]  SampleProcess DestroyResource start.
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  0 deviceID
        [INFO]  end to reset device is 0
        [INFO]  SampleProcess DestroyResource success.
        [INFO]  end to finalize acl
        ......
        ```

        After the executable file is executed successfully, a result file is generated in the directory at the same level as the  **main**  file for later query.

        The child images cropped from the input image  **dvpp\_vpc\_1920×1980\_nv12.yuv**  are as follows:  **cropName0**,  **cropName1**,  **cropName2**,  **cropName3**,  **cropName4**,  **cropName5**,  **cropName6**, and  **cropName7**



## Build and Run \(Atlas 200 DK\)<a name="section112152503113"></a>

1.  Build the code.
    1.  Log in to the  development environment  as the running user.
    2.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/minirc**.

        ```
        mkdir -p build/intermediates/minirc
        ```

    3.  Go to the  **build/intermediates/minirc**  directory and run the  **cmake**  command.

        Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.

        ```
        cd build/intermediates/minirc
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
    1.  Upload the  **acl\_vpc\_batchcrop**  folder in the  development environment  to the  board environment, for example,  **$HOME/acl\_vpc\_batchcrop**, as the running user.
    2.  Log in to the  board environment  as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_batchcrop/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_batchcrop/out**\) and run the executable file.

        ```
        ./main
        ```

        The following messages indicate that the file is successfully executed.

        ```
        [INFO]  aclInit success, ret = 0.
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  dvpp init resource success
        [INFO]  open file = ./dvpp_vpc_1920x1080_nv12.yuv success.
        [INFO]  start set inputDesc success.
        [INFO]  write out to file ./cropName0 success.
        [INFO]  write out to file ./cropName1 success.
        [INFO]  write out to file ./cropName2 success.
        [INFO]  write out to file ./cropName3 success.
        [INFO]  write out to file ./cropName4 success.
        [INFO]  write out to file ./cropName5 success.
        [INFO]  write out to file ./cropName6 success.
        [INFO]  write out to file ./cropName7 success.
        [INFO]  ProcessBatchCrop success.
        [INFO]  ProcessBatchCrop success.
        [INFO]  DestroyBatchCropResource start
        [INFO]  DestroyBatchCropResource end
        [INFO]  SampleProcess DestroyResource start.
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  0 deviceID
        [INFO]  end to reset device is 0
        [INFO]  SampleProcess DestroyResource success.
        [INFO]  end to finalize acl
        ......
        ```

        After the executable file is executed successfully, a result file is generated in the directory at the same level as the  **main**  file for later query.

        The child images cropped from the input image  **dvpp\_vpc\_1920×1980\_nv12.yuv**  are as follows:  **cropName0**,  **cropName1**,  **cropName2**,  **cropName3**,  **cropName4**,  **cropName5**,  **cropName6**, and  **cropName7**



