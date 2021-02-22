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

-   OS and architecture: CentOS 7.6 x86\_64, CentOS AArch64, or Ubuntu 18.04 x86\_64
-   Version: 20.2
-   Compiler:
    -   Ascend 310 EP/Ascend 710: g++
    -   Atlas 200 DK: aarch64-linux-gnu-g++

-   SoC: Ascend 310 AI Processor or Ascend 710 AI Processor
-   Ascend AI Software Stack deployed

## Environment Variables<a name="section1931223812141"></a>

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


## Build and Run \(Ascend310 EP/Ascend 710\)<a name="section16011204259"></a>

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
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_vpc\_smallResolution\_crop**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_smallResolution\_crop/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Switch to the directory where the executable file  **main**  is located, for example,  **$HOME/acl\_vpc\_smallResolution\_crop/out**. The following command calls VPC twice to crop a 6 x 6 ROI from the input image and resize the cropped image to 224 x 224.

        ```
        ./main --inImgName dvpp_vpc_1920x1080_nv12.yuv --inFormat 1 --inWidth 1920 --inHeight 1080 --cLeftOffset 0 --cRightOffset 5 --cTopOffset 0 --cBottomOffset 5 --outImgName output_224_224.yuv --outFormat 1 --outWidth 224 --outHeight 224 --pLeftOffset 0 --pRightOffset 223 --pTopOffset 0 --pBottomOffset 223
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



## Build and Run \(Atlas 200 DK\)<a name="section18557246182520"></a>

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
    1.  As the running user, upload the sample folder in the  development environment  to the  board environment, for example,  **$HOME/acl\_vpc\_smallResolution\_crop**.
    2.  Log in to the  board environment  as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_smallResolution\_crop/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Switch to the directory where the executable file  **main**  is located, for example,  **$HOME/acl\_vpc\_smallResolution\_crop/out**. The following command calls VPC twice to crop a 6 x 6 ROI from the input image and resize the cropped image to 224 x 224.

        ```
        ./main --inImgName dvpp_vpc_1920x1080_nv12.yuv --inFormat 1 --inWidth 1920 --inHeight 1080 --cLeftOffset 0 --cRightOffset 5 --cTopOffset 0 --cBottomOffset 5 --outImgName output_224_224.yuv --outFormat 1 --outWidth 224 --outHeight 224 --pLeftOffset 0 --pRightOffset 223 --pTopOffset 0 --pBottomOffset 223
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



