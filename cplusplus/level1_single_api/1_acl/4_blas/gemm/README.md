# Matrix-Matrix Multiplication<a name="EN-US_TOPIC_0302603657"></a>

## Overview<a name="section14302794244"></a>

This example implements matrix-matrix multiplication and outputs a 16 x 16 matrix: C = αAB + βC, where, A, B, and C are all 16 x 16 matrices.

## Principles<a name="section1934715933412"></a>

The following lists the key functions involved in this sample.

-   Initialization
    -   **aclInit**: initializes AscendCL.
    -   **aclFinalize**: deinitializes AscendCL.

-   Device management
    -   **aclrtSetDevice**: sets the compute device.
    -   **aclrtGetRunMode**: obtains the run mode of the  Ascend AI Software Stack. The internal processing varies with the run mode.
    -   **aclrtResetDevice**: resets the compute device and cleans up all resources associated with the device.

-   Stream management
    -   **aclrtCreateStream**: creates a stream.
    -   **aclrtDestroyStream**: destroys a stream.
    -   **aclrtSynchronizeStream**: waits for stream tasks to complete.

-   Memory management
    -   **aclrtMallocHost**: allocates host memory.
    -   **aclrtFreeHost**: frees host memory.
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.

-   Data transfer
    -   **aclrtMemcpy**: copies memory.

-   Single-operator execution
    -   **aclblasGemmEx**: implements matrix-matrix multiplication. You can specify the data types of the elements in the matrices. A matrix-matrix multiplication operator GEMM has been encapsulated in the  **aclblasGEMMEx**  API.
    -   Ascend Tensor Compiler \(ATC\): builds the GEMM operator description \(including the input and output tensor description and operator attributes\) into an offline model \(.om file\) that adapts to the Ascend AI Processor, to verify the GEMM execution result.


## Directory Structure<a name="section1338083213243"></a>

The sample directory is organized as follows:

```
├── inc                                 
│   ├── common.h                    //Header file that declares common functions (such as file reading function)
│   ├── gemm_runner.h               //Header file that declares the functions related to matrix multiplication
                 
├── run
│   ├── out  
│   │   ├──test_data
│   │   │   ├── config                           
│   │   │   │     ├── acl.json           //Configuration file for system initialization
│   │   │   │     ├── gemm.json          //Description information file of the matrix multiplication operator
│   │   │   ├── data                           
│   │   │   │     ├── generate_data.py   //Data used to generate matrix A and matrix B

├── src
│   ├── CMakeLists.txt             //Build script
│   ├── common.cpp                 //Implementation file of common functions (such as the file reading function)
│   ├── gemm_main.cpp              //Implementation file of the main function
│   ├── gemm_runner.cpp            //Implementation file for executing functions related to matrix multiplication
```

## Environment Requirements<a name="section3833348101215"></a>

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, Ubuntu 18.04 aarch64,  EulerOS x86, EulerOS AArch64
-   Compiler: g++ or aarch64-linux-gnu-g++
-   SoC: Ascend 310 AI Processor, Ascend 310P AI Processor, Ascend 910 AI Processor
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed


## Environment Variables<a name="section4721588588"></a>

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


## Build and Run <a name="section105241721131111"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.

    2.  Build the operator description information \(.json file\) of the matrix-matrix multiplication operator into an offline model \(.om file\) that adapts to the Ascend AI Processor.

        Run the following command in the  **acl\_execute\_gemm**  directory (take Ascend310 as an example):

        ```
        atc --singleop=run/out/test_data/config/gemm.json --soc_version=Ascend310 --output=run/out/op_models
        ```

        -   **--singleop**: directory of the single-operator definition file \(.json\)
        -   **--soc\_version**: SoC version, either  **Ascend310**  or  **Ascend310P**.
            -   Ascend 310 AI Processor, set this parameter to **Ascend310**.
            -   Ascend 310P AI Processor, set this parameter to **Ascend310P**.
            -   Ascend 910 AI Processor, set this parameter to **Ascend910A** or **Ascend910B** or **Ascend910ProA** or **Ascend910ProB** or **Ascend910PremiumA**. **Pro** or **Premium** indicate the performance improvement level. **A** or **B** indicate PartialGood level. Select a value based on the site requirements.
        -   **--output**: directory for storing the generated .om file, that is, the  **run/out/op\_models**  directory.


2.  Build the code.
    1.  Log in to the  development environment  as the running user.
    2.  Go to the  **/run/out/test\_data/data**  directory under the sample directory and run the  **generate\_data.py**  script. The data files  **matrix\_a.bin**  of matrix A,  **matrix\_b.bin**  of matrix B, and  **matrix\_c.bin**  of matrix C are generated to the  **run/out/test\_data/data**  directory.

        ```
        python3.7.5 generate_data.py
        ```

        Data in the  **output.bin**  file generated in the  **run/out/test\_data/data**  directory is the matrix-matrix multiplication result calculated using the formula in the  **generate\_data.py**  script and is not used as the input data of this sample.

    3.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/device**.

        ```
        mkdir -p build/intermediates/host
        ```

    4.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

        Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.

        Set **DCMAKE_SKIP_RPATH** to  **TRUE**,  **rpath**  (path specified by  **NPU_HOST_LIB**) is not added to the executable generated after build. The executable automatically looks up for dynamic libraries in the path  included in  **LD_LIBRARY_PATH**.

        -   If the operating system architecture of the  development environment is the same as that of the running environment, run the following commands to perform compilation.

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
            ```

        -   If the operating system architecture of the  development environment is different from that of the running environment, run the following commands to perform cross compilation.

            For example, if the development environment is x86 and the running environment is AArch64 , run the following command:
            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
            ```


    5.  Run the  **make **command. The  **execute\_gemm\_op**  executable file is generated in  **/out**  under the sample directory.

        ```
        make
        ```


3.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_execute\_gemm**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **execute\_gemm\_op**  is located \(for example,  **$HOME/acl\_execute\_gemm/out**\) and grant execute permission on the  **execute\_gemm\_op**  file in the directory.

        ```
        chmod +x execute_gemm_op
        ```

    4.  Go to the directory where the executable file  **execute\_gemm\_op**  is located \(for example,  **$HOME/acl\_execute\_gemm/out**\) and run the executable file.

        ```
        ./execute_gemm_op
        ```

        After the command is executed successfully, the data of matrix A and matrix B, and the matrix-matrix multiplication result are displayed in the terminal window. In addition, a  **matrix\_c.bin**  file that stores the matrix-matrix multiplication result is generated in the  **result\_files**  directory.



