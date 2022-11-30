# Matrix-Matrix Multiplication<a name="EN-US_TOPIC_0302603657"></a>

## Overview<a name="section14302794244"></a>

This example implements matrix-matrix multiplication and outputs a 16 x 16 matrix: C = αAB + βC, where, A, B, and C are all 16 x 16 matrices.



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
-   The Ascend AI software stack has been deployed on the environment and the corresponding environment variables have been configured. Please refer to the corresponding version of the CANN installation guide in [Link](https://www.hiascend.com/document).
    
     In the following steps, the development environment refers to the environment for compiling and developing code, and the operating environment refers to the environment for running programs such as operators, inference, or training. 

     The operating environment must have an Ascend AI processor. The development environment and the running environment can be co-located on the same server, or they can be set up separately. In separate scenarios, the executable files compiled in the development environment are executed in the running environment. If the operating system architecture of the development environment and the running environment is Different, you need to perform cross-compilation in the development environment.


## Prepare single operator model and test data <a name="section4721588590"></a>

1.  To configure CANN basic environment variables and Python environment variables, see [Link](../../../environment/environment_variable_configuration.md).

2.  Log in to the  development environment  as the running user.

3.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level1_single_api/1_acl/4_blas/gemm" sample directory.
     
     Please note that the sample directories below refer to the "cplusplus/level1_single_api/1_acl/4_blas/gemm" directory.

4.  Prepare test data.

     Go to the  **/run/out/test\_data/data**  directory under the sample directory and run the  **generate\_data.py**  script. The data files  **matrix\_a.bin**  of matrix A,  **matrix\_b.bin**  of matrix B, and  **matrix\_c.bin**  of matrix C are generated to the  **run/out/test\_data/data**  directory.

        ```
        python3.7.5 generate_data.py
        ```

        Data in the  **output.bin**  file generated in the  **run/out/test\_data/data**  directory is the matrix-matrix multiplication result calculated using the formula in the  **generate\_data.py**  script and is not used as the input data of this sample.

5.  Prepare single operator model.

    Build the operator description information \(.json file\) of the matrix-matrix multiplication operator into an offline model \(.om file\) that adapts to the Ascend AI Processor.

    Run the following command in the  **acl\_execute\_gemm**  directory (take Ascend310 as an example):

    ```
    atc --singleop=run/out/test_data/config/gemm.json --soc_version=Ascend310 --output=run/out/op_models
    ```

    -   **--singleop**: directory of the single-operator definition file \(.json\)
    -   **--soc\_version**: Version of the Ascend AI processor. Go to the CANN software installation directory/compiler/data/platform_config directory. The name of the .ini file is the version of the Ascend AI processor. Select the version as required. 
    -   **--output**: directory for storing the generated .om file, that is, the  **run/out/op\_models**  directory.


## Build and Run <a name="section105241721131111"></a>

1.  To configure CANN basic environment variables and Python environment variables, see [Link](../../../environment/environment_variable_configuration.md).

2.  Build the code.
    1.  Log in to the  development environment  as the running user.

    2.  After downloading the sample warehouse code and uploading it to the environment, please go to the "cplusplus/level1_single_api/1_acl/4_blas/gemm" sample directory.
     
         Please note that the sample directories below refer to the "cplusplus/level1_single_api/1_acl/4_blas/gemm" directory.

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

    4.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/device**.

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

            For example, if the development environment is x86 and the running environment is AArch64 , run the following command:
            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
            ```


    6.  Run the  **make **command. The  **execute\_gemm\_op**  executable file is generated in  **/out**  under the sample directory.

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



## Key Interfaces<a name="section1934715933412"></a>

The following lists the key functions and key interfaces involved in this sample.

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

