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

-   OS and architecture: CentOS 7.6 x86\_64, CentOS AArch64, or Ubuntu 18.04 x86\_64
-   Version: 20.2
-   Compiler:
    -   Ascend 310 EP/Ascend 710: g++
    -   Atlas 200 DK: aarch64-linux-gnu-g++

-   SoC: Ascend 310 AI Processor or Ascend 710 AI Processor
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed

## Environment Variables<a name="section4721588588"></a>

-   **Ascend 310 EP/Ascend 710:**
    1.  In the development environment, set the environment variables on which model conversion depends.

        Replace  _**$\{install\_path\}**_  with the actual  Ascend-CANN-Toolkit  installation path.

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    2.  Set the header file path and library file path environment variables for the  **src/CMakeLists.txt**  build script in the development environment.

        The following is an example. Replace  ****_$HOME/Ascend_**_**/ascend-toolkit/latest**_/_\{os\_arch\}_**  with the ACLlib path under the  Ascend-CANN-Toolkit  directory of the corresponding architecture.

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


        Use the .so library files in the  ****_$HOME/Ascend_**_**/ascend-toolkit/latest**_/_\{os\_arch\}_/acllib/lib64/stub**  directory to build the code logic using AscendCL APIs, without depending on any .so library files of other components \(such as Driver\). At run time on the host, the app links to the .so library files in the  ****_$HOME/Ascend_**_**/nnrt/latest**_/acllib/lib64**  directory on the host through the configured environment variable and automatically links to the .so library files of other components.

    3.  Set the library path environment variable in the operating environment for app execution.

        The following is an example. Replace  ****_$HOME/Ascend_**_**/nnrt/latest**_**  with the ACLlib path.

        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```


-   **Atlas 200 DK:**

    You only need to set environment variables in the development environment. Environment variables in the operating environment have been set in the phase of preparing a bootable SD card.

    1.  In the development environment, set the environment variables on which model conversion depends.

        Replace  _**$\{install\_path\}**_  with the actual  Ascend-CANN-Toolkit  installation path.

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    2.  Set the header file path and library file path environment variables for the  **src/CMakeLists.txt**  build script in the development environment.

        The following is an example. Replace  ****_$HOME/Ascend_**_**/ascend-toolkit/latest**_/arm64-linux**  with the ACLlib path under the ARM  Ascend-CANN-Toolkit  directory.

        ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
        ```

        The .so library files in the  **_$HOME/Ascend_**_**/ascend-toolkit/latest**_**/arm64-linux/acllib/lib64/stub**  directory are required to build code using AscendCL APIs, without depending on any .so library files of other components \(such as Driver\). At run time in the  board environment, the app links to the .so library files in the  **$HOME/Ascend/acllib/lib64**  directory in the open-form ACLlib installation path in the  board environment  through the configured environment variable and automatically links to the .so library files of other components.



## Build and Run \(Ascend310 EP/Ascend 710\)<a name="section105241721131111"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.
    2.  Set environment variables.

        Replace  _**$\{install\_path\}**_  with the actual  Ascend-CANN-Toolkit  installation path.

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  Build the operator description information \(.json file\) of the matrix-matrix multiplication operator into an offline model \(.om file\) that adapts to the Ascend AI Processor.

        Run the following command in the  **acl\_execute\_gemm**  directory:

        ```
        atc --singleop=run/out/test_data/config/gemm.json --soc_version=${soc_version} --output=run/out/op_models
        ```

        -   **--singleop**: directory of the single-operator definition file \(.json\)
        -   **--soc\_version**: SoC version, either  **Ascend310**  or  **Ascend710**.
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



## Build and Run \(Atlas 200 DK\)<a name="section7393133582419"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.
    2.  Set environment variables.

        Replace  _**$\{install\_path\}**_  with the  Ascend-CANN-Toolkit  installation path.

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  Build the operator description information \(.json file\) of the matrix-matrix multiplication operator into an offline model \(.om file\) that adapts to the Ascend AI Processor.

        Run the following command in the  **acl\_execute\_gemm**  directory:

        ```
        atc --singleop=run/out/test_data/config/gemm.json --soc_version=${soc_version} --output=run/out/op_models
        ```

        -   **--singleop**: directory of the single-operator definition file \(.json\)
        -   **--soc\_version**: SoC version, either  **Ascend310**  or  **Ascend710**.
        -   **--output**: directory for storing the generated .om file, that is, the  **run/out/op\_models**  directory.


2.  Build the code.
    1.  Log in to the development environment as the running user.
    2.  Go to the  **/run/out/test\_data/data**  directory under the sample directory and run the  **generate\_data.py**  script. The data files  **matrix\_a.bin**  of matrix A,  **matrix\_b.bin**  of matrix B, and  **matrix\_c.bin**  of matrix C are generated to the  **run/out/test\_data/data**  directory.

        ```
        python3.7.5 generate_data.py
        ```

        Data in the  **output.bin**  file generated in the  **run/out/test\_data/data**  directory is the matrix-matrix multiplication result calculated using the formula in the  **generate\_data.py**  script and is not used as the input data of this sample.

    3.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/minirc**.

        ```
        mkdir -p build/intermediates/minirc
        ```

    4.  Go to the  **build/intermediates/minirc**  directory and run the  **cmake**  command.

        Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.

        ```
        cd build/intermediates/minirc
        cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
        ```

    5.  Run the  **make **command. The  **execute\_gemm\_op**  executable file is generated in  **/out**  under the sample directory.

        ```
        make
        ```


3.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  board environment, for example,  **$HOME/acl\_execute\_gemm**.
    2.  Log in to the  board environment  as the running user.
    3.  Go to the directory where the executable file  **execute\_gemm\_op**  is located \(for example,  **$HOME/acl\_execute\_gemm/out**\) and grant execute permission on the  **execute\_gemm\_op**  file in the directory.

        ```
        chmod +x execute_gemm_op
        ```

    4.  Go to the directory where the executable file  **execute\_gemm\_op**  is located \(for example,  **$HOME/acl\_execute\_gemm/out**\) and run the executable file.

        ```
        ./execute_gemm_op
        ```

        After the command is executed successfully, the data of matrix A and matrix B, and the matrix-matrix multiplication result are displayed in the terminal window. In addition, a  **matrix\_c.bin**  file that stores the matrix-matrix multiplication result is generated in the  **result\_files**  directory.



