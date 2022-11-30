# Operator Verification on Network: Add

## Overview

This sample verifies the function of the  [custom operator Add](../../1_custom_op/doc/Add_EN.md)  by using AscendCL online compile for execution.

Note: The generation of a single-operator model file depends only on the operator code implementation file, operator prototype definition, and operator information library, but does not depend on the operator adaptation plugin.

## Directory Structure

```
├── inc                           // Header file directory
│   ├── common.h                  // Common method class declaration file, used to read binary files
│   ├── operator_desc.h          // Operator description declaration file, including the operator inputs and outputs, operator type, input description, and output description
│   ├── op_runner.h              // Operator execution information declaration file, including the numbers and sizes of operator inputs and outputs
├── run                           // Directory of files required for single-operator execution
│   ├── out    // Directory of executables required for single-operator execution
│        └── test_data         // Directory of test data files
│           ├── config
│               └── acl.json       // File for AscendCL initialization, which must not be modified
│           ├── data
│               └── generate_data.py    // Script for generating test data
├── src
│   ├── CMakeLists.txt    // Build script
│   ├── common.cpp         // Common function file, used to read binary files
│   ├── main.cpp    // File containing the operator configuration, used to build the single-operator Add into an OM file and load the OM file for execution. To verify other single-operators, modify the configuration based on this file.
│   ├── operator_desc.cpp     // File used to construct the input and output descriptions of the operator
│   ├── op_runner.cpp   // Function implementation file for building and running a single-operator
```

## Environment Requirements

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, Ubuntu 18.04 aarch64,  EulerOS x86, EulerOS AArch64
-   SoC: Ascend 310P, or Ascend 910
-   Python version and dependency library: Python 3.7.*x* (3.7.0 to 3.7.11) and Python 3.8.*x* (3.8.0 to 3.8.11).
-   Ascend AI Software Stack deployed
-   Custom operator built and deployed by referring to  [custom\_op](../../1_custom_op)

## Environment Variables

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

     After setting the following environment variables, the compilation script will find the header file that the compilation depends on according to the directory "{DDK_PATH}/runtime/include/acl", and the library file that the compilation depends on according to the directory pointed to by the {NPU_HOST_LIB} environment variable. Please replace  **$HOME/Ascend**  with the actual component installation path.

     -  If the operating system architecture of the  development environment is the same as that of the running environment, run the following command:

        ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
        export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
        ```

     - If the operating system architecture of the  development environment is different from that of the running environment, a cross compiler is requird.

       For example, if the development environment is x86 and the running environment is AArch64 , the configuration example is as follows（Note that arm64-linux is only an example, replace it with the actual architecture directory name）:

       ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
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


## Build and Run \(Ascend 310P/Ascend 910\)
1.  Generate test data.

    Go to the  **run/out/test\_data/data**  directory of the sample project and run the following command:

    **python3.7.5 generate\_data.py**

    Two data files  **input\_0.bin**  and  **input\_1.bin**  with shape \(8, 16\) and data type int32 are generated in the current directory for verifying the Add operator.

2. Build the sample project to generate an executable for single-operator verification.
    1.  Go to the  **acl\_execute\_add**  directory of the sample project and run the following command in this directory to create a directory for storing the generated executable, for example,  **build/intermediates/host**.

       **mkdir -p build/intermediates/host**

    2.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

       -   If the operating system architecture of the  development environment is the same as that of the running environment, run the following command:

           **cd build/intermediates/host**

           **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

       -   If the operating system architecture of the  development environment is different from that of the running environment, a cross compiler is requird.

           For example, if the development environment is x86 and the running environment is AArch64 , run the following command:

           **cd build/intermediates/host**

           **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

         The parameters are described as follows:

         - Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.
         - **DCMAKE\_CXX\_COMPILER**: compiler used to build the app.
         - **DCMAKE\_SKIP\_RPATH**: If it is set to  **TRUE**,  **rpath**  \(path specified by  **NPU\_HOST\_LIB**\) is not added to the executable generated after build. The executable automatically looks up for dynamic libraries in the path included in  **LD\_LIBRARY\_PATH**.

    3.  Run the following command to generate an executable:

        **make**

        The executable  **execute\_add\_op**  is generated in the  **run/out**  directory of the project.


3.  Execute the single-operator verification file on the host of the hardware device.
    1.  As a running user \(for example,  **HwHiAiUser**\), copy the  **out**  folder in the  **acl\_execute\_add/run/**  directory of the sample project in the  development environment  to any directory in the  operating environment  \(host\), for example,  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_add/**.

        **Note**: If your development environment is the host of the hardware device, skip this step.

    2.  Execute the  **execute\_add\_op**  file in the  operating environment  to verify the single-operator model file.

        Run the following commands in  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_add/out**:

        **chmod +x execute\_add\_op**

        **./execute\_add\_op**

        The following information is displayed. \(Note: The data file is randomly generated by the data generation script, so the displayed data may be different.\)

        ```
        [INFO]  Input[0]:
                -4        -4        -1         7         0         9        -4         5        -9        -9        -3         7                                                                                                -6         1         8         3
                -6        -9         8        -3        -9         0         0         4        -3         7        -6        -9                                                                                                 6         6         1        -8
                -7         7        -3         5         8        -3         6        -4         6         9         8       -10                                                                                                 7         3         3         9
                -4         6         5         6        -5         3        -1         1         1        -8        -4         9                                                                                                -6        -9         6        -8
                 5         8         5         2        -9         5        -8        -2        -1       -10        -5         5                                                                                                 7       -10        -8       -10
                 0         3        -7         8         3         3       -10         5        -7         6        -3         2                                                                                                 7       -10        -8         0
                -2        -5         8        -4         1         8         4        -5        -7         1        -9         8                                                                                                 2         3        -3         5
                 8        -6        -8        -5         8       -10         5        -4        -5        -1         0       -10                                                                                                 8         6        -6        -3
        [INFO]  Set input[1] from test_data/data/input_1.bin success.
        [INFO]  Input[1]:
                -8        -1        -3         9        -2         8        -9         7        -7         7        -5         4                                                                                                 9         6        -2         9
                -6         1        -3         9        -5         5         4        -4        -8        -7        -1         9                                                                                                 6         0         9       -10
                -6         6        -1        -2        -3         5         1         3        -4         0         6         4                                                                                                -4       -10        -2         7
                 9         2         2         6        -7        -8         9         6        -2        -5        -8         5                                                                                                 9        -5         1         7
                -9        -3        -9        -4         6         0         5        -4        -4         1        -1         2                                                                                                 1         7         8       -10
                 1         3        -5        -8       -10        -3        -7         7         8        -3        -9         5                                                                                                -7        -6        -6        -4
                -3         3         4        -5         5         4        -9         0        -8         2        -3        -6                                                                                                 5         4        -6        -8
                 0         8         9        -2         4         1         8        -6        -8         1        -1        -9                                                                                                -2         0       -10         7
           ......
           ......
        [INFO]  Output[0]:
               -12        -5        -4        16        -2        17       -13        12       -16        -2        -8        11                                                                                                 3         7         6        12
               -12        -8         5         6       -14         5         4         0       -11         0        -7         0                                                                                                12         6        10       -18
               -13        13        -4         3         5         2         7        -1         2         9        14        -6                                                                                                 3        -7         1        16
                 5         8         7        12       -12        -5         8         7        -1       -13       -12        14                                                                                                 3       -14         7        -1
                -4         5        -4        -2        -3         5        -3        -6        -5        -9        -6         7                                                                                                 8        -3         0       -20
                 1         6       -12         0        -7         0       -17        12         1         3       -12         7                                                                                                 0       -16       -14        -4
                -5        -2        12        -9         6        12        -5        -5       -15         3       -12         2                                                                                                 7         7        -9        -3
                 8         2         1        -7        12        -9        13       -10       -13         0        -1       -19                                                                                                 6         6       -16         4
        [INFO]  Write output[0] success. output file = result_files/output_0.bin
        [INFO]  Run op success
        ```

        As shown above, the output result is the sum of input 1 and input 2. The Add operator has passed the verification.

        **result\_files/output\_0.bin**: result binary file.



