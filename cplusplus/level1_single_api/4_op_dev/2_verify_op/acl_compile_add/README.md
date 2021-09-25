# Operator Verification on Network: Add<a name="EN-US_TOPIC_0302083215"></a>

## Overview<a name="section1421916179418"></a>

This sample verifies the function of the  [custom operator Add](../../1_custom_op/doc/Add_EN.md)  by using AscendCL online compile for execution.

Note: The generation of a single-operator model file depends only on the operator code implementation file, operator prototype definition, and operator information library, but does not depend on the operator adaptation plugin.

## Directory Structure<a name="section8733528154320"></a>

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

## Environment Requirements<a name="en-us_topic_0230709958_section1256019267915"></a>

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, EulerOS x86, EulerOS AArch64
-   Compiler:
    -   For Ascend 710, or Ascend 910:
        -   g++ in the x86 operating environment
        -   g++-aarch64-linux-gnu in the ARM64 operating environment

-   SoC: Ascend 710, or Ascend 910
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed
-   Custom operator built and deployed by referring to  [custom\_op](../../1_custom_op)

## Environment Variables<a name="section053142383519"></a>

-   Ascend 910
    1.  In the development environment, set environment variables and configure the header search path and library search path on which the build of the AscendCL single-operator verification program depends.

        The build script searches for the required header files and libraries through the paths specified by the environment variables. Replace  **$HOME/Ascend**  with the actual Ascend-CANN-Toolkit installation path.

        -   In the x86 operating environment

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/fwkacllib/lib64/stub
            ```

        -   In the ARM64 operating environment

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/fwkacllib/lib64/stub
            ```


        ```
        Note:
        Dependency on the *.so libraries in the lib64/stub directory of the FwkACLlib installation path is to avoid dependency on any *.so library of other components during building the code logic based on the AscendCL API. After the build is complete, an app that runs on the host will be linked to the *.so libraries in the fwkacllib/lib64 or acllib/lib64 directory included in the LD_LIBRARY_PATH environment variable, and be automatically linked to the *.so libraries on which other components depend.
        ```
    
    2.  In the operating environment, set the environment variable of the ACLlib path on which app execution depends.
    
        -   If Ascend-CANN-Toolkit is installed in the operating environment, set the environment variable as follows:
    
            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/ascend-toolkit/latest/fwkacllib/lib64
            ```
    
        -   If Ascend-CANN-NNRT is installed in the operating environment, set the environment variable as follows:
    
            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
            ```
    
        -   If Ascend-CANN-NNAE is installed in the operating environment, set the environment variable as follows:
    
            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/nnae/latest/fwkacllib/lib64
            ```


        Replace  **$HOME/Ascend**  with the actual component installation path.


-   Ascend 710
    1.  In the development environment, set environment variables and configure the header search path and library search path on which the build of the AscendCL single-operator verification program depends.

        The build script searches for the required header files and libraries through the paths specified by the environment variables. Replace  **$HOME/Ascend**  with the actual Ascend-CANN-Toolkit installation path.

        -   In the x86 operating environment

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/acllib/lib64/stub
            ```

        -   In the ARM64 operating environment

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
            ```


        ```
        Note:
        Dependency on the *.so libraries in the lib64/stub directory of the ACLlib installation path is to avoid dependency on any *.so library of other components during building the code logic based on the AscendCL API. After the build is complete, an app that runs on the host will be linked to the *.so libraries in the acllib/lib64 directory included in the LD_LIBRARY_PATH environment variable, and be automatically linked to the *.so libraries on which other components depend.
        ```
    
    2.  In the operating environment, set the environment variable of the ACLlib path on which app execution depends.
    
        The following is an example only. Replace  **_$HOME/Ascend_**_**/nnrt/latest**_  with the actual Ascend-CANN-NNRT installation path.
    
        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```

## Build and Run \(Ascend 710/Ascend 910\)<a name="section170442411445"></a>
1.  Generate test data.

    Go to the  **run/out/test\_data/data**  directory of the sample project and run the following command:

    **python3.7.5 generate\_data.py**

    Two data files  **input\_0.bin**  and  **input\_1.bin**  with shape \(8, 16\) and data type int32 are generated in the current directory for verifying the Add operator.

2.  Build the sample project to generate an executable for single-operator verification.
    1.  For Ascend 310 and Ascend 910, change  **acllib**  to  **fwkacllib**  in the  **src/CMakeLists.txt**  file. Skip this step for Ascend 710.

        ```
        # Header path
        include_directories(
            ${INC_PATH}/acllib/include/
            ../inc/
        )
        ```

    2.  Go to the  **acl\_execute\_add**  directory of the sample project and run the following command in this directory to create a directory for storing the generated executable, for example,  **build/intermediates/host**.

        **mkdir -p build/intermediates/host**

    3.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

        -   If the operating system architecture of the  development environment is the same as that of the running environment, run the following command:

            **cd build/intermediates/host**

            **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   If the operating system architecture of the  development environment is different from that of the running environment, a cross compiler is requird.

            For example, if the development environment is x86 and the running environment is AArch64 , run the following command:

            **cd build/intermediates/host**
            
            **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**


        The parameters are described as follows:
    
        -   Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.
        -   **DCMAKE\_CXX\_COMPILER**: compiler used to build the app.
        -   **DCMAKE\_SKIP\_RPATH**: If it is set to  **TRUE**,  **rpath**  \(path specified by  **NPU\_HOST\_LIB**\) is not added to the executable generated after build. The executable automatically looks up for dynamic libraries in the path \(**_xxx_/acllib/lib64**  or  **_xxx_/fwkacllib/lib64**\) included in  **LD\_LIBRARY\_PATH**.
    
    4.  Run the following command to generate an executable:
    
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

    

