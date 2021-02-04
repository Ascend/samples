# Execution Verification of the Conv2d Operator<a name="EN-US_TOPIC_0302083387"></a>

## Overview<a name="section1421916179418"></a>

This sample verifies the function of the  [custom operator Conv2dTik](../../1_custom_op/doc/Conv2d_EN.md)  by converting the custom operator file into a single-operator offline model file and loading the file using AscendCL for execution. 

Note: The generation of a single-operator model file depends only on the operator code implementation file, operator prototype definition, and operator information library, but does not depend on the operator adaptation plug-in.

## Directory Structure<a name="section8733528154320"></a>

```
├── inc                           // Header file directory
│   ├── common.h                  // Common method class declaration file, used to read binary files
│   ├── operator_desc.h          // Operator description declaration file, including the operator inputs and outputs, operator type, input description, and output description
│   ├── op_runner.h              // Operator execution information declaration file, including the numbers and sizes of operator inputs and outputs
├── run                           // Directory of files required for single-operator execution
│   ├── out    // Directory of executable files required for single-operator execution
│        └── test_data         // Directory of test data files
│           ├── config
│               └── acl.json       // File for AscendCL initialization, which must not be modified
│               └── conv2d_tik_op.json    // Operator description file, used to construct a single-operator model file
│           ├── data
│               └── check_out.py    // Script for generating the expected result and comparing with the actual result
│               └── generate_conv2d.py    // Script for generating test data
├── src
│   ├── CMakeLists.txt    // Build script
│   ├── common.cpp         // Common function file, used to read binary files
│   ├── main.cpp    // File containing the operator configuration, used to build the single-operator Add into an OM file and load the OM file for execution. To verify other single-operators, modify the configuration based on this file.
│   ├── operator_desc.cpp     // File used to construct the input and output description of the operator
│   ├── op_runner.cpp   // Function implementation file for building and running a single-operator
```

## Environment Requirements<a name="en-us_topic_0230709958_section1256019267915"></a>

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64
-   Version: 20.2
-   Compiler:
    -   Ascend 310 EP/Ascend 710: g++
    -   Atlas 200 DK: AArch64-linux-gnu-g++

-   SoC: Ascend 310 AI Processor, Ascend 710 AI Processor
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed
-   The custom operator has been compiled and deployed by referring to  [custom\_op](../../1_custom_op).

## Environment Variables<a name="section053142383519"></a>

-   Ascend 310 EP/Ascend 710
    1.  In the development environment, set the environment variables for generating a single-operator offline model.

        The following is an example:

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

        **install\_path**  specifies the installation path of  Ascend-cann-toolkit.

    2.  In the development environment, set environment variables of the header file path and library file path on which the AscendCL single-operator verification program compilation depends.

        The build script looks up for the header files and library files based on the paths specified by those environment variables. Set  **_$HOME/Ascend_**_**/ascend-toolkit/latest**_  to the installation path of  Ascend-cann-toolkit.

        -   Configure environment variables as follows if the OS architecture is x86.

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/acllib/lib64/stub
            ```

        -   Configure environment variables as follows if the OS architecture is ARM64.

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
            ```


        ```
        Note:
        The .so library files in the lib64/stub directory of the ACLlib installation path are used to build code using AscendCL APIs without depending on any .so library of other components.
        At run time on the host, the app links to the .so library files in the $HOME/Ascend/nnrt/latest/acllib/lib64 directory on the host through the configured environment variable and automatically links to the .so library files of other components.
        ```

    3.  In the operating environment, set the environment variable of the ACLlib path on which app execution depends.

        The following is an example only. Replace  **_$HOME/Ascend_**_**/nnrt/latest**_  with the actual Ascend-CANN-NNRT installation path.

        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```


-   Atlas 200 DK
    1.  In the development environment, set the environment variables for generating a single-operator offline model.

        The following is an example:

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

        **install\_path**  specifies the installation path of  Ascend-cann-toolkit.

    2.  In the development environment, set environment variables of the header file path and library file path on which the AscendCL single-operator verification program compilation depends.

        The following is an example of setting environment variables. Replace  **_$HOME/Ascend_**_**/ascend-toolkit/latest**_  with the installation path of the  Ascend-cann-toolkit  toolkit.

        ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
        ```

        ```
        Note:
        The .so library files in the lib64/stub directory of the ACLlib installation path are used to build code using AscendCL APIs without depending on any .so library of other components.
        In the run time in the board environment, the app links to the .so library files in the $HOME/Ascend/acllib/lib64 directory in the board environment through the configured environment variable and automatically links to the .so library files of other components.
        ```

        The environment variable  **LD\_LIBRARY\_PATH**  on which AscendCL depends has been configured in the bootable SD card preparation phase. You do not need to configure it separately.



## Build and Run \(Ascend 310 EP/Ascend 710\)<a name="section13498193914019"></a>

1.  Generate the single-operator offline model file of the Conv2dTik operator.
    1.  Log in to the development environment as a running user \(for example,  **HwHiAiUser**\) and go to the  **acl\_execute\_conv2d/run/out**  directory of the sample project.
    2.  Run the following command in the  **out**  directory to generate a single-operator model file:

        **atc --singleop=test\_data/config/conv2d\_tik\_op.json  --soc\_version=_$\{soc\_version\} _  --output=op\_models**

        Specifically,

        -   **singleop**  specifies the operator description file \(.json\).
        -   **soc\_version**  specifies the model of the Ascend AI Processor. Replace it with the actual version.

            It is the exact name of the .ini file in  **atc/data/platform\_config**  in the ATC installation path. If you still cannot determine the  **_$\{sos\_version\}_**  using the preceding method, perform the following operations:

            1.  Click  [here](https://support.huawei.com/enterprise/en/ascend-computing/cann-pid-251168373?category=operation-maintenance)  to download  _CANN Ascend-DMI Tool User Guide_.
            2.  Complete the operations described in  **Tool Usage Guide**  \>  **Before You Start**, and then proceed to  **Tool Usage Guide**  \>  **Querying Device Real-Time Status**.
            3.  Run the related command to view the chip details. For example, in the output of the  **ascend-dmi -i -dt**  command,  **Chip Name**  field corresponds to  **_$\{sos\_version\}_**.

        -   **--output=op\_models**: the generated model file stored in the  **op\_models**  folder in the current directory.

        After the model conversion is successful, the following files are generated:

        The single-operator model file  **0\_Conv2DTik\_1\_0\_8\_512\_7\_7\_1\_0\_512\_512\_3\_3\_1\_0\_8\_512\_7\_7.om**  is generated in the  **op\_models**  directory of the current directory. The file is named in the format of No.+opType+input description \(dateType\_format\_shape\)+output description.

        View the enumerated values of dataType and format in the  **atc/include/graph/types.h**  file. The enumerated values start from 0 and increase in ascending order.

        **Note:**  During model conversion, operators in the custom operator library are preferentially searched to match the operators in the model file.


2.  Generate test data.

    Go to the  **run/out/test\_data/data**  directory of the sample project and run the following command:

    **python3.7.5 generate\_conv2d.py**

    The data files  **input\_0.bin**  and  **input\_1.bin**  are generated in the current directory for verifying the Conv2dTik operator.

3.  Build the sample project to generate an executable file for single-operator verification.
    1.  Go to the  **acl\_execute\_conv2d**  directory of the sample project and run the following command in this directory to create a directory for storing the generated executable file, for example,  **build/intermediates/host**.

        **mkdir -p build/intermediates/host**

    2.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

        **cd build/intermediates/host**

        **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.
        -   **DCMAKE\_CXX\_COMPILER**: specifies the compiler used to build the app.
        -   **DCMAKE\_SKIP\_RPATH**: If set to  **TRUE**, this parameter indicates that  **rpath**  \(path specified by  **NPU\_HOST\_LIB**\) is not added to the executable file generated after build.

            The executable automatically looks up for dynamic libraries in the path \(**_xxx_/acllib/lib64**  or  **_xxx_/fwkacllib/lib64**\) included in  **LD\_LIBRARY\_PATH**.


    3.  Run the following command to generate an executable file:

        **make**

        The executable file  **execute\_conv2d\_op**  is generated in the  **run/out**  directory of the project.


4.  Execute the single-operator verification file on the host side of the hardware device.
    1.  As a running user \(for example,  **HwHiAiUser**\), copy the out folder in the  **acl\_execute\_conv2d/run/**  directory of the sample project in the  development environment  to any directory in the  operating environment  \(host\), for example,  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/**.

        **Note**: If your development environment is the host side of the hardware device, skip this step.

    2.  Execute the  **execute\_conv2d\_op**  file in the operating environment to verify the single-operator model file.

        Run the following command in  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/out**:

        **chmod +x execute\_conv2d\_op**

        **./execute\_conv2d\_op**

        After the execution is complete, the input and output data is displayed on the screen, and the result binary file  **result\_files/output\_0.bin**  is generated.

    3.  Compare the actual result with the expected result.

        Go to the  **test\_data/data**  directory and run the following command:

        **python3.7.5 check\_out.py**

        **Note:**  Before running this script, ensure that TensorFlow 1.15 has been installed in the environment. This script calls the TensorFlow command to generate the expected result and compares the actual result with the expected result.

        If the following information is displayed, the actual running result is consistent with the expected result.

        ```
        Compared with the tf conv2d method, the result is correct.
        ```

        If the following information is displayed, the actual running result is not consistent with the expected result.

        ```
        Compared with the tf conv2d method, the result is wrong.
        ```



## Build and Run \(Atlas 200 DK\)<a name="section9789152720117"></a>

1.  Generate the single-operator offline model file of the Conv2dTik operator.
    1.  Log in to the development environment as a running user \(for example,  **HwHiAiUser**\) and go to the  **acl\_execute\_conv2d/run/out**  directory of the sample project.
    2.  Run the following command in the  **out**  directory to generate a single-operator model file:

        **atc --singleop=test\_data/config/conv2d\_tik\_op.json  --soc\_version=Ascend310  --output=op\_models**

        Specifically,

        -   **singleop**  specifies the operator description file \(.json\).
        -   **soc\_version**  specifies the model of the Ascend AI Processor. 
        -   **--output=op\_models**: the generated model file stored in the  **op\_models**  folder in the current directory.

        After the model conversion is successful, the following files are generated:

        The single-operator model file  **0\_Conv2DTik\_1\_0\_8\_512\_7\_7\_1\_0\_512\_512\_3\_3\_1\_0\_8\_512\_7\_7.om**  is generated in the  **op\_models**  directory of the current directory. The file is named in the format of No.+opType+input description \(dateType\_format\_shape\)+output description.

        View the enumerated values of dataType and format in the  **atc/include/graph/types.h**  file. The enumerated values start from 0 and increase in ascending order.

        **Note:**  During model conversion, operators in the custom operator library are preferentially searched to match the operators in the model file.


2.  Generate test data.

    Go to the  **run/out/test\_data/data**  directory of the sample project and run the following command:

    **python3.7.5 generate\_conv2d.py**

    The data files  **input\_0.bin**  and  **input\_1.bin**  are generated in the current directory for verifying the Conv2dTik operator.

3.  Build the sample project to generate an executable file for single-operator verification.
    1.  Go to the  **acl\_execute\_conv2d**  directory of the sample project and run the following command in this directory to create a directory for storing the generated executable file, for example,  **build/intermediates/host**.

        **mkdir -p build/intermediates/host**

    2.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

        **cd build/intermediates/host**

        **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.
        -   **DCMAKE\_CXX\_COMPILER**: specifies the compiler used to build the app.
        -   **DCMAKE\_SKIP\_RPATH**: If set to  **TRUE**, this parameter indicates that  **rpath**  \(path specified by  **NPU\_HOST\_LIB**\) is not added to the executable file generated after build.

            The executable automatically looks up for dynamic libraries in the path \(**_xxx_/acllib/lib64**\) included in  **LD\_LIBRARY\_PATH**.


    3.  Run the following command to generate an executable file:

        **make**

        The executable file  **execute\_conv2d\_op**  is generated in the  **run/out**  directory of the project.


4.  Execute the single-operator verification file in the operating environment.
    1.  As a running user \(for example,  **HwHiAiUser**\), copy the out folder in the  **acl\_execute\_conv2d/run/**  directory of the sample project in the development environment to any directory in the board environment, for example,  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/**.
    2.  Execute the  **execute\_conv2d\_op**  file in the  board environment  to verify the single-operator model file.

        Run the following command in  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/out**:

        **chmod +x execute\_conv2d\_op**

        **./execute\_conv2d\_op**

        After the execution is complete, the input and output data is displayed on the screen, and the result binary file  **result\_files/output\_0.bin**  is generated.

    3.  The  **check\_out.py**  script in the  **test\_data/data**  directory is used to compare the actual operator running result with the expected result generated by calling the TensorFlow command.

        You can copy the  **out**  folder generated after the result file is generated to the development environment, go to the  **test\_data/data**  directory in the development environment, and perform the following operations to verify the result:

        **python3.7.5 check\_out.py**

        **Note:**  Before running this script, ensure that TensorFlow 1.15 has been installed in the development environment. This script calls the TensorFlow command to generate the expected result and compares the actual result with the expected result.

        If the following information is displayed, the actual running result is consistent with the expected result.

        ```
        Compared with the tf conv2d method, the result is correct.
        ```

        If the following information is displayed, the actual running result is not consistent with the expected result.

        ```
        Compared with the tf conv2d method, the result is wrong.
        ```



