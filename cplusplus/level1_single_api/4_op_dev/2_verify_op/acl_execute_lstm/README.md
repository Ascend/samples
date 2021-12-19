# Operator Verification on Network: LSTM<a name="EN-US_TOPIC_0302083145"></a>

## Overview<a name="section1421916179418"></a>

This sample verifies the function of the  [custom operator LSTMTik](../../1_custom_op/doc/LSTM_EN.md)  by converting the custom operator file into a single-operator offline model file and loading the file using AscendCL for execution. 

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
│               └── lstm_op.json    // Operator description file, used to construct a single-operator model file
│           ├── data
│               └── compare_result.py    // Script for generating the expected result and comparing with the actual result
│               └── generate_data.py    // Script for generating test data
├── src
│   ├── CMakeLists.txt    // Build script
│   ├── common.cpp         // Common function file, used to read binary files
│   ├── main.cpp    // File containing the operator configuration, used to build the single-operator Add into an OM file and load the OM file for execution. To verify other single-operators, modify the configuration based on this file.
│   ├── operator_desc.cpp     // File used to construct the input and output descriptions of the operator
│   ├── op_runner.cpp   // Function implementation file for building and running a single-operator
```

## Environment Requirements<a name="en-us_topic_0230709958_section1256019267915"></a>

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64
-   SoC: Ascend 310
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed
-   Custom operator built and deployed by referring to  [custom\_op](../../1_custom_op)

## Environment Variables<a name="section053142383519"></a>

-   Ascend 310 EP
    1.  In the development environment, set the environment variables for generating a single-operator offline model.

        The following is an example:

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH # Specify Python 3.7.5 to use if multiple versions of Python 3 exist, and specify your Python 3.7.5 installation path. Set the path of the ATC executable file.
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

        Replace  **$HOME/Ascend**  with the actual Ascend-CANN-Toolkit installation path.

    2.  In the development environment, set environment variables and configure the header search path and library search path on which the build of the AscendCL single-operator verification program depends.

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
    
    3.  In the operating environment, set the environment variable of the ACLlib path on which app execution depends.
    
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


-   Atlas 200 DK
    1.  In the development environment, set the environment variables for generating a single-operator offline model.

        The following is an example:

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=/usr/local/python3.7.5/bin:${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH # Specify Python 3.7.5 to use if multiple versions of Python 3 exist, and specify your Python 3.7.5 installation path. Set the path of the ATC executable file.
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

        Replace  **$HOME/Ascend**  with the actual Ascend-CANN-Toolkit installation path.

    2.  In the development environment, set environment variables and configure the header search path and library search path on which the build of the AscendCL single-operator verification program depends.

        The following is an example. Replace  **$HOME/Ascend**  with the actual Ascend-CANN-Toolkit installation path.

        ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
        ```

        ```
        Note:
        Dependency on the *.so libraries in the lib64/stub directory of the ACLlib installation path is to avoid dependency on any *.so library of other components during building the code logic based on the AscendCL API. After the build is complete, an app that runs on the board will be linked to the *.so libraries in the acllib/lib64 directory included in the LD_LIBRARY_PATH environment variable, and be automatically linked to the *.so libraries on which other components depend.
        ```

        The environment variable  **LD\_LIBRARY\_PATH**  on which AscendCL depends has been configured in the bootable SD card preparation phase. You do not need to configure it separately.



## Build and Run \(Ascend 310 EP\)<a name="section1042793885117"></a>

1.  Generate the single-operator offline model file of the LSTMTik operator.
    1.  Log in to the development environment as a running user \(for example,  **HwHiAiUser**\) and go to the  **acl\_execute\_lstm/run/out**  directory of the sample project.
    2.  Run the following command in the  **out **directory to generate a single-operator model file:

        **atc --singleop=test\_data/config/lstm\_op.json  --soc\_version=_$\{soc\_version\}_  --output=op\_models**

        Specifically,

        -   **singleop**: operator description file \(.json\).
        -   **soc\_version**: Ascend AI Processor version.
        -   **--output=op\_models**: indicates that the generated model file is stored in the  **op\_models**  folder in the current directory.

        After the model conversion is successful, the following files are generated:

        The single-operator model file  **0\_LSTMTik\_xxxx.om**  is generated in the  **op\_models**  subdirectory of the current directory. The file is named in the format of "No.+opType+input description \(dataType\_format\_shape\)+output description".

        View the enumerated values of  **dataType **and format in the  **atc/include/graph/types.h**  file. The enumerated values start from 0 and increase in ascending order.

        **Note:**  During model conversion, operators in the custom OPP are preferentially searched to match the operators in the model file.


2.  Generate test data.

    Go to the  **run/out/test\_data/data**  directory of the sample project and run the following command:

    **python3.7.5 generate\_data.py**

    The data file  **input\_\*.bin**  and expected result file  **output\_golden\_\*.bin**  are generated in the current directory for LSTMTik operator verification.

3.  Build the sample project to generate an executable for single-operator verification.
    1.  Change  **acllib**  to  **fwkacllib**  in the  **src/CMakeLists.txt**  file.

        ```
        # Header path
        include_directories(
            ${INC_PATH}/acllib/include/
            ../inc/
        )
        ```

    2.  Go to the  **acl\_execute\_lstm**  directory of the sample project and run the following command in this directory to create a directory for storing the generated executable, for example,  **build/intermediates/host**.

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
    
        The executable  **execute\_lstm\_op**  is generated in the  **run/out**  directory of the project.


4.  Execute the single-operator verification file on the host of the hardware device.
    1.  As a running user \(for example,  **HwHiAiUser**\), copy the  **out**  folder in the  **acl\_execute\_lstm/run/**  directory of the sample project in the  development environment  to any directory in the  operating environment  \(host\), for example,  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_lstm/**.

        **Note**: If your development environment is the host of the hardware device, skip this step.

    2.  Execute the  **execute\_lstm\_op**  file in the  operating environment  to verify the single-operator model file.

        Run the following commands in  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_lstm/out**:

        **chmod +x execute\_lstm\_op**

        **./execute\_lstm\_op**

        After the execution is complete, the result binary file  **result\_files/output\_\*.bin**  is generated.

    3.  Compare the actual result with the expected result.

        Go to the  **test\_data/data**  directory and run the following command:

        **python3.7.5 compare\_result.py**

        This script is used to compare the actual operator running result with the expected result.

        If the following information is displayed, the actual running result is consistent with the expected result.

        ```
        compare success
        ```

        If the following information is displayed, the actual running result is not consistent with the expected result.

        ```
        compare failed
        ```



## Build and Run \(Atlas 200 DK\)<a name="section17953191113524"></a>

1.  Generate the single-operator offline model file of the LSTMTik operator.
    1.  Log in to the development environment as a running user \(for example,  **HwHiAiUser**\) and go to the  **acl\_execute\_lstm/run/out**  directory of the sample project.
    2.  Run the following command in the  **out**  directory to generate a single-operator model file:

        **atc --singleop=test\_data/config/lstm\_op.json  --soc\_version=Ascend310 --output=op\_models**

        Specifically,

        -   **singleop**: operator description file \(.json\).
        -   **soc\_version**: Ascend AI Processor version.
        -   **--output=op\_models**: indicates that the generated model file is stored in the  **op\_models**  folder in the current directory.

        After the model conversion is successful, the following files are generated:

        The single-operator model file  **0\_LSTMTik\_xxxx.om**  is generated in the  **op\_models**  subdirectory of the current directory. The file is named in the format of "No.+opType+input description \(dataType\_format\_shape\)+output description".

        View the enumerated values of  **dataType **and format in the  **atc/include/graph/types.h**  file. The enumerated values start from 0 and increase in ascending order.

        **Note:**  During model conversion, operators in the custom OPP are preferentially searched to match the operators in the model file.


2.  Generate test data.

    Go to the  **run/out/test\_data/data**  directory of the sample project and run the following command:

    **python3.7.5 generate\_data.py**

    The data file  **input\_\*.bin**  and expected result file  **output\_golden\_\*.bin**  are generated in the current directory for LSTMTik operator verification.

3.  Build the sample project to generate an executable for single-operator verification.
    1.  Go to the  **acl\_execute\_lstm**  directory of the sample project and run the following command in this directory to create a directory for storing the generated executable, for example,  **build/intermediates/host**.

        **mkdir -p build/intermediates/host**

    2.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

        **cd build/intermediates/host**

        **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.
        -   **DCMAKE\_CXX\_COMPILER**: compiler used to build the app.
        -   **DCMAKE\_SKIP\_RPATH**: If it is set to  **TRUE**,  **rpath**  \(path specified by  **NPU\_HOST\_LIB**\) is not added to the executable generated after build.

            The executable automatically looks up for dynamic libraries in the path \(**_xxx_/acllib/lib64**\) included in  **LD\_LIBRARY\_PATH**.


    3.  Run the following command to generate an executable:
    
        **make**
    
        The executable  **execute\_lstm\_op**  is generated in the  **run/out**  directory of the project.


4.  Execute the single-operator verification file in the operating environment.
    1.  As a running user \(for example,  **HwHiAiUser**\), copy the  **out**  folder in the  **acl\_execute\_lstm/run/**  directory of the sample project in the development environment to any directory in the board environment, for example,  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_lstm/**.
    2.  Execute the  **execute\_lstm\_op**  file in the  board environment  to verify the single-operator model file.

        Run the following commands in  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_lstm/out**:

        **chmod +x execute\_lstm\_op**

        **./execute\_lstm\_op**

        After the execution is complete, the input and output data is displayed on the screen, and the result binary file  **result\_files/output\_\*.bin**  is generated.

    3.  Find the  **compare\_result.py**  script provided in the  **test\_data/data**  directory to compare the actual operator result with the expected result.

        You can copy the  **out**  folder generated after the result file is generated to the development environment, go to the  **test\_data/data**  directory in the development environment, and perform the following operations to verify the result:

        **python3.7.5 compare\_result.py**

        This script is used to compare the actual operator running result with the expected result.

        If the following information is displayed, the actual running result is consistent with the expected result.

        ```
        compare success
        ```

        If the following information is displayed, the actual running result is not consistent with the expected result.

        ```
        compare failed
        ```



