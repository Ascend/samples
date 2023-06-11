# Operator Verification on Network: Softmax

## Overview

This sample verifies the function of the  [custom operator Softmax](../../1_custom_op/doc/Softmax_EN.md)  by converting the custom operator file into a single-operator offline model file and loading the file using AscendCL for execution.

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
│               └── softmax_op.json    // Operator description file, used to construct a single-operator model file
│           ├── data
│               └── generate_data.py    // Script for generating test data
├── src
│   ├── CMakeLists.txt    // Build script
│   ├── common.cpp         // Common function file, used to read binary files
│   ├── main.cpp    // File containing the operator configuration, used to build the single-operator Softmax into an OM file and load the OM file for execution. To verify other single-operators, modify the configuration based on this file.
│   ├── operator_desc.cpp     // File used to construct the input and output descriptions of the operator
│   ├── op_runner.cpp   // Function implementation file for building and running a single-operator
```

## Environment Requirements

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, Ubuntu 18.04 aarch64,  EulerOS x86, EulerOS AArch64
-   SoC: Ascend 310, Ascend 310P, or Ascend 910
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

       For example, if the development environment is x86 and the running environment is AArch64 , the configuration example is as follows(Note that arm64-linux is only an example, replace it with the actual architecture directory name）:

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

## Build and Run

1.  Generate the single-operator offline model file of the Softmax operator.
    1.  Log in to the development environment as a running user \(for example,  **HwHiAiUser**\) and go to the  **acl\_execute\_softmax/run/out**  directory of the sample project.
    2.  Run the following command in the  **out**  directory to generate a single-operator model file:

        **atc --singleop=test\_data/config/softmax\_op.json  --soc\_version=*Ascend310*  --output=op\_models**

        Specifically,

        -   **singleop**: operator description file \(.json\).
        -   **soc\_version**: Ascend AI Processor version. Replace it with the actual version.

        -   **--output=op\_models**: indicates that the generated model file is stored in the  **op\_models**  folder in the current directory.

        After the model conversion is successful, the following files are generated:

        The single-operator model file  **0\_Softmax\_0\_2\_8\_16\_0\_2\_8\_16.om**  is generated in the  **op\_models**  subdirectory of the current directory. The file is named in the format of "No.+opType+input description \(dataType\_format\_shape\)+output description".

        View the enumerated values of  **dataType**  and format in the  **include/graph/types.h**  file. The enumerated values start from 0 and increase in ascending order.

        **Note:**  During model conversion, operators in the custom OPP are preferentially searched to match the operators in the model file.


2.  Generate test data.

    Go to the  **run/out/test\_data/data**  directory of the sample project and run the following command:

    **python3.7.5 generate\_data.py**

    Two data files  **input\_0.bin**  with shape \(8, 16\) and data type float32 are generated in the current directory for verifying the Softmax operator.

3. Build the sample project to generate an executable for single-operator verification.
   1.  Go to the  **acl\_execute\_softmax**  directory of the sample project and run the following command in this directory to create a directory for storing the generated executable, for example,  **build/intermediates/host**.

       **mkdir -p build/intermediates/host**

   2.  Go to the  **build/intermediates/host**  directory and run the  **cmake**  command.

       Replace  **../../../src**  with the actual directory of  **CMakeLists.txt**.

       Set **DCMAKE_SKIP_RPATH** to  **TRUE**,  **rpath**  (path specified by  **NPU_HOST_LIB**) is not added to the executable generated after build. The executable automatically looks up for dynamic libraries in the path  included in  **LD_LIBRARY_PATH**.

       -   If the operating system architecture of the  development environment is the same as that of the running environment, run the following command:

           **cd build/intermediates/host**

           **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

       -   If the operating system architecture of the  development environment is different from that of the running environment, a cross compiler is requird.

           For example, if the development environment is x86 and the running environment is AArch64 , run the following command:

           **cd build/intermediates/host**

           **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**


   3.  Run the following command to generate an executable:

       **make**

       The executable  **execute\_softmax\_op**  is generated in the  **run/out**  directory of the project.


4.  Execute the single-operator verification file on the host of the hardware device.
    1.  As a running user \(for example,  **HwHiAiUser**\), copy the  **out**  folder in the  **acl\_execute\_softmax/run/**  directory of the sample project in the  development environment  to any directory in the  operating environment  \(host\), for example,  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_softmax/**.

        **Note**: If your development environment is the host of the hardware device, skip this step.

    2.  Execute the  **execute\_softmax\_op**  file in the  operating environment  to verify the single-operator model file.

        Run the following commands in  **/home/HwHiAiUser/HIAI\_PROJECTS/run\_softmax/out**:

        **chmod +x execute\_softmax\_op**

        **./execute\_softmax\_op**

        The following information is displayed. \(Note: The data file is randomly generated by the data generation script, so the displayed data may be different.\)

        ```
        [INFO]  Input[0]:
               1.964     5.18075     8.34857     5.89489     6.94057     8.13515     1.64323     4.21575     3.59446     3.36891     1.16488      5.4633     1.66912     6.08848     3.85598     3.90269
             3.03632     7.83259     1.08189     1.78934     2.52593     5.17109     6.03337     2.68933     8.83906     6.13864     4.99742      8.4141     2.78913     7.59506     6.87752      5.8658
             6.03144      7.5082     5.73127     8.10299     3.49008     7.11084     6.99786     7.79289     7.41494     8.61127      5.2607     2.81838     2.63016      4.6929     3.33906     2.67384
             4.97893     5.61952     3.42433     8.79005      9.7224     6.40261     5.23827     6.80337     7.02011      5.2128     1.75835     9.01168     6.47643     1.35168     5.08609     3.41378
             7.44854     3.07921      6.3708     7.84267     5.86042     8.36725     8.94286     2.37981     9.04679      6.7751     6.30535     6.99987      9.3375     9.41325     3.41567     1.62766
             7.23167     1.01582     1.44395     8.02107     5.05319      9.9051     8.26118     7.80592     8.97341     7.72043     8.68981     2.44615     2.14303     8.94444     6.11917     6.17793
             4.25618     2.39497     3.62921     4.73749     2.17078     5.32865     6.18814     7.84277     4.54349     2.95626     8.41946     5.01646     5.16211     3.51067      9.6782     7.60835
             8.58792     1.69972     1.22532      3.5807     2.70074     1.37077     4.77957     1.79133     8.35138     5.00774     4.82327     1.40899     4.99465     4.45809     3.60725     1.89699
        [INFO]  Output[0]:
              0.000703057    0.017539    0.416653   0.0358224    0.101927     0.33658 0.000510132  0.00668208  0.00358996  0.00286506 0.000316181   0.0232658  0.00052351   0.0434742  0.00466304  0.00488598
              0.00112468    0.136152 0.000159306 0.000323201 0.000675101  0.00950934   0.0225233 0.000794942      0.3725   0.0250236  0.00799329     0.24354 0.000878365    0.107365   0.0523892   0.0190483
               0.0229292    0.100401   0.0169836    0.181992  0.00180589   0.0674789   0.0602699    0.133468   0.0914609    0.302551   0.0106088 0.000922524  0.00076425  0.00601274  0.00155276 0.000798371
              0.00406458  0.00771294 0.000858738    0.183722    0.466745   0.0168776  0.00526799   0.0251976   0.0312961   0.0051355 0.000162307    0.229307   0.0181707 0.000108073  0.00452433 0.000849723
               0.0331273 0.000419381   0.0112752   0.0491305  0.00676817   0.0830185    0.147625 0.000208384    0.163793   0.0168933   0.0105609    0.021151    0.219053    0.236292 0.000587126 9.82216e-05
               0.0248403 4.96193e-05 7.61346e-05   0.0547002  0.00281225    0.359926   0.0695453   0.0441113     0.14177   0.0404972    0.106763 0.000207411 0.000153175    0.137722  0.00816595  0.00866017
              0.00266494 0.000414355  0.00142363  0.00431237 0.000331138  0.00778847    0.018396   0.0962329  0.00355192 0.000726338    0.171307  0.00569994  0.00659365  0.00126449     0.60317    0.076123
                0.518667 0.000528908 0.000329119   0.0034696  0.00143919 0.000380646   0.0115064 0.000579649     0.40941   0.0144557   0.0120205 0.000395476   0.0142676  0.00834304  0.00356296 0.000644249
        [INFO]  Write output[0] success. output file = result_files/output_0.bin
        [INFO]  Run op success
        ```

        As shown above, the output result is the softmax of input 1. The Softmax operator has passed the verification.

        **result\_files/output\_0.bin**: result binary file.
