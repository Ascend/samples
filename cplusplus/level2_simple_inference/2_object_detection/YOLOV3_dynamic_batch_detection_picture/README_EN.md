# Object Detection with Caffe YOLOv3 \(Dynamic Batch/Image Size\)<a name="EN-US_TOPIC_0302603644"></a>

## Overview<a name="section340311417417"></a>

This sample implements object detection based on the Caffe YOLOv3 network in the dynamic batch/image size scenario.

The Caffe YOLOv3 network is converted into an .om offline model that adapts to the Ascend AI Processor. In the conversion command, you need to set the batch size choices \(for example,  **1**,  **2**,  **4**, and  **8**\) or set the image size choices \(for example,  **416**,  **416**,  **832**,  **832**,  **1248**, and  **1248**\). In your app, load the .om file, select the batch/image size for inference by passing the corresponding argument, and output the inference result to a file.

## Principles<a name="section3558105154116"></a>

This sample involves the following key functions:

-   Initialization
    -   **aclInit**: initializes AscendCL.
    -   **aclFinalize**: deinitializes AscendCL.

-   Device management
    -   **aclrtSetDevice**: sets the compute device.
    -   **aclrtGetRunMode**: obtains the run mode of the  Ascend AI Software Stack. The internal processing varies with the run mode.
    -   **aclrtResetDevice**: resets the compute device and cleans up all resources associated with the device.

-   **Context management**
    -   **aclrtCreateContext**: creates a context.
    -   **aclrtDestroyContext**: destroys a context.

-   Stream management
    -   **aclrtCreateStream**: creates a stream.
    -   **aclrtDestroyStream**: destroys a stream.

-   Memory management
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.

-   Data transfer

    **aclrtMemcpy**: copies memory.

-   Model inference
    -   **aclmdlLoadFromFileWithMem**: loads a model from an .om file.
    -   **aclmdlSetDynamicBatchSize**  or  **aclmdlSetDynamicHWSize**: sets the batch or image size choices.
    -   **aclmdlExecute**: performs synchronous model inference.
    -   **aclmdlUnload**: unloads a model.

-   Data postprocessing

    **DumpModelOutputResult**: outputs the model inference result to a file. \(After the executable file is executed, the inference result file is stored in the same directory in the  operating environment  as the executable file of the app.\)

    ```
    processModel.DumpModelOutputResult();
    ```


## Directory Structure<a name="section14723181815424"></a>

The sample directory is organized as follows:

```
├── data
│   ├── tools_generate_data.py            //Script for generating test data

├── inc
│   ├── model_process.h              //Header file that declares functions related to model processing
│   ├── sample_process.h              //Header file that declares functions related to resource initialization and destruction
│   ├── utils.h                       //Header file that declares common functions (such as file reading function)

├── src
│   ├── acl.json         //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── main.cpp               //Main function, which is the implementation file of image classification
│   ├── model_process.cpp      //Implementation file of model processing functions
│   ├── sample_process.cpp     //Implementation file of functions related to resource initialization and destruction
│   ├── utils.cpp              //Implementation file of common functions (such as the file reading function)

├── .project     //Project information file, including the project type, project description, and type of the target device
├── CMakeLists.txt    //Build script that calls the CMakeLists file in the src directory
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

## Environment Variables<a name="section9202202861519"></a>

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



## Build and Run \(Ascend310 EP/Ascend 710\)<a name="section17151737173112"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.
    2.  Complete  **Preparations**, including obtaining the tool and setting environment variables, as described in "Getting Started with ATC" in  _ATC Tool Instructions_.
    3.  Prepare data.

        Download the .prototxt model file and .caffemodel pre-trained model file of the YOLOv3 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        [https://github.com/Ascend-Huawei/models/tree/master/computer\_vision/object\_detect/yolov3](https://github.com/Ascend-Huawei/models/tree/master/computer_vision/object_detect/yolov3)

        Find the download link in the  **README\_en.md**  file.

    4.  Go to the sample directory and convert the YOLOv3 network into an .om offline model that adapts to the Ascend AI Processor.

        If the input for model inference allows a dynamic batch size, run the following command to convert the model:

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:-1,3,416,416" --input_format=NCHW --dynamic_batch_size="1,2,4,8" --soc_version=${soc_version} --output=model/yolov3_dynamic_batch 
        ```

        If the input for model inference allows a dynamic image size, run the following command to convert the model:

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:1,3,-1,-1" --input_format=NCHW --dynamic_image_size="416,416;832,832;1248,1248" --soc_version=${soc_version} --output=model/yolov3_dynamic_hw 
        ```

        -   **--model**: directory of the source model file.
        -   **--weight**: directory of the weight file.
        -   **--framework**: source framework type, selected from  **0**  \(Caffe\),  **1**  \(MindSpore\),  **3**  \(TensorFlow\), and  **5**  \(ONNX\).
        -   **--input\_shape**: input shape.
        -   **--input\_format**: input format.
        -   **--dynamic\_batch\_size**: dynamic batch size choices. Applies to the scenario where image count per inference batch is unfixed.
        -   **--dynamic\_image\_size**: dynamic image size choices. Applies to the scenario where image size per inference batch is unfixed.
        -   **--soc\_version**: SoC version, either  **Ascend310**  or  **Ascend710**.
        -   **--output**: directory for storing the generated  **yolov3\_dynamic\_batch.om**  or  **yolov3\_dynamic\_hw.om**  file, that is,  **/model**  under the sample directory. The default path in the command example is recommended. To specify another path, you need to change the value of  **omModelPath**  in  **sample\_process.cpp**  before building the code.

            ```
            string omModelPath = "../model/yolov3_dynamic_batch.om";
            ......
            string omModelPath = "../model/yolov3_dynamic_hw.om";
            ```



2.  Build the code.
    1.  Log in to the  development environment  as the running user.
    2.  Run the  **tools\_generate\_data.py **script in  **/data**  under the sample directory. The test bin files corresponding to different batch sizes or image sizes are generated in  **/data**  under the sample directory.

        For example, run the following command to generate the  **input\_float32\_1x3x416x416.bin.in**  file for: batch size = 1, channel number = 3, image size = 416 x 416 pixels, and data type = float32.

        ```
        python3.7 tools_generate_data.py input -s [1,3,416,416] -r [2,3] -d float32
        ```

        The configuration options in  **tools\_generate\_data.py**  are described as follows:

        -   **input**: prefix of the .bin file name.
        -   **s**: shape of the input data.
        -   **r**: pixel value range of each channel. The value range is \[0, 255\]. After the  **tools\_generate\_data.py**  script is executed, the pixel value of each channel is randomly generated within the value range specified by the  **-r**  argument.
        -   **d**: data format, selected from  **int8**,  **uint8**,  **float16**,  **float32**,  **int32**, and  **uint32**.

    3.  Go to the sample directory and create a directory for storing build outputs. For example, the directory created in this sample is  **build/intermediates/host**.

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


    5.  Run the  **make **command. The  **main**  executable file is generated in  **/out**  under the sample directory.

        ```
        make
        ```


3.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_yolov3\_dynamic\_batch**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_yolov3\_dynamic\_batch/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_yolov3\_dynamic\_batch/out**\) and run the executable file.
        -   In the dynamic batch size scenario, run the following command. Replace the input argument with the actual batch size, which should be among the choices specified by the  **--dynamic\_batch\_size**  argument in the model conversion command.

            ```
            ./main 1
            ```

            The following messages indicate that the file is successfully executed.

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_batch.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model output success.
            [INFO]  set dynamic batch size[1] success.
            [INFO]  model input num[2], output num[3].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[16613376], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[-1][3][416][416]
            [INFO]  index[1]: name[ascend_mbatch_shape_data], inputSize[8], format[2], dataType[9]
            [INFO]  dimcount:[1],dims:[1]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[layer82-conv:0:layer82-conv], outputSize[1379040], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[8][255][13][13]
            [INFO]  index[1]: name[layer94-conv:0:layer94-conv], outputSize[5516160], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[8][255][26][26]
            [INFO]  index[2]: name[layer106-conv:0:layer106-conv], outputSize[22064640], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[8][255][52][52]
            [INFO]  start to print model dynamic batch info:
            [INFO]  dynamic batch count:[4],dims:{[1][2][4][8]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][255][13][13]
            [INFO]  index:1,dims:[1][255][26][26]
            [INFO]  index:2,dims:[1][255][52][52]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            [INFO]  end to finalize acl.
            ```


        -   In the dynamic image size scenario, run the following command. Replace the input arguments with the actual image height and width, which should be among the choices specified by the  **--dynamic\_image\_size**  argument in the model conversion command.

            ```
            ./main 416 416
            ```

            The following messages indicate that the file is successfully executed.

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_hw.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model input success.
            [INFO]  create model output success.
            [INFO]  set dynamic hw[416, 416] success.
            [INFO]  model input num[2], output num[3].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[18690048], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][3][-1][-1]
            [INFO]  index[1]: name[ascend_mbatch_shape_data], inputSize[16], format[2], dataType[9]
            [INFO]  dimcount:[1],dims:[2]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[layer82-conv:0:layer82-conv], outputSize[1551420], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][255][39][39]
            [INFO]  index[1]: name[layer94-conv:0:layer94-conv], outputSize[6205680], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][255][78][78]
            [INFO]  index[2]: name[layer106-conv:0:layer106-conv], outputSize[24822720], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][255][156][156]
            [INFO]  start to print model dynamic hw info:
            [INFO]  dynamic hw count:[3],dims:{[416, 416][832, 832][1248, 1248]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][255][13][13]
            [INFO]  index:1,dims:[1][255][26][26]
            [INFO]  index:2,dims:[1][255][52][52]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  destroy model description success.
            [INFO]  destroy model input success.
            [INFO]  destroy model output success.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            ```




## Build and Run \(Atlas 200 DK\)<a name="section518661623815"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.
    2.  Complete  **Preparations**, including obtaining the tool and setting environment variables, as described in "Getting Started with ATC" in  _ATC Tool Instructions_.
    3.  Prepare data.

        Download the .prototxt model file and .caffemodel pre-trained model file of the YOLOv3 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        [https://github.com/Ascend-Huawei/models/tree/master/computer\_vision/object\_detect/yolov3](https://github.com/Ascend-Huawei/models/tree/master/computer_vision/object_detect/yolov3)

        Find the download link in the  **README\_en.md**  file.

    4.  Go to the sample directory and convert the YOLOv3 network into an .om offline model that adapts to the Ascend AI Processor.

        If the input for model inference allows a dynamic batch size, run the following command to convert the model:

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:-1,3,416,416" --input_format=NCHW --dynamic_batch_size="1,2,4,8" --soc_version=${soc_version} --output=model/yolov3_dynamic_batch 
        ```

        If the input for model inference allows a dynamic image size, run the following command to convert the model:

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:1,3,-1,-1" --input_format=NCHW --dynamic_image_size="416,416;832,832;1248,1248" --soc_version=${soc_version} --output=model/yolov3_dynamic_hw 
        ```

        -   **--model**: directory of the source model file.
        -   **--weight**: directory of the weight file.
        -   **--framework**: source framework type, selected from  **0**  \(Caffe\),  **1**  \(MindSpore\),  **3**  \(TensorFlow\), and  **5**  \(ONNX\).
        -   **--input\_shape**: input shape.
        -   **--input\_format**: input format.
        -   **--dynamic\_batch\_size**: dynamic batch size choices. Applies to the scenario where image count per inference batch is unfixed.
        -   **--dynamic\_image\_size**: dynamic image size choices. Applies to the scenario where image size per inference batch is unfixed.
        -   **--soc\_version**: SoC version, either  **Ascend310**  or  **Ascend710**.
        -   **--output**: directory for storing the generated  **yolov3\_dynamic\_batch.om**  or  **yolov3\_dynamic\_hw.om**  file, that is,  **/model**  under the sample directory. The default path in the command example is recommended. To specify another path, you need to change the value of  **omModelPath**  in  **sample\_process.cpp**  before building the code.

            ```
            string omModelPath = "../model/yolov3_dynamic_batch.om";
            ......
            string omModelPath = "../model/yolov3_dynamic_hw.om";
            ```



2.  Build the code.
    1.  Log in to the  development environment  as the running user.
    2.  Run the  **tools\_generate\_data.py **script in the sample directory. The test bin files corresponding to different batch sizes or image sizes are generated in  **/data**  under the sample directory.

        For example, run the following command to generate the  **input\_float32\_1x3x416x416.bin.in**  file for: batch size = 1, channel number = 3, image size = 416 x 416 pixels, and data type = float32.

        ```
        python3.7 tools_generate_data.py input -s [1,3,416,416] -r [2,3] -d float32
        ```

        The configuration options in  **tools\_generate\_data.py**  are described as follows:

        -   **input**: prefix of the .bin file name.
        -   **s**: shape of the input data.
        -   **r**: pixel value range of each channel. The value range is \[0, 255\]. After the  **tools\_generate\_data.py**  script is executed, the pixel value of each channel is randomly generated within the value range specified by the  **-r**  argument.
        -   **d**: data format, selected from  **int8**,  **uint8**,  **float16**,  **float32**,  **int32**, and  **uint32**.

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

    5.  Run the  **make **command. The  **main**  executable file is generated in  **/out**  under the sample directory.

        ```
        make
        ```


3.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  board environment, for example,  **$HOME/acl\_yolov3\_dynamic\_batch**.
    2.  Log in to the  board environment  as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_yolov3\_dynamic\_batch/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_yolov3\_dynamic\_batch/out**\) and run the executable file.
        -   In the dynamic batch size scenario, run the following command. Replace the input argument with the actual batch size, which should be among the choices specified by the  **--dynamic\_batch\_size**  argument in the model conversion command.

            ```
            ./main 1
            ```

            The following messages indicate that the file is successfully executed.

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_batch.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model output success.
            [INFO]  set dynamic batch size[1] success.
            [INFO]  model input num[2], output num[3].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[16613376], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[-1][3][416][416]
            [INFO]  index[1]: name[ascend_mbatch_shape_data], inputSize[8], format[2], dataType[9]
            [INFO]  dimcount:[1],dims:[1]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[layer82-conv:0:layer82-conv], outputSize[1379040], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[8][255][13][13]
            [INFO]  index[1]: name[layer94-conv:0:layer94-conv], outputSize[5516160], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[8][255][26][26]
            [INFO]  index[2]: name[layer106-conv:0:layer106-conv], outputSize[22064640], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[8][255][52][52]
            [INFO]  start to print model dynamic batch info:
            [INFO]  dynamic batch count:[4],dims:{[1][2][4][8]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][255][13][13]
            [INFO]  index:1,dims:[1][255][26][26]
            [INFO]  index:2,dims:[1][255][52][52]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            [INFO]  end to finalize acl.
            ```


        -   In the dynamic image size scenario, run the following command. Replace the input arguments with the actual image height and width, which should be among the choices specified by the  **--dynamic\_image\_size**  argument in the model conversion command.

            ```
            ./main 416 416
            ```

            The following messages indicate that the file is successfully executed.

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_hw.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model input success.
            [INFO]  create model output success.
            [INFO]  set dynamic hw[416, 416] success.
            [INFO]  model input num[2], output num[3].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[18690048], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][3][-1][-1]
            [INFO]  index[1]: name[ascend_mbatch_shape_data], inputSize[16], format[2], dataType[9]
            [INFO]  dimcount:[1],dims:[2]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[layer82-conv:0:layer82-conv], outputSize[1551420], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][255][39][39]
            [INFO]  index[1]: name[layer94-conv:0:layer94-conv], outputSize[6205680], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][255][78][78]
            [INFO]  index[2]: name[layer106-conv:0:layer106-conv], outputSize[24822720], format[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][255][156][156]
            [INFO]  start to print model dynamic hw info:
            [INFO]  dynamic hw count:[3],dims:{[416, 416][832, 832][1248, 1248]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][255][13][13]
            [INFO]  index:1,dims:[1][255][26][26]
            [INFO]  index:2,dims:[1][255][52][52]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  destroy model description success.
            [INFO]  destroy model input success.
            [INFO]  destroy model output success.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            ```




