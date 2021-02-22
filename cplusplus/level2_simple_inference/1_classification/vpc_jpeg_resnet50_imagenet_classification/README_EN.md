# Image Classification with Caffe ResNet-50 \(Image Decoding+Cropping and Resizing+Image Encoding+Synchronous Inference\)<a name="EN-US_TOPIC_0302603658"></a>

## Overview<a name="section7940919203810"></a>

This sample shows how to classify images based on the Caffe ResNet-50 network \(single input with batch size = 1\).

According to the arguments of the app, the following functions can be implemented:

-   Encodes a YUV420SP image into a .jpg image.
-   Decodes two .jpg images into the YUV420SP \(NV12\) format, resizes the images, performs model inference to obtain the inference results of the two images, processes the inference results, and outputs the class indexes with the top confidence values and the sum of the top 5 confidence values.
-   Decodes two .jpg images into the YUV420SP \(NV12\) format, crops selected ROIs from the images, performs model inference to obtain the inference results of the two images, processes the inference results, and outputs the class indexes with the top confidence values and the sum of the top 5 confidence values.
-   Decodes two .jpg images into the YUV420SP \(NV12\) format, crops selected ROIs from the images and pastes each cropped image in the canvas, performs model inference to obtain the inference results of the two images, processes the inference results, and outputs the class indexes with the top confidence values and the sum of the top 5 confidence values.
-   Resizes 8192 x 8192 images in YUV420SP \(NV12\) format to 4000 x 4000.

Convert the source model file into an offline model that adapts to the Ascend AI Processor in advance. During model conversion, you need to set color space conversion \(CSC\) parameters to convert YUV420SP images into RGB images to meet the input requirements of the model.

## Principles<a name="section6271153719394"></a>

The following lists the key functions involved in this sample.

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
    -   **aclrtSynchronizeStream**: waits for stream tasks to complete.

-   Memory management
    -   **aclrtMallocHost**: allocates host memory.
    -   **aclrtFreeHost**: frees host memory.
    -   **aclrtMalloc**: allocates device memory.
    -   **aclrtFree**: frees device memory.
    -   In data preprocessing, if you need to allocate device memory to store the input or output data, call  **acldvppMalloc**  to allocate memory and call  **acldvppFree**  to free memory.

-   Data transfer

    **aclrtMemcpy**: copies memory.

-   **Data preprocessing**
    -   Image encoding

        **acldvppJpegEncodeAsync**: encodes YUV420SP images into .jpg images.

    -   Image decoding

        **acldvppJpegDecodeAsync**: decodes .jpg images into YUV420SP images.

    -   Resizing

        **acldvppVpcResizeAsync**: resizes the YUV420SP input.

    -   Cropping

        **acldvppVpcCropAsync**: crops a selected ROI from the input image and loads the cropped image to the output buffer.

    -   Cropping and pasting

        **acldvppVpcCropAndPasteAsync**: crops a selected ROI from the input image according to  **cropArea**  and loads the cropped image to the output buffer.


-   Model inference
    -   **aclmdlLoadFromFileWithMem**: loads a model from an .om file.
    -   **aclmdlExecute**: performs model inference.

        Before inference, use the CSC parameters in the .om file to convert a YUV420SP image into an RGB image.

    -   **aclmdlUnload**: unloads a model.


## Directory Structure<a name="section1394162513386"></a>

The sample directory is organized as follows:

```
├── caffe_model
│   ├── aipp.cfg        //Configuration file with CSC parameters, used for model conversion

├── data
│   ├── persian_cat_1024_1536_283.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.
│   ├── wood_rabbit_1024_1061_330.jpg            //Test image. Obtain the test image according to the guide and save it to the data directory.
│   ├── wood_rabbit_1024_1068_nv12.yuv            //Test image. Obtain the test image according to the guide and save it to the data directory.
│   ├── dvpp_vpc_8192x8192_nv12.yuv            //Test image. Obtain the test image according to the guide and save it to the data directory.

├── inc
│   ├── dvpp_process.h               //Header file that declares functions related to data preprocessing
│   ├── model_process.h              //Header file that declares functions related to model processing
│   ├── sample_process.h              //Header file that declares functions related to resource initialization and destruction
│   ├── utils.h                       //Header file that declares common functions (such as file reading function)

├── src
│   ├── acl.json         //Configuration file for system initialization
│   ├── CMakeLists.txt         //Build script
│   ├── dvpp_process.cpp       //Implementation file of functions related to video processing
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

## Environment Variables<a name="section1025910381783"></a>

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



## Build and Run \(Ascend310 EP/Ascend 710\)<a name="section183454368119"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.
    2.  Set environment variables.

        Replace  _**$\{install\_path\}**_  with the  Ascend-CANN-Toolkit  installation path.

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  Prepare data.

        Download the .prototxt model file and .caffemodel pre-trained model file of the ResNet-50 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        [https://github.com/Ascend-Huawei/models/tree/master/computer\_vision/classification/resnet50](https://github.com/Ascend-Huawei/models/tree/master/computer_vision/classification/resnet50)

        Find the download links in the  **README\_en.md**  file.

    4.  Convert the ResNet-50 network into an offline model \(.om file\) that adapts to Ascend AI Processors. During model conversion, you need to set CSC parameters to convert YUV420SP images to RGB images.

        Go to the sample directory and run the following command:

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --soc_version=${soc_version} --insert_op_conf=caffe_model/aipp.cfg --output=model/resnet50_aipp 
        ```

        -   **--model**: directory of the source model file.
        -   **--weight**: directory of the weight file.
        -   **--framework**: source framework type, selected from  **0**  \(Caffe\),  **1**  \(MindSpore\),  **3**  \(TensorFlow\), and  **5**  \(ONNX\).
        -   **--soc\_version**: SoC version, either  **Ascend310**  or  **Ascend710**.
        -   **--insert\_op\_conf**: path of the configuration file for inserting the AI Pre-Processing \(AIPP\) operator for AI Core–based image preprocessing including image resizing, CSC, and mean subtraction and factor multiplication \(for pixel changing\), prior to model inference.
        -   **--output**: directory for storing the generated  **resnet50\_aipp.om**  file, that is,  **/model**  under the sample directory. The default path in the command example is recommended. To specify another path, you need to change the value of  **omModelPath**  in  **sample\_process.cpp**  before building the code.

            ```
            const char* omModelPath = "../model/resnet50_aipp.om";
            ```



2.  Build the code.
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


3.  Prepare input images.

    Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp\_vpc\_8192x8192\_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_vpc_8192x8192_nv12.yuv)

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/persian\_cat\_1024\_1536\_283.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/persian_cat_1024_1536_283.jpg)

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood\_rabbit\_1024\_1061\_330.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1061_330.jpg)

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood\_rabbit\_1024\_1068\_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1068_nv12.yuv)

4.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  operating environment  \(host\), for example,  **$HOME/acl\_vpc\_jpege\_resnet50**.
    2.  Log in to the  operating environment  \(host\) as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_jpege\_resnet50/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_jpege\_resnet50/out**\) and run the executable file.

        1.  Decode two .jpg images into two YUV420SP \(NV12\) images, resize them, and perform model inference to obtain the inference results of the two images.

            ```
            ./main 0
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcResize
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[283], top1[0.500488], top5[0.863968]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcResize
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[330], top1[0.542480], top5[1.0000063]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        2.  Decode two .jpg images into two YUV420SP \(NV12\) images, crop selected ROIs from the images, and perform model inference to obtain the inference results of the two images.

            ```
            ./main 1
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcCrop
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[284], top1[0.961914], top5[0.999743]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcCrop
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[330], top1[0.631836], top5[0.998885]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        3.  Decode two .jpg images into two YUV420SP \(NV12\) images, crop selected ROIs from the images and paste each cropped image in the canvas, and perform model inference to obtain the inference results of the two images.

            ```
            ./main 2
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcCropAndPaste
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[284], top1[0.483398], top5[0.855194]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcCropAndPaste
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[331], top1[0.670898], top5[0.963564]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        4.  Encodes a YUV420SP image into a .jpg image.

            ```
            ./main 3
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] start to process picture:../data/wood_rabbit_1024_1068_nv12.yuv
            [INFO] call JpegE
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        5.  Resize the 8192 x 8192 image in YUV420SP format to 4000 x 4000.

            ```
            ./main 4
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] get run mode success
            [INFO] dvpp process 8k resize begin
            [INFO] dvpp init resource success
            [INFO] dvpp process 8k resize success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            [INFO] end to finalize acl
            ```


        After the executable file is executed successfully, a result file is generated in the  **result**  directory at the same level as the  **main**  file for later query. The result file includes the following:

        -   **dvpp\_output\_0**: output image generated after  **persian\_cat\_1024\_1536\_283.jpg**  is resized, cropped, or cropped and pasted.
        -   **dvpp\_output\_1**: output image generated after  **wood\_rabbit\_1024\_1061\_330.jpg**  is resized, cropped, or cropped and pasted.
        -   **model\_output\_0**: model inference result \(a binary file\) of  **persian\_cat\_1024\_1536\_283.jpg**.
        -   **model\_output\_0.txt**: model inference result \(a .txt file\) of  **persian\_cat\_1024\_1536\_283.jpg**.
        -   **model\_output\_1**: model inference result \(a binary file\) of  **wood\_rabbit\_1024\_1061\_330.jpg**.
        -   **model\_output\_1.txt**: model inference result \(a .txt file\) of  **wood\_rabbit\_1024\_1061\_330.jpg**.
        -   **jpege\_output\_0.jpg**: result image after  **wood\_rabbit\_1024\_1068\_nv12.yuv**  is encoded.
        -   **dvpp\_vpc\_4000x4000\_nv12.yuv**: result image after  **dvpp\_vpc\_8192x8192\_nv12.yuv **is resized.



## Build and Run \(Atlas 200 DK\)<a name="section1112951551211"></a>

1.  Convert your model.
    1.  Log in to the  development environment  as the running user.
    2.  Set environment variables.

        Replace  _**$\{install\_path\}**_  with the  Ascend-CANN-Toolkit  installation path.

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  Prepare data.

        Download the .prototxt model file and .caffemodel pre-trained model file of the ResNet-50 network and upload the files to  **/caffe\_model**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

        [https://github.com/Ascend-Huawei/models/tree/master/computer\_vision/classification/resnet50](https://github.com/Ascend-Huawei/models/tree/master/computer_vision/classification/resnet50)

        Find the download links in the  **README\_en.md**  file.

    4.  Convert the ResNet-50 network into an offline model \(.om file\) that adapts to Ascend AI Processors. During model conversion, you need to set CSC parameters to convert YUV420SP images to RGB images.

        Go to the sample directory and run the following command:

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --soc_version=${soc_version} --insert_op_conf=caffe_model/aipp.cfg --output=model/resnet50_aipp 
        ```

        -   **--model**: directory of the source model file.
        -   **--weight**: directory of the weight file.
        -   **--framework**: source framework type, selected from  **0**  \(Caffe\),  **1**  \(MindSpore\),  **3**  \(TensorFlow\), and  **5**  \(ONNX\).
        -   **--soc\_version**: SoC version, either  **Ascend310**  or  **Ascend710**.
        -   **--insert\_op\_conf**: path of the configuration file for inserting the AI Pre-Processing \(AIPP\) operator for AI Core–based image preprocessing including image resizing, CSC, and mean subtraction and factor multiplication \(for pixel changing\), prior to model inference.
        -   **--output**: directory for storing the generated  **resnet50\_aipp.om**  file, that is,  **/model**  under the sample directory. The default path in the command example is recommended. To specify another path, you need to change the value of  **omModelPath**  in  **sample\_process.cpp**  before building the code.

            ```
            const char* omModelPath = "../model/resnet50_aipp.om";
            ```



2.  Build the code.
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


3.  Prepare input images.

    Obtain the input images of the sample from the following link and upload the obtained images to  **/data**  under the sample directory in the  development environment  as the running user. If the directory does not exist, create it.

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp\_vpc\_8192x8192\_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_vpc_8192x8192_nv12.yuv)

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/persian\_cat\_1024\_1536\_283.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/persian_cat_1024_1536_283.jpg)

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood\_rabbit\_1024\_1061\_330.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1061_330.jpg)

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood\_rabbit\_1024\_1068\_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1068_nv12.yuv)

4.  Run the app.
    1.  As the running user, upload the sample folder in the  development environment  to the  board environment, for example,  **$HOME/acl\_vpc\_jpege\_resnet50**.
    2.  Log in to the  board environment  as the running user.
    3.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_jpege\_resnet50/out**\) and grant execute permission on the  **main**  file in the directory.

        ```
        chmod +x main
        ```

    4.  Go to the directory where the executable file  **main**  is located \(for example,  **$HOME/acl\_vpc\_jpege\_resnet50/out**\) and run the executable file.

        1.  Decode two .jpg images into two YUV420SP \(NV12\) images, resize them, and perform model inference to obtain the inference results of the two images.

            ```
            ./main 0
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcResize
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[283], top1[0.500488], top5[0.863968]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcResize
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[330], top1[0.542480], top5[1.0000063]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        2.  Decode two .jpg images into two YUV420SP \(NV12\) images, crop selected ROIs from the images, and perform model inference to obtain the inference results of the two images.

            ```
            ./main 1
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcCrop
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[284], top1[0.961914], top5[0.999743]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcCrop
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[330], top1[0.631836], top5[0.998885]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        3.  Decode two .jpg images into two YUV420SP \(NV12\) images, crop selected ROIs from the images and paste each cropped image in the canvas, and perform model inference to obtain the inference results of the two images.

            ```
            ./main 2
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcCropAndPaste
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[284], top1[0.483398], top5[0.855194]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcCropAndPaste
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[331], top1[0.670898], top5[0.963564]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        4.  Encodes a YUV420SP image into a .jpg image.

            ```
            ./main 3
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] start to process picture:../data/wood_rabbit_1024_1068_nv12.yuv
            [INFO] call JpegE
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        5.  Resize the 8192 x 8192 image in YUV420SP format to 4000 x 4000.

            ```
            ./main 4
            ```

            The command output similar to the following is displayed.

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] get run mode success
            [INFO] dvpp process 8k resize begin
            [INFO] dvpp init resource success
            [INFO] dvpp process 8k resize success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            [INFO] end to finalize acl
            ```


        After the executable file is executed successfully, a result file is generated in the  **result**  directory at the same level as the  **main**  file for later query. The result file includes the following:

        -   **dvpp\_output\_0**: output image generated after  **persian\_cat\_1024\_1536\_283.jpg**  is resized, cropped, or cropped and pasted.
        -   **dvpp\_output\_1**: output image generated after  **wood\_rabbit\_1024\_1061\_330.jpg**  is resized, cropped, or cropped and pasted.
        -   **model\_output\_0**: model inference result \(a binary file\) of  **persian\_cat\_1024\_1536\_283.jpg**.
        -   **model\_output\_0.txt**: model inference result \(a .txt file\) of  **persian\_cat\_1024\_1536\_283.jpg**.
        -   **model\_output\_1**: model inference result \(a binary file\) of  **wood\_rabbit\_1024\_1061\_330.jpg**.
        -   **model\_output\_1.txt**: model inference result \(a .txt file\) of  **wood\_rabbit\_1024\_1061\_330.jpg**.
        -   **jpege\_output\_0.jpg**: result image after  **wood\_rabbit\_1024\_1068\_nv12.yuv**  is encoded.
        -   **dvpp\_vpc\_4000x4000\_nv12.yuv**: result image after  **dvpp\_vpc\_8192x8192\_nv12.yuv **is resized.



