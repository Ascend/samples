[中文](README.md) | English

# AclLite Library
## Instructions for Use

The AclLite library encapsulates, from samples in the open source community, the repeated code of:

1. Atlas 200 DK onboard camera

2. Acl DVPP image and video processing

3. Acl device resource and model inference interface

It provides a group of simple public interfaces for users.

Note:

1. This library is used only for open source samples in the current community. It does not cover all application development scenarios of the Ascend platform and cannot be used as a standard library for user application development.

2. This library is verified only on the Atlas 200 DK and Atlas 300 (x86) servers.

3. The public library is for reference only. You can custom or rebuild it based on the official interface documents and existing code to adapt it to your use habits and service performance requirements.

## Methods of Build

#### Dependencies

| Adaptation Item| Adaptation Condition| Remarks|
|---|---|---|
| Applicable version| CANN>=5.0.4 | Download the software package of the corresponding version at [CANN Community Edition](https://www.hiascend.com/en/software/cann/community).|
| Device form| Atlas 200 DK/Atlas 300 (ai1s)  | The library has passed the tests on Atlas 200 DK and Atlas 300. For details about the product description, see [Hardware](https://www.hiascend.com/en/hardware/product).|
| Third-party dependency| presentagent, ffmpeg | For details, see [Third-Party Dependency Installation Guide (C++ Sample)](../../environment/README.md).|

#### Procedure

1. Obtain the source package.

   You can use either of the following methods to download the package:  
    - Using CLI
       ```    
       # In the development environment, run the following commands as a non-root user to download the source repository:   
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```   
    - Downloading the compressed package  
       ``` 
        # 1. Click **Clone or Download** in the upper right corner of the samples repository and click **Download ZIP**.   
        # 2. Upload the ZIP package to the home directory of a common user in the development environment, for example, `${HOME}/ascend-samples-master.zip`.    
        # 3. In the development environment, run the following commands to unzip the package:    
        cd ${HOME}    
        unzip ascend-samples-master.zip
        ```

2. Access the **acllite** directory.
```
cd ${HOME}/samples/cplusplus/common/acllite
```
3. Run the commands for make and installation.
```
make 
make install
```
4. Confirm the installation.

  If the installation is complete, the **libacllite.so** file generated after compilation is copied to `${THIRDPART_PATH}/lib`, and the header file is copied to `${THIRDPART_PATH}/include/acllite`. 


## Deployment Procedure

1. When the Ascend AI device is used as both the development environment and running environment:

   No additional deployment is required.

2. If the development environment is installed on a non-Ascend AI device:

    - Copy **libacllite.so** to the `${THIRDPART_PATH}/lib` directory in the running environment.
    - In the running environment, switch to the root user, open `/etc/ld.so.conf.d/mind_so.conf`, add `${THIRDPART_PATH}/lib` to the end of the file, save the file, and exit. Then run the `ldconfig` command.

Note: The development environment refers to the environment for compiling application code. The running environment refers to the Ascend AI device that runs the application. The development environment and running environment can be on the same hardware device or different hardware devices.

## Interface Design and Description

The AclLite public library is designed based on the object-oriented principle and consists of the following functional modules:
   
   1. Resource management module, which initializes and releases ACL inference resources.

   2. Model inference module, which initializes model resources, creates input and output, invokes inference, and releases resources.

   3. Image processing module, which processes image data.

   4. Video processing module, which processes video data.

   5. Application and multi-thread module. For a simple multi-thread sample, a set of application-thread management class-thread class interfaces in single-instance mode are provided to simplify multi-thread sample development.

   6. Other files, including the tool function files, encapsulated structure files, and error code files in the AclLite library.

The following describes the interface functions and restrictions by module. Only the interfaces that are directly invoked by users are described.

### 1. Resource Management Module

|  Item | Description|
|---|---|
| Function| AclLiteResource() |
| Feature| Constructor. Creates an AclLiteResource object.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteResource(int32_t devId, const std::string& aclConfigPath, bool useDefaultCtx = true) |
| Feature| Constructor. Creates an AclLiteResource object and specifies the device, config file, and the context of threads.| 
| Parameter| devId: device ID.<br>aclConfigPath: path of the config file.<br>useDefaultCtx: whether to use the context of the current thread.|
| Return value| None|

| Item| Description|
|---|---|
| Function| ~AclLiteResource() |
| Feature| Destructor. Destructs the AclLiteResource object.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteError Init() |
| Feature| Initialization function. Initializes Acl related device and context.| 
| Parameter| None|
| Return value| ACLLITE_OK: initialization success.<br>Other values: initialization failure.|

| Item| Description|
|---|---|
| Function| void Release() |
| Feature| Resource release function. Releases Acl related resources, such as the device and context.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| aclrtRunMode GetRunMode() |
| Feature| Obtains the device running mode.| 
| Parameter| None|
| Return value| ACL_DEVICE: The Ascend AI software stack runs in the control CPU or board environment of the device.<br> ACL_HOST: The Ascend AI software stack runs on the host.|

| Item| Description|
|---|---|
| Function| aclrtContext GetContext() |
| Feature| Obtains the running context of the program.| 
| Parameter| None|
| Return value| Context<br>nullptr: invalid context.<br>Non-nullptr: valid context.|

### 2. Model Inference Module
The interface prototype of this module is defined in the **AclLiteModel.h** file, which is used to initialize and release Acl inference resources.

#### 2.1 AclLiteModel Class

| Item| Description|
|---|---|
| Function| AclLiteModel() |
| Feature| Constructor. Creates an AclLiteModel object.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteModel(const std::string& modelPath) |
| Feature| Constructor. Creates an AclLiteModel class object and provides the path of the model file to be loaded.| 
| Parameter| modelPath: offline model path.|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteModel(void *modelAddr, size_t modelSize) |
| Feature| Constructor. Creates an AclLiteModel class object and provides the memory address and size of the model file to be loaded.| 
| Parameter| modelAddr: memory address of the offline model file<br>modelSize: memory size of the offline model file|
| Return value| None|

| Item| Description|
|---|---|
| Function| ~AclLiteModel() |
| Feature| Destructor. Destructs an AclLiteModel class object.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteError Init() |
| Feature| Initialization function. You need to provide the model file path and memory address of the model file.| 
| Parameter| None|
| Return value| ACLLITE_OK: initialization success.<br>Other values: initialization failure.|

| Item| Description|
|---|---|
| Function| AclLiteError Init(const std::string& modelPath) |
| Feature| Initialization function. The Init() function is called after the path of the model file to be loaded is provided.| 
| Parameter| modelPath: offline model path.|
| Return value| ACLLITE_OK: initialization success.<br>Other values: initialization failure.|

| Item| Description|
|---|---|
| Function| AclLiteError Init(void *modelAddr, size_t modelSize) |
| Feature|Initialization function. Provides the memory address and size of the model file to be loaded.| 
| Parameter| modelAddr: memory address of the offline model file<br>modelSize: memory size of the offline model file|
| Return value| ACLLITE_OK: initialization success.<br>Other values: initialization failure.|

| Item| Description|
|---|---|
| Function| void DestroyResource() |
| Feature| Resource release function. Releases the resources related to the AclLiteModel class object, such as the model input, output, and desc data.| 
| Parameter| None|
| Return value| None|
| Remarks| isReleased_ is used to prevent multiple resource release errors.|

| Item| Description|
|---|---|
| Function| AclLiteError CreateInput(void *input, uint32_t inputsize) |
| Feature| Creates model input (scenario: model with one input).| 
| Parameter| input: model input data<br>input1size: size of the model input data|
| Constraint| Data must be stored in the device or DVPP memory.|
| Return value| ACLLITE_OK: creation success.<br>Other values: creation failure.|

| Item| Description|
|---|---|
| Function| AclLiteError CreateInput(void *input1, uint32_t input1size, void* input2, uint32_t input2size) |
| Feature| Creates model input (scenario: model with two inputs).| 
| Parameter| input1: first input data of the model<br>input1size: size of the first input data of the model<br>input2: second input data of the model<br>input2size: size of the second input data of the model|
| Constraint| Data must be stored in the device or DVPP memory.|
| Return value| ACLLITE_OK: creation success.<br>Other values: creation failure.|

| Item| Description|
|---|---|
| Function| AclLiteError CreateInput(std::vector<DataInfo>& inputData) |
| Feature| Creates model input (scenario: model with multiple inputs).| 
| Parameter| inputData: model input data vector|
| Return value| ACLLITE_OK: creation success.<br>Other values: creation failure.|
| Constraint| Data must be stored in the device or DVPP memory.|
| Remarks| For details about the DataInfo data structure, see [**DataInfo**](#DataInfo).|

| Item| Description|
|---|---|
| Function| AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs, void *data, uint32_t size) |
| Feature| Executes model inference. This interface applies to the scenario where the model has only one input. The second and third parameters are used to construct the model input and then the input is sent for inference.| 
| Parameter| inferOutputs: model inference result<br>data: model input data<br>size: size of the model input data|
| Return value| ACLLITE_OK: inference success.<br>Other values: inference failure.|
| Remarks| The inference result data is stored locally. If the model is connected or the DVPP function is used, the data needs to be copied.<br>For details about the InferenceOutput data structure, see [**InferenceOutput**](#InferenceOutput).|

| Item| Description|
|---|---|
| Function| AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs) |
| Feature| Performs model inference.| 
| Parameter| inferOutputs: model inference result.|
| Return value| ACLLITE_OK: inference success.<br>Other values: inference failure.|
| Remarks| The inference result data is stored locally. If the model is connected or the DVPP function is used, the data needs to be copied.<br>For details about the InferenceOutput data structure, see [**InferenceOutput**](#InferenceOutput).|

| Item| Description|
|---|---|
| Function| size_t GetModelInputSize(int index) |
| Feature| Obtains the input data size of a model.| 
| Parameter| index: sequence number of the input of the model. The index starts from 0.|
| Return value| Model input data size<br>> 0: success<br>Other values: failure|
| Remarks| The size is measured in bytes.|

| Item| Description|
|---|---|
| Function| void DestroyInput() |
| Feature| Destroys the model input.| 
| Parameter| None|
| Return value| None|
| Remarks| Only the dataset structure created by CreateInput() is released. The input data is not released.|

| Item| Description|
|---|---|
| Function| AclLiteError LoadModelFromFile(const std::string& modelPath) |
| Feature| Loads a model from a file.| 
| Parameter| modelPath: offline model path.|
| Return value| ACLLITE_OK: Loaded successfully.<br>Other values: Failed to load.|

| Item| Description|
|---|---|
| Function| AclLiteError LoadModelFromMem() |
| Feature| Loads a model from the memory.| 
| Parameter| None|
| Return value| ACLLITE_OK: Loaded successfully.<br>Other values: Failed to load.|

| Item| Description|
|---|---|
| Function| AclLiteError SetDesc() |
| Feature| Creates and sets model description.| 
| Parameter| None|
| Return value| ACLLITE_OK: The setting is successful.<br>Other values: The setting fails.|

| Item| Description|
|---|---|
| Function| AclLiteError CreateOutput() |
| Feature| Creates the model output.| 
| Parameter| None|
| Return value| ACLLITE_OK: Creation success.<br>Other values: Creation failure.|

| Item| Description|
|---|---|
| Function| AclLiteError AddDatasetBuffer(aclmdlDataset* dataset, void* buffer, uint32_t bufferSize) |
| Feature| Creates a data buffer and adds it to the dataset.| 
| Parameter| dataset: Dataset of the data buffer to be added.<br>buffer: Data for creating the data buffer.<br>bufferSize: Data size.|
| Return value| ACLLITE_OK: Added successfully.<br>Other values: Failed to be added.|

| Item| Description|
|---|---|
| Function| AclLiteError GetOutputItem(InferenceOutput& out, uint32_t idx) |
| Feature| Obtains the inference result by index and copy it to the local host.| 
| Parameter| out: Inference result copied to the local host.<br>idx: Index.|
| Return value| ACLLITE_OK: Copied successfully.<br>Other values: Failed to copy.|

| Item| Description|
|---|---|
| Function| void Unload() |
| Feature| Unloads a loaded model. If the model is stored in the memory, the memory is released.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| void DestroyDesc() |
| Feature| Destroys the model description.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| void DestroyOutput() |
| Feature| Destroys the model output.| 
| Parameter| None|
| Return value| None|

### 3. Image Processing Module

#### 3.1 AclLiteImageProc Class
The interface prototype of this module is defined in the **AclLiteImageProc.h** file and is used to process image data.

| Item| Description|
|---|---|
| Function| AclLiteImageProc() |
| Feature| Constructor that creates an AclLiteImageProc object to process image data.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| ~AclLiteImageProc() |
| Feature| Destructor function for destroying the AclLiteImageProc object| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteError Init() |
| Feature| Initializes the AclLiteImageProc object and specify the stream and channel required for using the DVPP function.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteError Resize(ImageData& dest,ImageData& src, uint32_t width, uint32_t height) |
| Feature| Resizes an image to a specified size.| 
| Parameter| dest: Compressed image.<br>src: Image to be compressed.<br>width: Target width of scaling.<br>height: Target height of scaling.|
| Return value| ACLLITE_OK: Scaling succeeded.<br>Other values: Failed to scale.|
| Constraints| For details about the constraints of this interface on data input and output, see [VPC Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0156.html) in [Ascend Community Documentation Center](https://www.hiascend.com/document?tag=community-developer). Select the correct CANN version.<br>|
| Remarks| acllite resize() internally encapsulates the alignment operation and uses the 16 x 2 alignment parameter. However, alignment may cause the size of the output scaled image to be inconsistent with the interface parameter.<br>For details about the ImageData data structure, see [**ImageData**](#ImageData).|

| Item| Description|
|---|---|
| Function| AclLiteError JpegD(ImageData& destYuv, ImageData& srcJpeg) |
| Feature| Decodes JPEG images into YUV images.| 
| Parameter| destYuv: Decoded YUV image.<br>srcJpeg: JPEG image to be decoded.|
| Return value| ACLLITE_OK: Decoding succeeded.<br>Other values: Failed to decode.|
| Constraints| For details about the constraints of this interface on data input and output, see [JPEGD Functions and Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0174.html) in [Ascend Community Documentation Center](https://www.hiascend.com/document?tag=community-developer). Select the correct CANN version.|
| Remarks| acllite JpegD() encapsulates the alignment operation internally. The current alignment parameter is 128 x 16. However, alignment may cause the width and height of the output decoded image to be different from those of the original image.<br>For details about the ImageData data structure, see [**ImageData**](#ImageData).|

| Item| Description|
|---|---|
| Function| AclLiteError JpegE(ImageData& destJpeg, ImageData& srcYuv) |
| Feature| Encodes a YUV image into a JPEG image.| 
| Parameter| destJpeg: Encoded JPEG image.<br>srcYuv: YUV image to be encoded.|
| Return value| ACLLITE_OK: Encoding succeeded.<br>Other values: Failed to encode.|
| Constraints| For details about the constraints of this interface on data input and output, see [JPEGE Functions and Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0181.html) in [Ascend Community Documentation Center](https://www.hiascend.com/document?tag=community-developer). Select the correct CANN version.|
| Remarks| acllite JpegE() internally encapsulates the alignment operation. The current alignment parameter is 16x2. However, alignment may cause the width and height of the output encoded image to be different from those of the original image.<br>For details about the ImageData data structure, see [**ImageData**](#ImageData).|

| Item| Description|
|---|---|
| Function| AclLiteError Crop(ImageData& dest, ImageData& src, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert) |
| Feature| Crops a rectangular area determined by (ltHorz, ltVert) and (rbHorz, rbVert) from the original image, and paste the area to the overridden area (0, 0) (rbHorz-ltHorz, ltVert-rbVert).| 
| Parameter| dest: Cropped image data.<br>src: Image to be processed.<br>ltHorz: X coordinate of the upper left point.<br>ltVert: Y coordinate of the upper left point.<br>rbHorz: X coordinate of the lower right point<br>rbVert: Y coordinate of the lower right point|
| Return value| ACLLITE_OK: Processing succeeded.<br>Other values: Failed to process.|
| Constraints| For details about the constraints of this interface on data input and output, see [VPC Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0156.html) in [Ascend Community Documentation Center](https://www.hiascend.com/document?tag=community-developer). Select the correct CANN version.<br>|
| Remarks| acllite Crop() encapsulates the alignment operation internally. It automatically processes the width, height, and coordinate offset of the input image to meet the restrictions on the VPC function.<br>For details about the ImageData data structure, see [**ImageData**](#ImageData).|

| Item| Description|
|---|---|
| Function| AclLiteError CropPaste(ImageData& dest, ImageData& src, uint32_t width, uint32_t height, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert) |
| Feature| Crops a rectangular area determined by (ltHorz, ltVert) or (rbHorz, rbVert) from the original image, and pastes the area to the overwritten area ((0, 0) (width, height).| 
| Parameter| dest: Cropped image data.<br>src: Image to be processed.<br>width: Width of the pasted image.<br>Height of the image after pasting<br>ltHorz: X coordinate of the upper left point of the cropped area.<br>ltVert: Y coordinate of the upper left point of the cropped area.<br>rbHorz: X coordinate of the lower right point of the cropped area.<br>rbVert: Y coordinate of the lower right point of the cropped area.|
| Return value| ACLLITE_OK: Processing succeeded.<br>Other values: Failed to process.|
| Constraints| For details about the constraints of this interface on data input and output, see [VPC Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0156.html) in [Ascend Community Documentation Center](https://www.hiascend.com/document?tag=community-developer). Select the correct CANN version.<br>|
| Remarks| After the image data is processed by this API, the width, height, and coordinate offset of the input image are automatically processed to meet the VPC function restrictions.<br>For details about the ImageData data structure, see [**ImageData**](#ImageData).|

| Item| Description|
|---|---|
| Function| AclLiteError ProportionPaste(ImageData& dest, ImageData& src, uint32_t ltHorz, uint32_t ltVert, uint32_t rbHorz, uint32_t rbVert); |
| Feature| Pastes the original image data to the area (0, 0) (rbHorz-ltHorz, ltVert-rbVert) without changing the aspect ratio.| 
| Parameter| dest: Cropped image data.<br>src: Image to be processed.<br>ltHorz: X coordinate of the upper left point.<br>ltVert: Y coordinate of the upper left point.<br>rbHorz: X coordinate of the lower right point.<br>rbVert: Y coordinate of the lower right point.|
| Return value| ACLLITE_OK: Processing succeeded.<br>Other values: Failed to process.|
| Constraints| For details about the constraints of this interface on data input and output, see [VPC Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0156.html) in [Ascend Community Documentation Center](https://www.hiascend.com/document?tag=community-developer). Select the correct CANN version.<br>|
| Remarks| After the image data is processed by this API, the width, height, and coordinate offset of the input image are automatically processed to meet the VPC function restrictions. Therefore, green edges may be generated. You can use the AIPP function to eliminate the green edges.<br>The blank area of the overridden area is filled with green edges after the operation. For example, if the original area is (0,0) (200,100) and the overridden area is (0,0) (50,50), the (0, 0) (50, 25) area is valid image data, (0, 25) (50, 50) is filled with green edges. In this example, the DVPP alignment constraint is ignored. The upper left corner is the origin, and the direction from the X axis to the right and from the Y axis to the bottom is positive.<br>For details about the ImageData data structure, see [**ImageData**](#ImageData).|

| Item| Description|
|---|---|
| Function| void DestroyResource() |
| Feature| Destroys the resources related to the AclLiteImageProc class object.| 
| Parameter| None|
| Return value| None|

### 4. Video Processing Module

This module processes video data.

#### 4.1 AclLiteVideoProc Class

This class is used to decode Atlas 200 DK onboard cameras, RTSP video streams, MP4 files, and H.264/H.265 raw stream files, and encode YUV images. The prototype of this class is defined in the **AclLiteVideoProc.h** file.

| Item| Description|
|---|---|
| Function| AclLiteVideoProc() |
| Feature| Constructor. Generates a camera instance. If the camera in slot 0 is available, slot 0 is selected. Otherwise, slot 1 is selected. If both cameras are unavailable, only an instance is generated and no camera is enabled.| 
| Parameter| None. However, after the function is enabled, the resolution parameters are specified as follows: The width is 1280, the height is 720, and the frame rate is 15.|
| Return value| None|
| Constraints| 1. Only Atlas 200 DK devices are supported.<br>2. If the Raspberry Pi v2.1 camera is used, the camera frame rate (FPS) ranges from 1 to 20.<br>3. If the Raspberry Pi v1.3 camera is used, the camera frame rate (FPS) ranges from 1 to 15.<br>4. The default resolution of the camera must meet the driver requirements. Currently, the following five resolutions are supported: 1920 x 1080, 1280 x 720, 704 x 576, 704 x 288 and 352 x 288.|

| Item| Description|
|---|---|
| Function| AclLiteVideoProc(uint32_t cameraId, uint32_t width = 1280, uint32_t height = 720, uint32_t fps = 15) |
| Feature| Constructor. Generates a camera instance. If the camera is unavailable, only an instance is generated and the camera is not enabled.| 
| Parameter| cameraId: Camera ID. **0** indicates the camera in slot CAMERA0, and **1** indicates the camera in slot CAMERA1.<br>width: Width of the camera resolution.<br>height: Height of the camera resolution.<br>fps: Camera frame rate.|
| Return value| None|
| Constraints| 1. Only Atlas 200 DK devices are supported.<br>2. If the Raspberry Pi v2.1 camera is used, the camera frame rate (FPS) ranges from 1 to 20.<br>3. If the Raspberry Pi v1.3 camera is used, the camera frame rate (FPS) ranges from 1 to 15.<br>4. The default resolution of the camera must meet the driver requirements. Currently, the following five resolutions are supported: 1920 x 1080, 1280 x 720, 704 x 576, 704 x 288 and 352 x 288.|

| Item| Description|
|---|---|
| Function| AclLiteVideoProc(const string& videoPath, aclrtContext context) |
| Feature| Constructor. Creates a video or RTSP stream instance to be decoded.| 
| Parameter| videoPath: Address of the video file or RTSP stream to be decoded.<br>context: ACL context used when the decoder uses the DVPP VDEC function for decoding. If this parameter is optional, the input parameter is considered as nullptr, and the context of the current thread is used for decoding.|
| Return value| None|
| Constraints| 1. For details, see [VDEC Functions and Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0192.html) in the application development manual in [Ascend Community Documentation Center](https://www.hiascend.com/en/document?tag=community-developer). Select the correct CANN version.<br>2. Before creating an instance, you need to initialize acl (aclInit) and set device (aclrtSetDevice).|

| Item| Description|
|---|---|
| Function| AclLiteVideoProc(VencConfig& vencConfig, aclrtContext context) |
| Feature|  Constructor. Creates a video instance to be encoded.| 
| Parameter| vencConfig: Encoding configuration file, VencConfig type structure. For details about the structure, see [**VencConfig**](#VencConfig).<br>context: ACL context used when the decoder uses the DVPP VENC function for encoding. If this parameter is left empty, the input parameter is considered as nullptr and the context of the current thread is used for decoding.|
| Return value| None|
| Constraints| 1. For details, see [VENC Functions and Restrictions](https://www.hiascend.com/document/detail/en/CANNCommunityEdition/51RC1alphaX/infacldevg/aclcppdevg/aclcppdevg_03_0198.html) in the application development manual in [Ascend Community Documentation Center](https://www.hiascend.com/en/document?tag=community-developer). Select the correct CANN version.<br>2. Before creating an instance, you need to initialize acl (aclInit) and set device (aclrtSetDevice).|

| Item| Description|
|---|---|
| Description| ~AclLiteVideoProc() |
| Feature|  Destructor.| 
| Parameter| None|
| Return value| None|
| Constraints| None|

| Item| Description|
|---|---|
| Description| AclLiteError Open() |
| Feature| Opens camera or video stream.| 
| Parameter| None|
| Return value| ACLLITE_OK: Opened successfully.<br>Non-ACLLITE_OK: Failed to open.|

| Item| Description|
|---|---|
| Function| bool IsOpened() |
| Feature| Checks whether the camera or video stream is enabled.| 
| Parameter| None|
| Return value| true: The camera is enabled or video streams can be decoded.<br>false: The camera is unavailable or video streams cannot be decoded.|

| Item| Description|
|---|---|
| Function| uint32_t Get(StreamProperty key) |
| Feature| Obtains the actual value of an attribute based on the key value.| 
| Parameter| key: StreamProperty of the enumerated type. For details, see [**AclLiteVideoCapBase Class**](#AclLiteVideoCapBase Class).|
| Return value| Attribute value|
| Constraints| Determine whether to support the obtaining of the attribute corresponding to the key value based on the get() of the object generated by each constructor.|

| Item| Description|
|---|---|
| Function| AclLiteError Set(StreamProperty key, uint32_t value) |
| Feature| Sets the actual value of the corresponding attribute based on the key value.| 
| Parameter| key: StreamProperty of the enumerated type. For details, see [**AclLiteVideoCapBase Class**](#AclLiteVideoCapBase Class).<br> value: Attribute value.|
| Return value| ACLLITE_OK: Set successfully.<br>Non-ACLLITE_OK: Failed to set.|
| Constraints| Determine whether to support the setting of the attribute corresponding to the key value based on the get() of the object generated by each constructor.|

| Item| Description|
|---|---|
| Function| AclLiteError Read(ImageData& frame) |
| Feature| Acquires a frame of image data to be processed.<br>There are three scenarios:<br>1. A frame of data read from a camera.<br>2. A frame of data read from a video file or RTSP stream.<br>3. One frame of data processed by the DVPP VENC.| 
| Parameter| frame: Input image data and attribute, ImageData structure data. For details about the structure, see [**ImageData**](#ImageData).|
| **Return Value**| ACLLITE_OK: Read successfully.<br>Non-ACLLITE_OK: Failed to read.|
| Constraints| The memory for storing the obtained data is the DVPP memory. The memory cannot be transferred between different contexts. Therefore, the context transferred when a decoder is created must be the same as the context of the thread for calling the read interface. Otherwise, the image data is unavailable.|
| Remarks| During pre-processing and post-processing, check whether data needs to be copied between the DVPP, device, and host.|

| Item| Item|
|---|---|
| Function| AclLiteError Close() |
| Feature| Stops reading image data.| 
| Parameter| None|
| Return value| ACLLITE_OK: Disabled or stopped successfully.<br>Non-ACLLITE_OK: Failed to disable or stop.|

<a name="AclLiteVideoCapBase Class"></a>
#### 4.2 AclLiteVideoCapBase Class

The prototype of this class is defined in the **AclLiteVideoCapBase.h** file. By inheriting this class, the public library derives the CameraCapture, VideoCapture, and VideoWriter classes and specifies their functions. In this way, the three classes can be integrated and invoked by AclLiteVideoProc and provided for users.
CameraCapture class: Reads image frames from cameras.
VideoCapture class: Reads image frames from videos.
VideoWriter class: Writes image frames into video files.

You can inherit this class for secondary development to customize and add functions of the video processing module based on actual service scenarios. Even the public library is more useful. You can refer to the AclLite idea to sort out and encapsulate the Ascend CL APIs based on your usage habits and required functions.

| Item| Description|
|---|---|
| Enumeration type| StreamProperty |
| Feature| Labels attributes that can be set.| 
| Parameter|  FRAME_WIDTH = 1 // Image width.<br>FRAME_HEIGHT = 2 // Image height.<br>VIDEO_FPS = 3 // Decoding frame rate.<br>OUTPUT_IMAGE_FORMAT = 4 // Output image format.<br>RTSP_TRANSPORT = 5 // RTSP stream transmission protocol (UDP/TCP).<br>STREAM_FORMAT = 6 // Stream format (H265 MP/H264 BP/MP/HP).|

| Item| Description|
|---|---|
| Function| AclLiteVideoCapBase() |
| Feature| Constructor. You need to implement it in the inheritance class.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| virtual ~AclLiteVideoCapBase() |
| Feature| Destructor. You need to implement it in the inheritance class.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| virtual bool IsOpened() |
| Feature| This is a pure virtual function. You need to implement it in the inheritance class. This function is used to check whether the status of data sources such as videos and cameras is normal.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| virtual AclLiteError Set(StreamProperty key, uint32_t value) |
| Feature| This is a pure virtual function and has no implementation. You need to implement it in the inheritance class. This function is used to provide users with an interface for setting the value of a specific attribute during data processing.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| virtual uint32_t Get(StreamProperty key) |
| Feature| This is a pure virtual function and has no implementation. You need to implement it in the inheritance class. This function is used to provide users with an interface for obtaining the value of a specific attribute during data processing.| 
| Parameter.| None|
| **Return Value**| None|

| Item| Description|
|---|---|
| Function| virtual AclLiteError Read(ImageData& frame) |
| Feature| Pure virtual function, which is not implemented. You need to implement it in the inheritance class. This function is used to provide an interface for users to read data.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| virtual AclLiteError Close() |
| Feature| It is a pure virtual function and has no implementation. You need to implement it in the inheritance class. This function is used to correctly release resources related to data sources such as videos and cameras.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| virtual AclLiteError Open() |
| Feature| It is a pure virtual function and has no implementation. You need to implement it in the inheritance class. This function is used to initialize the data sources such as the video and camera for reading.| 
| Parameter| None|
| Return value| None|

### 5. Application and Multi-thread Module

#### 5.1 AclLiteApp Class

The AclLiteApp class is used to create and manage application threads. This class is a class in single-instance mode. The prototype of this class is defined in the **AclLiteApp.h** file.

| Item| Item|
|---|---|
| Function| AclLiteApp()<br>AclLiteApp(const AclLiteApp&) = delete<br>AclLiteApp& operator=(const AclLiteApp&) = delete; |
| Feature| Constructor.| 
| Parameter| None|
| Return value| None|
| Remarks| The AclLiteApp class is a single-instance class. Therefore, when creating an instance in the application, use the GetInstance method to obtain the unique instance of the class instead of directly using the constructor function to create an instance. In addition, the instance cannot be copied.|

| Item| Description|
|---|---|
| Function| ~AclLiteApp() |
| Feature| Destructor. Releases all threads of an application.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| static AclLiteApp& GetInstance() |
| Feature| Obtains the globally unique AclLiteApp instance.| 
| Parameter| None|
| Return value| AclLiteApp instance|

| Item| Description|
|---|---|
| Function| AclLiteError Init() |
| Feature| Initializes the AclLiteApp instance and creates the first thread. The thread ID is 0. This thread is used to process public tasks of the entire application, for example, communication with other application processes and ending of the application.| 
| Parameter| None|
| Return value| ACLLITE_OK: Initialized successfully.<br>Non-ACLLITE_OK: Failed to initialize.|

| Item| Description|
|---|---|
| Function| int CreateAclLiteThread(AclLiteThread* thInst, const std::string& instName,aclrtContext context, aclrtRunMode runMode) |
| Feature| Creates an AclLite thread, executes the Init method of the service object instance provided by the user, and then waits for the service message cyclically. The thread is detached, which is a non-block API. Therefore, after this API is called, a cyclic wait API must be provided to wait for the thread to end.| 
| Parameter| thInst: Service thread instance of the application. You need to reload the Init method in the thread to initialize the thread and reload the Process method to process received messages.<br>instName: Thread name. The name must be globally unique. Otherwise, the creation fails. Users can invoke the GetAclLiteThreadIdByName interface to obtain the thread ID by using the thread name as a parameter.<br>context: ACL context of the thread.<br>runMode: Application run mode. The value can be ACL_DEVICE (the Ascend AI software stack runs on the control CPU or board of the device) or ACL_HOST (the Ascend AI software stack runs on the host).|
| Return value| Thread ID<br>-1: Failed to create.<br>>0: Created successfully.|

| Item| Description|
|---|---|
| Function| int CreateAclLiteThreadMgr(AclLiteThread* thInst, const string& instName, aclrtContext context, aclrtRunMode runMode) |
| Feature| This method is used to create the management structure corresponding to the AclLite thread and implement the following functions:<br>1. Check whether the user thread name is globally unique.<br>2. Set the context, run mode, name, and ID of the user thread so that the user thread can use the corresponding interface to access the information.<br>3. Add the thread management object to the AclLiteApp thread management table.| 
| Parameter| thInst: Service thread instance of the application. You need to reload the Init method in the thread to initialize the thread and reload the Process method to process received messages.<br>instName: Thread name. The name must be globally unique. Otherwise, the creation fails. Users can invoke the GetAclLiteThreadIdByName interface to obtain the thread ID by using the thread name as a parameter.<br>context: ACL context of the thread.<br>runMode: Application run mode. The value can be ACL_DEVICE (the Ascend AI software stack runs on the control CPU or board of the device) or ACL_HOST (the Ascend AI software stack runs on the host).|
| Return value| Thread ID<br>-1: Failed to create.<br>>0: Created successfully.|

| Item| Description|
|---|---|
| Function| bool CheckThreadNameUnique(const std::string& threadName) |
| Feature| Checks whether the thread name is globally unique.| 
| Parameter| threadName: Thread name.|
| Return value| true: The thread name is unique.<br>false: The thread name is not unique.|

| Item| Description|
|---|---|
| Function| bool CheckThreadAbnormal() |
| Feature| Checks whether abnormal threads exist.| 
| Parameter| None|
| Return value| true: Abnormal threads exist.<br>false: No abnormal threads.|

| Item| Description|
|---|---|
| Function| int GetAclLiteThreadIdByName(const std::string& threadName) |
| Feature| Obtains the thread ID based on the thread name.| 
| Parameter| threadName: Thread name.|
| Return value| Thread ID<br>-1: Invalid ID.<br>>0: ID corresponding to the thread name.|

| Item| Description|
|---|---|
| Function| int Start(vector<AclLiteThreadParam>& threadParamTbl) |
| Feature| Starts all threads in the thread table.| 
| Parameter| threadParamTbl: Thread table.|
| Return value| 0: Started successfully.<br>Non-0: Failed to create.|
| Remarks| 1. Currently, threads can be created in either of the following ways:<br>(1) Use the CreateAclLiteThread API to pass the corresponding parameters and start a thread.<br>(2) Construct an AclLiteThreadParam table and input this method to start all threads at a time.<br>2. The time sequence must be ensured during thread creation. For example, if an upstream thread is created and a message needs to be sent to a downstream thread, the ID of the downstream thread must be obtained. However, if the downstream thread is not created, no thread ID is allocated. As a result, the message cannot be sent. Therefore, the upstream thread can obtain the ID of the downstream thread only after the downstream thread is successfully created.<br>3. For details about the AclLiteThreadParam structure, see [**AclLiteThreadParam**]().|

| Item| Description|
|---|---|
| Function| void Wait(); |
| Feature| Blocks thread 0, the main thread. When the CreateAclLiteThread or start method is used to create service threads in the main process and each thread is notified to work, each thread is detached and the main thread is not blocked. In this case, if the main thread executes all functions, the entire application exits directly, and each thread cannot complete its own work. Therefore, a blocking point is required in the main thread to ensure that the main thread does not end and the entire application works all the time.| 
| Parameter| None|
| Return value| None|
| Remarks| This method uses the sleep infinite loop to wait. To end the application, press Ctrl+C or kill the application process.|

| Item| Description|
|---|---|
| Function| void Wait(AclLiteMsgProcess msgProcess, void* param) |
| Feature| When AclLiteApp is created, thread 0 (main thread) is started. The thread has a message queue. Because the thread is busy, it does not use the message queue to receive and process messages. You can define a message processing function and invoke the wait method. In this method, the message queue of thread 0 is polled. After receiving a message, the msgProcess method is invoked to process the message. If the processing fails and a non-0 value is returned, this function exits the polling (infinite loop).| 
| Parameter| msgProcess: User-defined message processing function entry.<br> param: Parameter passed to the message processing function.|
| Return value| None|
| Remarks| AclLiteMsgProcess is the message processing API of the main thread that needs to be implemented by the user. It is used to determine the received signal, exit the application, and release resources.<br>Currently, only the function pointer of this type is defined in **AclLiteApp.h** for reference.<br>typedef int (\*AclLiteMsgProcess)(uint32_t msgId, shared_ptr<void> msgData, void* userData); |

| Item| Description|
|---|---|
| Function| void WaitEnd() |
| Feature| Sets **waitEnd_** to true. The Wait interface breaks and exits the waiting state.| 
| Parameter| None|
| Return value| None|
| Remarks| waitEnd: Waits for termination.<br>When AclLiteApp is created, thread 0 (main thread) is started. This thread is in the busy state and continuously queries the value of **waitEnd_**.<br>When a user needs to end the entire app, this API can be called. The value of **waitEnd_** is set to true. When the main thread detects that the value of **waitEnd_** is true, it sends an exit signal to each thread and releases resources to end the app.|

| Item| Description|
|---|---|
| Function| AclLiteError SendMessage(int dest, int msgId, shared_ptr<void> data) |
| Feature| Sends messages to the target thread.| 
| Parameter| dest: ID of the destination thread, which is the thread management object corresponding to the thread and is a subscript in the thread management table **threadList_** of AtlasApp.<br>msgId: Message ID, which is defined by users in their applications.<br>data: Message data. The data type is defined by users.|
| Return value| ACLLITE_OK: Message sent successfully.<br>Non-ACLLITE_OK: Failed to send the message.|

| Item| Description|
|---|---|
| Function| void Exit() |
| Feature| Notifies all threads to end. After all threads end, ends the application.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| void ReleaseThreads() |
| Feature| Changes the thread status in the thread table to exit and release all thread resources.| 
| Parameter| None|
| Return value| None|

#### 5.2 AclLiteThread Class

The AclLiteThread class is an abstract class. It is the base class of user thread objects and provides public methods and properties of thread objects. When using the AclLite library to create a thread, you must overload this base class as the parent class. This base class provides two pure virtual functions Init() and Process(). The user thread object must implement the two functions. The prototype of this class is defined in the **AclLiteThread.h** file.

| Item| Description|
|---|---|
| Function| AclLiteThread() |
| Feature| Constructor. Creates an AclLiteThread class object.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| ~AclLiteThread() |
| Feature| Destructor. Destroys an AclLiteThread class object.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| virtual int Init() |
| Feature| This is a pure virtual function. You need to inherit the AclLiteThread class, define your own service thread object, and overload the function to implement the initialization required by your thread. If this parameter is not overloaded, ACLLITE_OK is returned by default.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|------|---|
| Function| virtual int Process(int msgId, shared_ptr<void> msgData) |
| Feature| Pure virtual function. Thread objects that inherit AclLiteThread must overload Process() to process received messages. After another thread sends a message to the current thread, the thread management object of the current thread obtains the message in the message polling of AclLiteThreadMgr::ThreadEntry, that is, the message is received. Then, the thread management object calls this method to pass the message ID and message data to this method for processing. The AclLiteThreadMgr::ThreadEntry method is not perceivable to users, so **you do not need to pay attention to message receiving and process calling**. You only need to focus on the implementation of the process.| 
| Parameter| None|
| Return value| None|
| Remarks| If the implemented method returns a failure message, the thread ends. Therefore, the return value must be considered carefully based on the overall service.|

| Item| Description|
|---|---|
| Function| int SelfInstanceId() |
| Feature| Returns the thread ID.| 
| Parameter| None|
| Return value| Thread ID<br>-1: Invalid ID.<br>>0: Valid ID.|
| Remarks| This interface can return a correct value only after a thread object is instantiated and a thread is created. After instantiation, there is no thread ID before a thread is created. The return value is INVALID_INSTANCE_ID(-1).|

| Item| Description|
|---|---|
| Function| string& SelfInstanceName() |
| Feature| Returns the thread name.| 
| Parameter| None|
| Return value| Thread name. Values are as follows:<br>Empty string: Invalid name.<br>Non-empty string: Valid name|
| Remarks| Thread-related interface, which is valid only after a thread is created.|

| Item| Description|
|---|---|
| Function| aclrtContext GetContext() |
| Feature| Returns the context when the thread is running.| 
| Parameter| None|
| Return value| context:<br>nullptr: Invalid context.<br>Non-nullptr: Valid context.|
| Remarks| Thread-related interface, which is valid only after a thread is created.|

| Item| Description|
|---|---|
| Function| aclrtRunMode GetRunMode() |
| Feature| Returns the runmode when the thread is running.| 
| Parameter| None|
| Return value| run mode:<br>ACL_DEVICE (The Ascend AI software stack runs in the control CPU or board environment of the device.)<br>ACL_HOST (The Ascend AI software stack runs on the host.)|
| Remarks| Thread-related interface, which is valid only after a thread is created.|

| Item| Description|
|---|---|
| Function| AclLiteError BaseConfig(int instanceId, const string& threadName, aclrtContext context, aclrtRunMode runMode) |
| Feature| Sets the thread ID, thread name, context, and run mode of the current thread.| 
| Parameter| instanceId: Thread ID.<br>const string& threadName: Thread name.<br>context: Thread context.<br>aclrtRunMode runMode: Thread run mode.|
| Return value| ACLLITE_OK: Set successfully.<br>Non-ACLLITE_OK: Failed to set.|
| Remarks| This function is expected to be called only once in AclLiteApp::CreateAclLiteThreadMgr. Do not call this function to modify these attributes during running.|

| Item| <a name="AclLiteThreadParam ">Description</a>|
|---|---|
| Structure| AclLiteThreadParam |
| Feature| Encapsulates attributes and running resources related to a thread object.| 
| Member|  threadInst: Pointer pointing to the AclLiteThread object.<br>threadInstName: Thread name, which is globally unique.<br>context: Context where the thread is running.<br> threadInstId: Thread ID, which is globally unique.|

#### 5.3 AclLiteThreadMgr Class
The prototype of this class is defined in the **AclLiteThreadMgr.h** file. The AclLiteThreadMgr class is used to configure an object for AclLiteThread to manage the interaction of message queues between threads. This ensures that service thread objects are logically isolated so that developers can focus on the implementation of service thread functions. You do not need to send and receive message data and maintain message queues in AclLiteThread.

| Item| Description|
|---|---|
| Function| AclLiteThreadMgr(AclLiteThread* userThreadInstance, const std::string& threadName) |
| Feature| Constructor. Generates the thread management object corresponding to a thread.| 
| Parameter| userThreadInstance: AclLiteThread thread instance.<br>threadName: Thread name.|
| Return value| None|
| Constraints| The thread name is globally unique.|

| Item| Description|
|---|---|
| Function| ~AclLiteThreadMgr() |
| Feature| Destructor.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| static void ThreadEntry(void* data) |
| Feature| Thread entry function created in the CreateThread method. When a thread is started, this function sets the context of the thread based on the context, run mode, and user thread object instance transferred during thread creation. It also calls the Init method of the user thread instance object, polls whether a message is received, and calls the Process function of the user thread instance after receiving the message. If the Process returns a failure (non-zero value), the polling stops and the thread ends.| 
| Parameter| arg: Thread creation parameter, which is the type pointer of the thread management object and is forcibly converted to the void* type.|
| Return value| None|

| Item| Description|
|---|---|
| Function| void CreateThread() |
| Feature| Calls the thread entry function to start the corresponding thread and detach the thread.| 
| Parameter| None|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteError WaitThreadInitEnd() |
| Feature| Checks whether the AclLiteThread thread instance is initialized successfully.| 
| Parameter| None|
| Return value| ACLLITE_OK: Initialized successfully.<br>Non-ACLLITE_OK: Failed to initialize.|

| Item| Description|
|---|---|
| Function| AclLiteError PushMsgToQueue(shared_ptr<AclLiteMessage>& pMessage) |
| Feature| Sends the message data to the message queue.| 
| Parameter| pMessage: Message data.|
| Return value| ACLLITE_OK: Sent successfully.<br>Non-ACLLITE_OK: Failed to send.|
| Remarks| For details about the AclLiteMessage structure, see [**AclLiteMessage**](#AclLiteMessage).|

| Item| Description|
|---|---|
| Function| shared_ptr<AclLiteMessage> PopMsgFromQueue() |
| Feature| Pops message data from a message queue.| 
| Parameter| None|
| Return value| Messages data.<br>nullptr: Invalid message data.<br>Non-nullptr: Valid message data.|

| Item| Description|
|---|---|
| Function| AclLiteThread* GetUserInstance() |
| Feature| Obtains the AclLiteThread thread instance.| 
| Parameter| None|
| Return value| AclLiteThread instance:<br>nullptr: Invalid thread instance.<br>Non-nullptr: Valid thread instance.|

| Item| Description|
|---|---|
| Function| const std::string& GetThreadName() |
| Feature| Obtains the AclLiteThread thread name.| 
| Parameter| None|
| Return value| Thread name. Values are as follows:<br>Empty string: Failed to obtain the name.<br>Non-null character string: Actual name.|

| Item| Description|
|---|---|
| Function| void SetStatus(AclLiteThreadStatus status) |
| Feature| Sets the thread instance status.| 
| Parameter| status: Status of the thread to be set.|
| Return value| None|
| Remarks| AclLiteThreadStatus:<br>THREAD_READY = 0 // The thread is ready.<br>THREAD_RUNNING = 1 // The thread is running.<br>THREAD_EXITING = 2 // The thread is exiting.<br>THREAD_EXITED = 3 // The thread exits.<br>THREAD_ERROR = 4 // Thread initialization fails.|

| Item| Description|
|---|---|
| Function| AclLiteThreadStatus GetStatus() |
| Feature| Obtains the status of a thread instance.| 
| Parameter| None|
| Return value| status:<br>THREAD_READY = 0 // The thread is ready.<br>THREAD_RUNNING = 1 // The thread is running.<br>THREAD_EXITING = 2 // The thread is exiting.<br>THREAD_EXITED = 3 // The thread exits.<br>THREAD_ERROR = 4 // Thread initialization fails.|


### 6. Other

#### 6.1 AclLiteUtils.h

This file defines the tool functions and macros used in the AclLite public library.

| Item| Description|
|---|---|
| Macro| RGBU8_IMAGE_SIZE(width, height) |
| Feature| Calculates the size of the RGB 24-bit image data.| 
| Parameter| width: Image width.<br>height: Image height.|

| Item| Description|
|---|---|
| Macro| RGBF32_IMAGE_SIZE(width, height) |
| Feature| Calculates the size of the RGB C3F32 image data.| 
| Parameter| width: Image width.<br>height: Image height.|

| Item| Description|
|---|---|
| Macro| YUV420SP_SIZE(width, height) |
| Feature| Calculates the size of the YUVSP420 image data.| 
| Parameter| width: Image width.<br>height: Image height.|

| Item| Description|
|---|---|
| Macro| YUV420SP_CV_MAT_HEIGHT(height) |
| Feature| Calculates the height when the YUVSP420 NV12 image is loaded to the OpenCV mat.| 
| Parameter| Height of a YUV image|

| Item| Description|
|---|---|
| Macro| SHARED_PTR_DVPP_BUF(buf) |
| Feature| Creates a smart pointer that points to the DVPP memory.| 
| Parameter| buf: Pointer to the DVPP memory.|

| Item| Description|
|---|---|
| Macro| SHARED_PTR_DEV_BUF(buf) |
| Feature| Creates a smart pointer that points to the device memory.| 
| Parameter| buf: Pointer to the device memory.|

| Item| Description|
|---|---|
| Macro| SHARED_PTR_U8_BUF(buf) |
| Feature| Creates a smart pointer that points to the memory.| 
| Parameter| buf: Pointer to the memory.|

| Item| Description|
|---|---|
| Macro| ALIGN_UP(num, align) |
| Feature| Calculates the value after alignment.| 
| Parameter| num: Original value.<br>align: Number to be aligned.|

| Item| Description|
|---|---|
| Macro| ALIGN_UP2(num) |
| Feature| Aligns data by 2.| 
| Parameter| num: Original value.|

| Item| Description|
|---|---|
| Macro| ALIGN_UP16(num) |
| Feature| Aligns data by 16.| 
| Parameter| num: Original value.|

| Item| Description|
|---|---|
| Macro| ALIGN_UP128(num) |
| Feature| Aligns data by 128.| 
| Parameter| num: Original value.|

| Item| Description|
|---|---|
| Macro| SIZEOF_ARRAY(array) |
| Feature| Calculates the number of data records in an array.| 
| Parameter| array: Array.|

| Item| Description|
|---|---|
| Macro| ACLLITE_LOG_ERROR(fmt, ...) |
| Feature| Print ACL error logs to the **/var/log/npu/slog/host-0/host-xxxx.log**.| 
| Parameter| fmt: Printed log content. Formatted character strings are supported. Recorded logs include functions, files, and line numbers.|

| Item| Description|
|---|---|
| Macro| ACLLITE_LOG_INFO(fmt, ...) |
| Feature| Print ACL INFO-level logs to the **/var/log/npu/slog/host-0/host-xxxx.log**.| 
| Parameter| fmt: Printed log content. Formatted character strings are supported. Recorded logs include functions, files, and line numbers.|

| Item| Description|
|---|---|
| Macro| ACLLITE_LOG_WARNING(fmt, ...) |
| Feature| Print ACL warning logs to the **/var/log/npu/slog/host-0/host-xxxx.log**.| 
| Parameter| fmt: Printed log content. Formatted character strings are supported. Recorded logs include functions, files, and line numbers.|

| Item| Description|
|---|---|
| Macro| ACLLITE_LOG_DEBUG(fmt, ...) |
| Feature| Print ACL debug logs to the **/var/log/npu/slog/host-0/host-xxxx.log**.| 
| Parameter| fmt: Printed log content. Formatted character strings are supported. Recorded logs include functions, files, and line numbers.|

| Item| Description|
|---|---|
| Macro| TIME_START(X) |
| Feature| Timestamp start point.| 
| Parameter| X: Name of the function to be measured.|
| Remarks| TIME_START(X) and TIME_END(X) must be used together. The same X is used as a group. The TIME_XXX_SHOW(X) series macros are used for printing.|

| Item| Description|
|---|---|
| Macro| TIME_END(X) |
| Feature| End point of the timestamp.| 
| Parameter| X: Name of the function to be measured.|
| Remarks| TIME_START(X) and TIME_END(X) must be used together. The same X is used as a group. The TIME_XXX_SHOW(X) series macros are used for printing.|

| Item| Description|
|---|---|
| Macro| TIME_USEC(X) |
| Feature| Time consumption in microseconds.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_USEC_SHOW(X) |
| Feature| Prints the time consumption in microseconds.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_MSEC(X) |
| Feature| Records time consumption in milliseconds.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_MSEC_SHOW(X) |
| Feature| Prints the time consumption in milliseconds.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_SEC(X) |
| Feature| Records time consumption in seconds.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_SEC_SHOW(X) |
| Feature| Prints the time consumption in seconds.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_MINUTE(X) |
| Feature| Records the time consumption in minutes.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_MINUTE_SHOW(X) |
| Feature| Prints the time consumption in minutes.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_HOUR(X) |
| Feature| Records the time consumption in hours.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Macro| TIME_HOUR(X) |
| Feature| Prints the time consumption by hour.| 
| Parameter| X: Name of the function to be measured.|

| Item| Description|
|---|---|
| Function| bool IsDirectory(const std::string &path) |
| Feature| Checks whether a character string is a valid folder path.| 
| Parameter| path: Input character string.|
| Return value| true: Valid path.<br>false: Invalid path.|

| Item| Description|
|---|---|
| Function| void* CopyDataToDevice(const void* data, uint32_t size, aclrtRunMode curRunMode, MemoryType memType) |
| Feature| Copies data to the device or DVPP memory.| 
| Parameter| data: Data to be copied.<br>size: Size of the data to be copied.<br>curRunMode: Program run mode.<br> memType: Memory type of the destination side.|
| Return value| Memory pointer on the destination side:<br>nullptr: Failed to copy.<br>Non-nullptr: copied successfully.|
| Remarks| For details about the value of aclrtRunMode, see the official document.<br>For details about the value of MemoryType, see [**MemoryType**](#MemoryType). In this interface, the value is MEMORY_DEVICE or MEMORY_DVPP.|

| Item| Description|
|---|---|
| Function| AclLiteError CopyDataToDeviceEx(void* dest, uint32_t destSize, const void* src, uint32_t srcSize, aclrtRunMode runMode) |
| Feature| Copies data to the device.| 
| Parameter| dest: Destination memory.<br>destSize: Size of the target memory.<br>src: Memory to be copied.<br>srcSize: Size of the memory to be copied.<br>curRunMode: Program run mode.|
| Return value| ACLLITE_OK: Copied successfully.<br>Non-ACLLITE_OK: Failed to copy.|
| Remarks| For details about the value of aclrtRunMode, see the official document.|

| Item| Description|
|---|---|
| Function| void* CopyDataToHost(const void* data, uint32_t size,  aclrtRunMode curRunMode, MemoryType memType)|
| Feature| Copies data to the host or common memory.| 
| Parameter| data: Data to be copied.<br>size: Size of the data to be copied.<br>curRunMode: Program run mode.<br> memType: Memory type of the destination side.|
| Return value| Memory pointer on the destination side:<br>nullptr: Failed to copy.<br>Non-nullptr: copied successfully.|
| Remarks| For details about the value of aclrtRunMode, see the official document.<br>For details about the value of MemoryType, see [**MemoryType**](#MemoryType). In this interface, the value is MEMORY_NORMAL or MEMORY_HOST.|

| Item| Description|
|---|---|
| Function| AclLiteError CopyDataToHostEx(void* dest, uint32_t destSize, const void* src, uint32_t srcSize, aclrtRunMode runMode); |
| Feature| Copies data to the host.| 
| Parameter| dest: Destination memory.<br>destSize: Size of the target memory.<br>src: Memory to be copied.<br>srcSize: Size of the memory to be copied.<br>curRunMode: Program run mode.|
| Return value| ACLLITE_OK: Copied successfully.<br>Non-ACLLITE_OK: Failed to copy.|
| Remarks| For details about the value of aclrtRunMode, see the official document.|

| Item| Description|
|---|---|
| Function| void* CopyData(const void* data, uint32_t size, aclrtMemcpyKind policy, MemoryType memType) |
| Feature| Copies data.| 
| Parameter| data: Data to be copied.<br>size: Size of the data to be copied.<br>policy: Memory copy type.<br> memType: Memory type of the destination side.|
| Return value| Memory pointer on the destination side|
| Remarks| For details about the value of aclrtMemcpyKind, see the official document.|

| Item| Description|
|---|---|
| Function| AclLiteError ReadJpeg(ImageData& image, const std::string& fileName) |
| Feature| Reads JPEG images.| 
| Parameter| image: Stores the read image data.<br>fileName: Path of the image file that is read.|
| Return value| ACLLITE_OK: Read successfully.<br>Non-ACLLITE_OK: Failed to read.|
| Constraints| Only baseline images are supported. Progressive JPEG images are not supported.|
| Remarks| For details about the ImageData data structure, see [**ImageData**](#ImageData).|

| Item| Description|
|---|---|
| Function| void GetAllFiles(const std::string &pathList, std::vector<std::string> &fileVec)|
| Feature| Obtains the names of all files in a path.| 
| Parameter| pathList: File path.<br>fileVec: Vector for storing file names.|
| Return value| None|

| Item| Description|
|---|---|
| Function| void SaveBinFile(const std::string& filename, const void* data, uint32_t size) |
| Feature| Saves data as a binary file.| 
| Parameter| filename: Name of the file to be added.<br>data: Data to be saved.<br>size: Size of the data to be saved.|
| Return value| None|

| Item| Description|
|---|---|
| Function| AclLiteError ReadBinFile(const std::string& filename, void*& data, uint32_t& size) |
| Feature| Reads the binary file.| 
| Parameter| filename: Name of the file to be added.<br>data: Data to be read.<br>size: Size of the data to be read.|
| Return value| ACLLITE_OK: Read successfully.<br>Non-ACLLITE_OK: Failed to read.|

| Item| Description|
|---|---|
| Function| AclLiteError CopyImageToLocal(ImageData& destImage, ImageData& srcImage, aclrtRunMode curRunMode); |
| Feature| Copies data of the ImageData type to the local host.| 
| Parameter| destImage: Copied data.<br>srcImage: Data to be copied.<br>curRunMode: Program run mode.|
| Return value| ACLLITE_OK: Copied successfully.<br>Non-ACLLITE_OK: Failed to copy.|
| Remarks| For details about the value of aclrtRunMode, see the official document.|

| Item| Description|
|---|---|
| Function| AclLiteError CopyImageToDevice(ImageData& destImage, ImageData& srcImage, aclrtRunMode curRunMode, MemoryType memType);|
| Feature| Copies the ImageData data to the local computer and to the **device/dvpp** directory.| 
| Parameter| destImage: Copied data.<br>srcImage: Data to be copied.<br>curRunMode: Program run mode.<br>memType: Memory type of the destination side.|
| Return value| ACLLITE_OK: Copied successfully.<br>Non-ACLLITE_OK: Failed to copy.|
| Remarks| For details about the value of aclrtRunMode, see the official document.<br>For details about the value of MemoryType, see [**MemoryType**](#MemoryType). In this interface, the value is MEMORY_DEVICE or MEMORY_DVPP.|

| Item| Description|
|---|---|
| Function| bool IsIpAddrWithPort(const std::string& addrStr) |
| Feature| Checks whether the IP address is in the <1-255>.<0-255>.<0-255>.<0-255>:\<port> format.| 
| Parameter| addrStr: IP address to be checked. The value is a character string.|
| Return value| true: Correct format.<br>false: Incorrect format.|

| Item| Description|
|---|---|
| Function| void ParseIpAddr(std::string& ip, std::string& port, const std::string& addr) |
| Feature| The <1-255>.<0-255>.<0-255>.<0-255>:\<port> format is split into two character string variables: ip and port.| 
| Parameter| ip: Character string variable after splitting, which is the IP address part.<br>port: Character string variable after splitting, which is the port part.<br>addr: Sring variable indicating a complete address. |
| Return value| None|

| Item| Description|
|---|---|
| Function| bool IsVideoFile(const std::string& path) |
| Feature| Checks whether a character string is an MP4 file.| 
| Parameter| path: Character string to be checked.|
| Return value| true: MP4<br>false: Not MP4|
| Remarks| This function only verifies the file name but not the file property.|

| Item| Description|
|---|---|
| Function| bool IsRtspAddr(const std::string &str) |
| Feature| Determines whether a character string is in the RTSP stream format, for example, rtsp://...| 
| Parameter| str: Character string to be checked.|
| Return value| true: RTSP stream.<br>false: Non-RTSP stream.|
| Remarks| This function only verifies the file name but not the file property.|

| Item| Description|
|---|---|
| Function| bool IsDigitStr(const std::string& str) |
| Feature| Checks whether a character string is a numeric string.| 
| Parameter| str: Character string to be checked.|
| Return value| true: Numeric string.<br>false: Non-numeric string.|

| Item| Description|
|---|---|
| Function| bool IsPathExist(const std::string &path) |
| Feature| Checks whether the file path exists. | 
| Parameter| path: Character string to be checked.|
| Return value| true: The path exists.<br>false: The path does not exist.|

| Item| Description|
|---|---|
| Function| bool ReadConfig(std::map<std::string, std::string>& config, const char* configFile) |
| Feature| Reads the configuration file and saves the parsing result to the map.| 
| Parameter| config: Stores the map of the parsing result.<br>configFile: Path of the file to be read.|
| Return value| true: Read successfully.<br>false: Failed to read.|
| Constraints| The configuration item name must be unique and in the following format:<br> [baseconf]<br>presenter_server_ip=xxx.xxx.x.xxx |

| Item| Description|
|---|---|
| Function| void PrintConfig(const std::map<std::string, std::string> & m) |
| Feature| Prints configuration items.| 
| Parameter| m: Map of configuration items to be printed.|
| Return value| None|

<a name="AclLiteType.h"></a>
#### 6.2 AclLiteType.h

This file defines the structures and enumeration variables used in the AclLite public library.

| Item| <a name="MemoryType">Description</a>|
|---|---|
| Enumeration type| MemoryType |
| Feature| Labels different types of memory.| 
| Options are as follows:| MEMORY_NORMAL = 0 // General memory. Use new or delete to apply for or release the memory.<br>MEMORY_HOST // Host-side memory, which is applied for or released using aclrtMallocHost or aclrtFreeHost.<br>MEMORY_DEVICE // Memory on the device. aclrtMalloc and aclrtFree are used to apply for and release the memory.<br>MEMORY_DVPP // DVPP memory. acldvppMalloc and acldvppFree are used to apply for and release the DVPP memory.<br>MEMORY_INVALID_TYPE // Invalid memory type.|

| Item| Description|
|---|---|
| Enumeration type| CopyDirection |
| Feature| Labels the copy direction.| 
| Options are as follows:| TO_DEVICE = 0 // To the device.<br>TO_HOST // To the host.<br>INVALID_COPY_DIRECT // Invalid copy direction.|

| Item| Description|
|---|---|
| Enumeration type| CameraId |
| Feature| Labels the Atlas 200 DK camera.| 
| Options are as follows:| CAMERA_ID_0 = 0 // Camera in slot 0.<br>CAMERA_ID_1 // Camera in slot 1.<br>INVALID_COPY_DIRECT // Invalid slot.|

| Item| Description|
|---|---|
| Enumeration type| VencStatus |
| Feature| Status of the VENC service thread.| 
| Options are as follows:| STATUS_VENC_INIT = 0 // Initialization.<br>STATUS_VENC_WORK // Working.<br>STATUS_VENC_FINISH // Service completed.<br>STATUS_VENC_EXIT // Thread exits.<br>STATUS_VENC_ERROR // Initialization failure or encoding error.|

| Item| <a name="VencConfig">Description</a>|
|---|---|
| Structure| VencConfig |
| Function| Stores the parameters required for VencHelper initialization.| 
| Member| maxWidth: Width.<br>maxHeight: Height.<br>outFile: Encodes output file.<br>format: Format of the image to be encoded.<br>enType: Encoding file stream format.<br>context: Thread running context.<br>runMode: Thread run mode.|
| Constraints| format: PIXEL_FORMAT_YUV_SEMIPLANAR_420 / PIXEL_FORMAT_YVU_SEMIPLANAR_420<br>enType: H265_MAIN_LEVEL / H264_BASELINE_LEVEL / H264_MAIN_LEVEL / H264_HIGH_LEVEL|

| Item| <a name="ImageData">Description</a>|
|---|---|
| Structure| ImageData |
| Feature| Data structure for encapsulating image data and image parameters.| 
| Member|  format: Image format.<br>width: Image width.<br>height: Image height.<br> alignWidth: Width after alignment.<br>alignHeight: Height after alignment.<br>size: Image data size.<br>data: Image data.|

| Item| Description|
|---|---|
| Structure| Resolution |
| Feature| Resolution| 
| Member| width: Width.<br>height: Height.|

| Item| Description|
|---|---|
| Structure| Rect |
| Feature| Coordinates of a rectangle.| 
| Member| ltX: X coordinate of the upper left point.<br>ltY: Y coordinate of the upper left point.<br>rbX: X coordinate of the lower right point.<br>rbY: Y coordinate of the lower right point.|

| Item| Description|
|---|---|
| Structure| BBox |
| Feature| BoundingBox data.| 
| Member| rect: left point of the rectangle.<br>score: Confidence score.<br>text: Class label.|
| Remarks|  |

| Item| <a name="AclLiteMessage">Description</a>|
|---|---|
| Structure| AclLiteMessage |
| Feature| Messages data.| 
| Member| dest: Target thread ID.<br>msgId: Message ID.<br>data: Message data.|

| Item| <a name="DataInfo">Description</a>|
|---|---|
| Structure| DataInfo |
| Feature| Data information, which stores the data content and size and is used to generate the model input and output.| 
| Member| data: Data<br>size: Data size.|
| Remarks|  |

| Item| <a name="InferenceOutput">Description</a>|
|---|---|
| Structure| InferenceOutput |
| Feature| Stores model inference result data.| 
| Member| data: Result data.<br>size: Data size.|

#### 6.3 AclLiteError.h

This file defines the error codes used in the ACLLite public library. For details, see the file.

#### 6.4 ThreadSafeQueue.h

This file defines a template class ThreadSafeQueue, which is used to provide users with data security queues that can be directly reused in multi-thread scenarios. For details, see the file.
