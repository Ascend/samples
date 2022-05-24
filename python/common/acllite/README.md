English|[中文](README_CN.md)
  
# Python_acllite Usage Description

## Usage

1. This library is used only for open source samples in the current community. It does not cover all application development scenarios of the Ascend platform and cannot be used as a standard library for user application development.

2. This library is verified only on the Atlas 200 DK and Atlas 300 (x86) servers.

## Constraints

| Adaptation Item    | Adaptation Condition                                                    | Remarks                                                        |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Applicable version  | ≥ 5.0.4                                                     | The version has been installed. For details about the version information, see [Version Number Change Notice](https://www.hiascend.com/en/software/cann/notice).|
| Applicable hardware  | Atlas 200 DK/Atlas 300 ([AI1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))| The library has passed the tests on Atlas 200 DK and Atlas 300. For details about the product description, see [Hardware](https://www.hiascend.com/en/hardware/product).|
| Third-party dependency| pyav, NumPy, and Pillow                                       | Install the dependencies by referring to [Third-Party Dependency Installation Guide (Python Sample)](../../environment).|


## Procedure

#### Public Library Deployment

Before running the application, deploy the public library to the operating environment. For details, see [Third-Party Dependency Installation Guide (Python Sample)](../../environment).
Then, add the path to the PYTHONPATH environment variable. For example, in the preceding document, the public library is copied to `$THIRDPART_PATH`. Add the following information to the **~/.bashrc** file:

```
export PYTHONPATH=$THIRDPART_PATH/acllite:$PYTHONPATH
```

Save the configured environment variable and run the following command to make the environment variable take effect:

```
source ~/.bashrc
```

After the preceding steps are complete, call the acllite library interfaces in the application code. For example:

```
import presenteragent.presenter_channel as presenter_channel

chan = presenter_channel.open_channel(config_file)
```

## Interface Description

### AclLiteImage Class

Provided in the python-acllite public library, the AclLiteImage class provides a unified data encapsulation structure for the Atlas 200 DK camera, JPG images, and input image data to facilitate subsequent processing by the public library interfaces.

#### \_\_init\_\_

Method: \_\_init\_\_ (image, width=0, height=0, size=0, memory_type=const.MEMORY_NORMAL):

Note:

Generates data of the AclLiteImage structure based on the initialization parameter list.

Input parameter:

**image**: image data. It can be NumPy arrays, JPEG/PNG image paths, and memory data. If this parameter is left blank or set to an unsupported type, an error is reported.

**width**: image width. If the image is a JPEG or PNG image, this parameter can be left empty. In this case, the default value **0** is used.

**height**: image height. If the image is a JPEG or PNG image, this parameter can be left empty. In this case, the default value **0** is used.

**size**: image data size. If the image is a JPEG or PNG image, this parameter can be left empty. In this case, the default value **0** is used.

**memory_type**: image data storage type, that is, whether the image data is stored in the normal memory, device memory, host memory, or DVPP memory. If the image is a JPEG or PNG image, this parameter can be left empty. In this case, the default value **MEMORY_NORMAL** is used.

Return value:

Data of the AclLiteImage structure

Constraints:

None

#### save

Method: save(filename):

Note:

Converts the AclLiteImage data into the NumPy array and saves it as a binary file.

Input parameter:

**filename**: name of the saved file.

Return value:

None

Constraints:

None

### Camera Class

The Camera class provides Python interfaces for Atlas 200 DK onboard camera decoding.

#### is_opened

Method: is_opened()

Note:

Checks whether the onboard camera of the Atlas 200 DK is opened based on the camera ID of the initialized cameracapture class object.

Input parameter:

None

Return value:

**TRUE**: The camera is turned on.

**FALSE**: The camera is not turned on.

Constraints:

None

#### read

Method: read()

Note:

Reads images from the camera specified by the ID during initialization of the cameracapture class object and saves the images as data in the AclLiteImage structure.

Input parameter:

None

Return value:

Image data of the AclLiteImage type

Constraints:

None

#### close

Method: close()

Note:

Close the camera.

Input parameter:

None

Return value:

None. If the command is executed properly, the **Close camera** field is displayed.

Constraints:

None

### AclLiteModel Class

Provided in the python-acllite library, the AclLiteModel class encapsulates APIs related to ACL model inference, including but not limited to model loading and initialization, creation of model input and output datasets, model inference execution, and resource release.

#### execute

Method: execute(input_list)

Note:

Model inference API, which converts the input data into ACL dataset data and sends the data to the model for inference. The inference result is represented in NumPy array.

Input parameter:

**input_list**: model input data, which can be AclLiteImage, NumPy array, or {'data': ,'size':} dict structure data.

Return value:

**NumPy array**, which is used to represent the model inference result.

Constraints:

None


### AclLiteImageProc Class

Provided in the python-acllite library, the AclLiteImageProc class encapsulates CANN media data processing APIs, including but not limited to image decoding and encoding, video decoding and encoding, and image cropping and resizing.

#### jpegd

Method: jpegd(image):

Note:

Image decoding API, which is used to convert JPEG images into YUV images.

Input parameter:

**image**: original JPEG image data, which is stored in the AclLiteImage structure.

Return value:

**AclLiteImage**: stores YUV image data.

Constraints:

None

#### jpege

Method: jpege(image):

Note:

Image decoding API, which is used to convert YUV images into JPEG images.

Input parameter:

**image**: original YUV image data, which is stored in the AclLiteImage structure.

Return value:

**AclLiteImage**: stores JPEG image data.

Constraints:

None

#### crop_and_paste

Method: crop_and_paste(image, width, height, crop_and_paste_width, crop_and_paste_height)

Note:

Image vision preprocessing core (VPC) API, which is used to crop the original image and paste it to the target size.

Input parameter:

**image**: original image data, which is stored in the AclLiteImage structure.

**width**: width of the original image.

**height**: height of the original image.

**crop_and_paste_width**: width of the target image after the VPC.

**crop_and_paste_height**: height of the target image after the VPC.

Return value:

**AclLiteImage**: stores the image data after the VPC.

Constraints:

None

#### resize

Method: resize(image, resize_width, resize_height)

Note:

Resizes the input image to the specified size.

Input parameter:

**image**: original image data, which is stored in the AclLiteImage structure.

**resize_width**: destination image width.

**resize_height**: destination image height.

Return value:

**AclLiteImage**: stores the image data after resizing.

Constraints:

None

### Dvpp_Vdec Class

Provided in the python-acllite library, the Dvpp_Vdec class encapsulates video stream decoding interfaces, including video stream frame cut.

#### read

Method: read(no_wait):

Note:

Video frame read interface (asynchronous), which reads data from the queue and sends the data to the decoder.

Input parameter:

**no_wait**: Boolean variable. If the value is **true**, data is continuously read from the queue. The **is_finished ()** interface is used to check whether the frame data is decoded completely. If the value is **false**, data is read from the queue at the interval specified by **READ_TIMEOUT**. If the value is empty, an error is reported. The default value is **false**.

Return value:

**ret**: interface execution result. **SUCCESS** indicates that the execution is successfully completed. **FAILED** indicates that the interface fails to read data from the queue because some data is not decoded.

**image**: video frame that is read.

Constraints:

The video stream must be in one of the following formats:

H.264: main, baseline, or high level, and in the annex-b format

H.265: main level

#### process

Method: process(input_data, input_size, user_data)

Note:

Video decoding interface. It transmits the video frame data to be decoded to the decoder.

Input parameter:

**input_data**: input data.

**input_size**: size of the input data.

**user_data**: Python object (user-defined). If you want to obtain the sequence number of the decoded frame, define the **user_data** parameter. Then, the sequence number can be passed to the VDEC callback function to determine the frame to be processed.

Return value:

**ret**: interface execution result. The value can be **SUCCESS** or **FAILED**.


Constraints:

None
