[TOC]

# SceneRecognition

# 1 Introduction

## 1.1 Background

The purposes of this reference design are as follows:

1. Provide reference examples of Atlas-based OCR for financial, security, and Internet customers to verify the feasibility.

2. Function as a programming example (referred to as demo) for customers to develop OCR applications using ACL, open source code to customers for reference, and lower the admission threshold for customers.

3. Provide system optimization cases under the new architecture and build a prototype system with optimal performance, making Atlas a best option for customers.

This demo only concerns vertical text and recognizes the text in JPEG input images. This document describes the OCR Demo implementation scheme.

## 1.2 Supported Products

An Atlas 300-3010 is used for experiment verification. For details about product pictures and hardware parameters, see the *Atlas 300 AI Accelerator Card User Guide (Model 3010)*. An Atlas 800 AI server with Atlas 300 cards is used as the hardware platform. A server needs to be accessed using the network. Therefore, you need to use a client such as a laptop or PC to access the server.

### Supported Product Forms

Atlas 800 (Model 3000), Atlas 800 (Model 3010), Atlas 300 (Model 3010)

### Supported ACL Version

1.75.T11.0.B116, 1.75.T15.0.B150, 20.1.0

To query the ACL version, run the following command in the environment where Atlas products are installed:
```bash
npu-smi info
```

## 1.3 Software Solution

The software solution includes three necessary parts of OCR: text detection, image correction and character recognition, especially text detection and recognition.
To achieve high performance, a pipeline processing solution is used. For details about the functions of each module in the pipeline, see Table1.1 Functions of each module in the system solution.

Table 1.1 Functions of each module in the system solution:

| Number | Subsystem                  | Function Description                     |
| ------ | -------------------------- | ---------------------------------------- |
| 1      | Image reading              | Read an image from a specified folder or file. (Only JPEG images are supported.) |
| 2      | Image preprocessing        | To reduce the transmission bandwidth, videos or images need to be encoded and compressed before being transmitted over the network. The subsequent processing is based on RGB data. Therefore, decoding is required. To ensure the overall system performance, the hardware (DVPP) is used for resizing the decoded images to meet the requirements on input of the text detection model. |
| 3      | Text detection             | An core module of the system. The open-source model AdvancedEAST is used for text detection. It can detect the four vertex coordinates of the area where text is located. Typically multiple text boxes can be detected in an image. AdvancedEAST performs perspective transformation and text recognition on each of these text boxes. |
| 4      | Perspective transformation | Considering the shooting angle of an input image, the text in the image may be distorted. In this case, crop the detected text box and perform perspective transformation to obtain the aligned text-based image. Then resize the aligned text-based image using DVPP to meet the requirements on input of the text detection model. |
| 5      | Text recognition           | An core module of the system. The open-source model chinese-ocr is used for text recognition to detect text in images and output corresponding text information. |
| 6      | Result processing          | Combine the recognition results of multiple text boxes of the same image and save them to the result file. The image name, image ID, text box ID, text box coordinates, and text information in the text box are displayed. |

### Main Code Directories

This project name of this demo is SceneTextRecognition. In the root directory, src is the source code directory, and dist is the running directory. The following describes the subdirectories of dist and src:
```
.
├── AscendBase
│   └── src
│       └── Base
│           ├── BlockingQueue
│           ├── CommandParser
│           ├── CommonDataType
│           ├── ConfigParser
│           ├── DvppCommon
│           ├── ErrorCode
│           ├── FileManager
│           ├── Framework
│           ├── Log
│           ├── PointerDeleter
│           ├── ResourceManager
│           └──Statistic
├── Common
│   ├── CommandLine.h
│   ├── CommonType.h
├── Data
│   ├── Config                   // Configuration file
│   └── Models
│           ├── TextDetection    // Text detection model
│           ├── TextRecognition  // Text model
├── Modules
│   ├── DetectPost               // Text detection post-processing module
│   ├── ImagePreprocess          // Image pre-processing module
│   ├── ImageReader              // Image reading module
│   ├── ResultProcess            // Result combination processing module
│   ├── TextDetection            // Text detection model inference module
│   ├── TextRecognition          // Text recognition model inference module
│   └── WarpPerspective          // Picture perspective transformation module
├── build.sh
└── CMakeLists.txt
```


# 2 Environment Setup

### 2.1 Third-Party Software Dependency Description

**Table 2-1** Third-Party software dependency

| Dependent Software | Version | How to Obtain                            | Description                              |
| ------------------ | ------- | ---------------------------------------- | ---------------------------------------- |
| gcc                | 7.3.0   | [Link](https://github.com/gcc-mirror/gcc/releases/tag/releases%2Fgcc-7.3.0) | used to compile the entire demo.         |
| opencv             | 4.2.0   | [Link](https://github.com/opencv/opencv/releases) | Basic component of OpenCV, which is used for basic image processing. |

*Note: GCC 7.3.0 or a later version is required for installing OpenCV-4.2.0. If the GCC version in the compilation environment is earlier than 7.3.0, upgrade GCC by referring to section 2.2 "Installing GCC-7.3.0."*

### 2.2 Installing GCC-7.3.0 (Skip This Step If the GCC Version is 7.3.0 or Later.)

**Table 2-2** GCC-7.3.0 installation software dependency list

| Software | Version | How to Obtain                         |
| -------- | ------- | ------------------------------------- |
| gmp      | 6.2.0   | [Link](https://ftp.gnu.org/gnu/gmp/)  |
| mpfr     | 4.0.2   | [Link](https://ftp.gnu.org/gnu/mpfr/) |
| mpc      | 1.1.0   | [Link](https://ftp.gnu.org/gnu/mpc/)  |


- **Step 1**   Log in to the server as the **root** user.

- **Step 2**   Install the dependencies.

  Install gmp:
  ```
   cd /your/path/to/uncompressed/gmp-6.2.0
   ./configure --prefix=/usr/local/gmp-6.2.0
   make -j
   make install
  ```

  Install mpfr:
   ```
   cd /your/path/to/uncompressed/mpfr-4.0.2
   ./configure --prefix=/usr/local/mpfr-4.0.2 --with-gmp=/usr/local/gmp-6.2.0
   make -j
   make install
   ```

   Install mpc:
   ```
   cd /your/path/to/uncompressed/mpc-1.1.0
   ./configure --prefix=/usr/local/mpc-1.1.0 --with-gmp=/usr/local/gmp-6.2.0 --with-mpfr=/usr/local/mpfr-4.0.2
   make -j
   make install
   ```
  Set the temporary environment variable:

  ```
  export LD_LIBRARY_PATH=/usr/local/gmp-6.2.0/lib:/usr/local/mpc-1.1.0/lib:/usr/local/mpfr-4.0.2/lib:$LD_LIBRARY_PATH
  ```

- **Step 3**   Download the **gcc-7.3.0.tar.gz** code package to any directory, for example, **/home/HwHiAiUser/gcc-7.3.0**, and run the following command to decompress the package:

  ```
  tar -xzvf gcc-7.3.0.tar.gz
  ```

- **Step 4**  Run the following commands to go to the decompression directory and set the GCC-7.3.0 installation path to **/usr/local/gcc-7.3.0**:

  ```
  cd gcc-7.3.0
  ./configure --prefix=/usr/local/gcc-7.3.0 --with-gmp=/usr/local/gmp-6.2.0 --with-mpfr=/usr/local/mpfr-4.0.2 --with-mpc=/usr/local/mpc-1.1.0 --disable-multilib
  make -j
  make install
  ```

  Check whether the installation is successful as prompted.
  After the preceding commands are run, GCC is installed in the **/usr/local/gcc-7.3.0** directory. After the installation is complete, set environment variables to ensure that the software compilation tool and the linked GCC-related .so libraries use the newly installed GCC version.
  ```bash
  export PATH=/usr/local/gcc-7.3.0/bin:$PATH
  export LD_LIBRARY_PATH=/usr/local/gcc-7.3.0/lib64:$LD_LIBRARY_PATH
  ```

### 2.3 Installing OpenCV-4.2.0

- **Step 1**   Log in to the server as the **root** user.

- **Step 2**   Download the **opencv-4.2.0.tar.gz** code package to any directory, for example, **/home/HwHiAiUser/opencv-4.2.0**, and run the following command to decompress the package:

  ```
  tar -xzvf opencv-4.2.0.tar.gz
  ```

- **Step 3**  Run the following commands to go to the decompression directory, create the build and compilation directories, and go to the directories:

  If the GCC version in the environment is earlier than 7.3.0, install GCC 7.3.0 by referring to section 2.2 and run the following commands to compile OpenCV:
  ```
  mkdir build
  cd build
  cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_CXX_COMPILER=/usr/local/gcc-7.3.0/bin/g++ -DCMAKE_C_COMPILER=/usr/local/gcc-7.3.0/bin/gcc \
  -DCMAKE_INSTALL_PREFIX=/usr/local/opencv \
  -DBUILD_TESTS=OFF -DWITH_WEBP=OFF -DWITH_LAPACK=OFF \
  -DBUILD_opencv_world=ON ..
  make -j
  make install
  ```

  If the GCC version is later than 7.3.0, run the following commands to compile OpenCV directly:
  ```
  mkdir build
  cd build
  cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local/opencv \
  -DBUILD_TESTS=OFF -DWITH_WEBP=OFF \
  -DBUILD_opencv_world=ON ..
  make -j
  make install
  ```

  Check whether the installation is successful as prompted.

The preceding third-party software is installed in **/usr/local/** by default. After the installation is complete, set environment variables.

```bash
export LD_LIBRARY_PATH=/usr/local/opencv/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/opencv/lib:$LD_LIBRARY_PATH
```

# 3 Compilation and Running

## 3.1 Setting Environment Variables

Before compiling and running the demo, you need to set environment variables.

*  `ASCEND_HOME`      Ascend installation path. Generally, the path is `/usr/local/Ascend`.
*  `LD_LIBRARY_PATH`  Specifies the dynamic library lookup path on which the program depends.

```bash
export ASCEND_HOME=/usr/local/Ascend
export LD_LIBRARY_PATH=$ASCEND_HOME/ascend-toolkit/latest/acllib/lib64:$LD_LIBRARY_PATH
```

## 3.2 Compiling

- **Step 1**   Log in to the server as the **root** user and set environment variables.

- **Step 2**   Download the demo code to any directory, for example, **/home/HwHiAiUser/SceneTextRecognition**.

- **Step 3**   Run the following commands to build code:

  ```
   cd /home/HwHiAiUser/SceneTextRecognition/src
   bash build.sh
  ```

  * Note: After the compilation is complete, the executable file ocr is generated and stored in the **home/HwHiAiUser/SceneTextRecognition/dist/** directory.

## 3.3 Performing Model Conversion

  The open-source model AdvancedEAST is used for text detection. For details about the example model, see the [Model Conversion Description](Data/Models/TextDetection/README.zh.md).

  The open-source model chineseocr is used for text recognition. For details about the example model, see the [Model Conversion Description](Data/Models/TextRecognition/README.zh.md).


## 3.4 Configuring

  Before running the program, configure the following information in `Data/Config/setup.config`:

  Configure deviceId, debug mode and the number of cycle run times for program running.
  **debugMode** indicates the debug mode. After the debug mode is enabled, images and logs in the entire precess are saved to facilitate fault locating. The debug mode is disabled by default.
  **runTimes** indicates the number of cyclical file reads. *0* indicates the long-term stability test. You need to press **Ctrl+C** to end the long-term stability test.
  ```bash
  SystemConfig.deviceId = 0
  SystemConfig.debugMode = 0         // 0: debug off  1: debug on
  SystemConfig.runTimes = 1          // 0: long time test  other: cycle run times
  ```

  Configure the path of the image to be processed. You can specify a folder or a file.
  ```bash
  # configuration for ImageReader, the input path can be an path of a file or a folder
  ImageReader.inputPath = ./image
  ```

  Configure the input width and height, model name, and converted .om model required by the text detection model.

  The input aspect ratio of the text recognition model can be 1:1, 1:2, or 2:1 to improve the inference precision.

  Set the minimum and maximum width and height values supported by the model in dynamicHWList. The combination of two values forms four levels, which must be the same as the level parameter set during model conversion.
  ```bash
  # configuration for TextDetection
  # 2 values of width and height, we will get 4 types here: 416,416;416,832;832,416;832,832
  TextDetection.dynamicHWList = 416,832
  TextDetection.modelName = AdvancedEAST
  TextDetection.modelPath = ./Data/Models/TextDetection/advancedeast.om
  ```

  Configure the input height, model name, and converted .om model required by the text recognition model.

  The number of characters in an image is not fixed. Therefore, the dynamic width needs to be supported. Set the width level supported by the model in dynamicWidthList, with a maximum of 100 levels. The width level must be the same as the level parameter set during model conversion.

  **keysFilePath** indicates the path of the character dictionary file, which is used to search for corresponding characters using the index obtained during model inference.
  ```bash
  # configuration for TextRecognition
  TextRecognition.modelHeight = 32
  TextRecognition.modelName = chinese-ocr
  TextRecognition.modelPath = ./Data/Models/TextRecognition/chineseocr.om
  TextRecognition.dynamicWidthList = 32,64,96,128,160,192,224,256,288,320
  TextRecognition.keysFilePath = ./Data/Models/TextRecognition/keys.txt
  ```

  Configure the path for saving the text detection result in the image. A folder with a specified name is created in the path where the program runs.
  **enableCallback** indicates whether to enable the callback function for processing recognition results.
  ```bash
  # configuration for ResultProcess
  ResultProcess.savePath = ./result
  ResultProcess.enableCallback = 1 # whether to enable callback functions
  ```

## 3.5 Running

### Input Image Constraints

Only the JPG format is supported.

### Program Running

- **Step 1** Upload the image for OCR recognition, for example, **xxx.jpg**, to the **/home/HwHiAiUser/SceneTextRecognition/dist/image** directory.

- **Step 2** Run the following command to start the OCR program:

  ```
  ./ocr
  ```

  Check whether the command is run successfully based on the screen logs.

  The recognition result is stored in the **/home/HwHiAiUser/SceneTextRecognition/dist/result** directory.

* Note: You can run the **./ocr -h** command to view all information about the command. Table 3-1 Parameters for running describes the parameters that can be used for running.

**Table 3-1** Parameters for running

| Parameter   | Description                              | Default value |
| ----------- | ---------------------------------------- | ------------- |
| --log_level | Debug Level. 0: debug; 1: info; 2: warn; 3: error; 4: fatal; 5: off. | 1             |
| --stats     | Performance statistics. true: enabled; false: disabled | false         |

### Result Display

  The OCR result is saved in the **x_xx_xxx.txt** file in the specified path of the configuration file. *x* indicates the image name, *xx* indicates the image ID, and *xxx* indicates the timestamp.
  Each x_xx_xxx.txt file stores the name and ID of each image. The formats of ID of the text box detected in the image, four vertex coordinates of the text box, and text content are as follows:
  ```bash
  Image: test_0
  Item0[(138, 19), (138, 46), (346, 46), (346, 19)]: 故人西辞黄鹤楼
  Item1[(137, 60), (136, 87), (346, 87), (346, 61)]: 烟花三月下扬州
  Item2[(137, 102), (136, 129), (345, 128), (345, 103)]: 孤帆远影碧空尽
  Item3[(137, 143), (136, 171), (342, 171), (343, 144)]: 惟见长江天际流
  Item4[(137, 185), (137, 212), (344, 211), (344, 185)]: 朝辞白帝彩云间
  Item5[(137, 226), (137, 255), (345, 254), (346, 229)]: 千里江陵一日还
  Item6[(137, 269), (136, 295), (346, 296), (346, 270)]: 两岸猿声啼不住
  Item7[(137, 309), (137, 337), (345, 337), (345, 313)]: 轻舟已过万重山
  Item8[(138, 351), (138, 378), (347, 380), (347, 352)]: 天门中断楚江开
  Item9[(138, 393), (137, 420), (347, 419), (347, 394)]: 碧水东流至此回
  ```

# 4 Dynamic Library Dependency Description

For details about the demo dynamic library dependency, see the **CMakeLists.txt** file in the **src** directory in the code. For details about parameters, see the **link_libraries** and **target_link_libraries** files.

**Table 4-1** Dynamic library dependency description

| Dependent Software | Description                              |
| ------------------ | ---------------------------------------- |
| libascendcl.so     | ACL framework interface. For details, see the ACL interface document. |
| libacl_dvpp.so     | ACL framework interface. For details, see the ACL interface document. |
| libpthread.so      | Thread library of C++.                   |
| libopencv_world.so | Basic component of OpenCV, which is used for basic image processing. |
