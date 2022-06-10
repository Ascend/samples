[中文](catenation_environmental_guidance_CN.md) | English

### (Recommended) Install the development environment on the Ascend AI device and use the environment as the samples of the running environment.

The following describes how to install the CANN package as the **HwHiAiUser** user. If the **root** user is used for installation, change all **${HOME}** in the installation preparation to **/usr/local**.

#### Installation Preparation
Run the following command to prepare for the installation:
  ```
  # Run the following command in any directory as the installation user to open the **.bashrc** file:
  vi ~/.bashrc  
  # Add the following content to the end of the file:
  export CPU_ARCH=`arch`
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  # Samples dependent files that need to be linked during the build.
  export LD_LIBRARY_PATH=${THIRDPART_PATH}/lib:$LD_LIBRARY_PATH  # Runtime link library files.
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest # Path for storing files after the CANN software is installed. Change it to the actual installation path.
  # Run the following command to save the file and exit:
  :wq!  
  # Run the following command to make the modification take effect immediately:
  source ~/.bashrc 
  # Create dependent folders related to samples:
  mkdir -p ${THIRDPART_PATH}
  # Download the source code and install the Git:
  cd ${HOME}
  sudo apt-get install git
  git clone https://github.com/Ascend/samples.git
  # Copy the public files to the dependency paths of samples:
  cp -r ${HOME}/samples/common ${THIRDPART_PATH}
  ```  
In the Atlas 200 DK scenario, run the following commands to copy the .so files such as **media_mini** and related header files to meet the camera sample build requirements:
  ```
  mkdir -p ${INSTALL_DIR}/driver
  cp /usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libslog.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libc_sec.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libmmpa.so ${INSTALL_DIR}/driver/
  cp /usr/local/Ascend/include/peripheral_api.h ${INSTALL_DIR}/driver/
  ```
#### Installation Process
##### Installing OpenCV
Run the following command to install OpenCV. If OpenCV related functions are not used in the code, skip this step.
  ```
  sudo apt-get install libopencv-dev
  ```
##### Installing FFmpeg
Run the following command to install the FFmpeg source code. The FFmpeg version installed by the APT is too early, so you need to install the FFmpeg source code. Installing the FFmpeg source code is necessary for installing the ACLLite library. If the FFmpeg or ACLLite library is not used in the code, skip this step.
  ```
  # Download FFmpeg.
  cd ${HOME}
  wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
  tar -zxvf ffmpeg-4.1.3.tar.gz
  cd ffmpeg-4.1.3
  # Install FFmpeg.
  ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --prefix=${THIRDPART_PATH}
  make -j8
  make install
  ```
##### Installing the ACLLite Library
Run the following command to install the ACLLite (Note that the FFmpeg source code must be installed before the installation): If the functions related to the ACLLite library are not used in the code, skip this step.
  ```
  # Build and install ACLLite.
  cd ${HOME}/samples/cplusplus/common/acllite/
  make
  make install
  ```
##### Installing Presenter Agent
Run the following commands to install ProtoBuf and Presenter Agent: If the Presenter Agent function is not used in the code, skip this step.
  ```
  # Install the ProtoBuf dependency.
  sudo apt-get install autoconf automake libtool
  # Download the ProtoBuf source code.
  cd ${HOME}
  git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
  # Build and install ProtoBuf.
  cd protobuf
  ./autogen.sh
  ./configure --prefix=${THIRDPART_PATH}
  make clean
  make -j8
  sudo make install
  # Go to the Presenter Agent source code directory and build the code.
  cd ${HOME}/samples/cplusplus/common/presenteragent/proto
  ${THIRDPART_PATH}/bin/protoc presenter_message.proto --cpp_out=./
  # Start to build Presenter Agent.
  cd ..
  make -j8
  make install
  ```
