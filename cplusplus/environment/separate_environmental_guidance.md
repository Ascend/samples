[中文](separate_environmental_guidance_CN.md) | English

### Installing Third-Party Software on a Non-Ascend AI Device in the Development Environment
**The following describes how to install the CANN package as HwHiAiUser, a common user.**

#### Installation Preparation
- Development environment   
  Run the following command to prepare for the installation. **Set the CPU_ARCH environment variable based on the CPU architecture of the running environment**.
  ```
  # Run the following command in any directory in the development environment as the installation user to open the .bashrc file:
  vi ~/.bashrc  
  # Add the following content to the end of the file: Set the CPU_ARCH environment variable based on the CPU architecture of the running environment, for example, export CPU_ARCH=aarch64.
  export CPU_ARCH=[aarch64/x86_64]
  # Set THIRDPART_PATH based on the installation path of the running environment. If the running environment is an ARM platform and the installation path is Ascend-arm, set THIRDPART_PATH to "export THIRDPART_PATH=${HOME}/Ascend-arm/thirdpart/${CPU_ARCH}".
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  # Third-party library to be linked during code build.
  # The file storage path for the CANN software installation. Set the last-level directory based on the running environment. If the running environment is an ARM platform, set this parameter to arm64-linux. If the running environment is an x86 platform, set this parameter to x86_64-linux. The following uses the ARM environment as an example.
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest/arm64-linux
  # Run the following command to save the file and exit:
  :wq!  
  # Run the following command to make the modification take effect immediately:
  source ~/.bashrc 
  # Create a third-party dependency folder.
  mkdir -p ${THIRDPART_PATH}
  # Copy public files to the third-party dependency folder.
  cd $HOME
  git clone https://github.com/Ascend/samples.git
  cp -r ${HOME}/samples/common ${THIRDPART_PATH}
  ```  
  If the running environment is Atlas 200 DK, run the following commands to copy dynamic libraries such as **media_mini** and related header files to meet the camera sample compilation requirements:
  ```
  mkdir -p ${INSTALL_DIR}/driver
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libslog.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libc_sec.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libmmpa.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/local/Ascend/include/peripheral_api.h ${INSTALL_DIR}/driver/
  ```
- Runtime environment   
  Run the following command to prepare for the installation:
  ```
  # Use the installation user to run the following command in any directory in the running to open the .bashrc file:
  vi ~/.bashrc  
  # Add the following content to the end of the file: Set the CPU_ARCH environment variable based on the CPU architecture of the running environment, for example, export CPU_ARCH=aarch64.
  export CPU_ARCH=`arch`
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  # Third-party library to be linked during code build.
  export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH  # Runtime link library files.
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest  # CANN component directory.
  # Run the following command to save the file and exit:
  :wq!  
  # Run the following command to make the modification take effect immediately:
  source ~/.bashrc 
  # Create a third-party dependency folder.
  mkdir -p ${THIRDPART_PATH}
  # Copy related data. X.X.X.X indicates the IP address of the operating environment.
  sudo scp -r HwHiAiUser@X.X.X.X:${THIRDPART_PATH}/common ${THIRDPART_PATH}
  ```  
  If the running environment is Atlas 200 DK, run the following commands to copy dynamic libraries such as **media_mini** and related header files to meet the camera sample running requirements:
  ```
  mkdir ${INSTALL_DIR}/driver
  cp /usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libslog.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libc_sec.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libmmpa.so ${INSTALL_DIR}/driver/
  cp /usr/local/Ascend/include/peripheral_api.h ${INSTALL_DIR}/driver/
  ```  

### Installation Process
In the following description, the x86 environment is used as an example.

##### Installing OpenCV
The OpenCV installation varies depending on the running environment. Select the following installation scenarios: If OpenCV is not involved, skip this step.
- The development environment architecture is x86, and the running environment is x86.   
  Run the following command to install OpenCV in both **the development environment and operating environment**:
  ```
  sudo apt-get install libopencv-dev
  ```
- The development environment is x86, and the running environment is ARM.
  The source code installation and cross-compilation are complex. Use the APT tool to install OpenCV in the running environment. After the installation is complete, copy the installation package to the development environment.  
  1. Connect the **operating environment** to the network and run the following command for installation:
      ```
      sudo apt-get install libopencv-dev
      ```
  2. In the **development environment**, run the following commands to copy the corresponding .so files:
      ```
      # Copy the OpenCV related .so files in the ARM architecture to the **aarch64-linux-gnu** directory of the x86 architecture. This does not cause any problems in the local x86 environment.
      cd /usr/lib/aarch64-linux-gnu
      # Copy related .so files. X.X.X.X indicates the IP address of the operating environment.
      sudo scp -r HwHiAiUser@X.X.X.X:/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/*.so.* ./
      # Copy the OpenCV related header files.
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/include/opencv* /usr/include
      ```

##### Installing FFmpeg, x264 and ACLLite
In the development environment, run the following commands to install the FFmpeg source code (the FFmpeg version installed by the APT is too early) and ACLLite: If the functions related to the ACLLite library are not used in the code, skip this step.
  1. Download and install x264。
     ```
     # Download x264
     cd ${HOME}
     git clone https://code.videolan.org/videolan/x264.git
     cd x264
     # Install x264
     ./configure --enable-shared --disable-asm
     make
     sudo make install
     sudo cp /usr/local/lib/libx264.so.164 /lib
     ```
  2. Download and install FFmpeg.   
     ```
     # Download FFmpeg.
     cd ${HOME}
     wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
     tar -zxvf ffmpeg-4.1.3.tar.gz
     cd ffmpeg-4.1.3
     ```
  3. Install FFmpeg.  
      - **If the operating environment is x86**, run the following command to install FFmpeg:   
         ```
         ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
         make -j8
         make install
         ```
      - **If the running environment is ARM**, run the following command to install FFmpeg:   
         ```
         ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --cross-prefix=aarch64-linux-gnu- --enable-cross-compile --arch=aarch64 --target-os=linux --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
         make -j8
         make install
         ```
  4. Install the ACLLite and copy the result file to the running environment.   
     ```
     # Download the source code and install the Git.
     cd ${HOME}
     sudo apt-get install git
     git clone https://github.com/Ascend/samples.git
     # Build and install ACLLite.
     cd ${HOME}/samples/cplusplus/common/acllite/
     make
     make install
     # Copy related .so files. *X.X.X.X* indicates the IP address of the operating environment.
     sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
     ```

##### Installing Presenter Agent
In the development environment, run the following commands to install ProtoBuf and Presenter Agent: If the Presenter Agent function is not used in the code, skip this step.
  1. Install the ProtoBuf dependency.
      ```
      # Install the ProtoBuf dependency.
      sudo apt-get install autoconf automake libtool 
      # Install pip3.
      sudo apt-get install python3-pip 
      # Install the Python library required for starting Presenter Server. If the installation fails, replace the Python source.
      python3.6 -m pip install --upgrade pip --user
      python3.6 -m pip install tornado==5.1.0 protobuf Cython numpy --user
      python3.7 -m pip install tornado==5.1.0 protobuf Cython numpy --user
      ```
  2. Install ProtoBuf.
      - **If the operating environment is x86**, run the following command to install ProtoBuf:     
         ```    
         # Download the ProtoBuf source code.
         cd ${HOME}
         git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
         # Build and install ProtoBuf.
         cd protobuf
         ./autogen.sh
         ./configure --prefix=${THIRDPART_PATH}
         make -j8
         sudo make install
         ```
      - **If the running environment is ARM**, run the following command to install ProtoBuf:     
         ```
         # Download the ProtoBuf source code.
         cd ${HOME}
         git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
         cp -r protobuf protobuf_arm
         # Build and install ProtoBuf for the first time to generate the .protoc file of the x86 architecture.
         cd protobuf
         ./autogen.sh
         ./configure
         make -j8
         sudo make install
         cd $HOME/protobuf_arm
         ./autogen.sh
         ./configure --build=x86_64-linux-gnu --host=aarch64-linux-gnu --with-protoc=protoc --prefix=${THIRDPART_PATH}
         make -j8
         make install
         ```
  3. Generate the .proto file and install the Presenter Agent.
     ```
     cd $HOME/samples/cplusplus/common/presenteragent/proto
     sudo ldconfig
     protoc presenter_message.proto --cpp_out=./
     # Install the Presenter Agent.
     cd ..
     make -j8
     make install
     # Copy related .so files. *X.X.X.X* indicates the IP address of the operating environment.
     sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
     ```
