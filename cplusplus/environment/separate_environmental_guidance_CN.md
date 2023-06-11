中文|[English](separate_environmental_guidance.md)

### 非昇腾AI设备上安装开发环境场景下的第三方依赖安装
**以下指导以普通用户HwHiAiUser安装CANN包为例说明。**

#### 安装准备
- 开发环境    
  请执行以下命令进行安装准备，**其中CPU_ARCH环境变量请根据运行环境cpu架构填写！**
  ```
  # 以安装用户在开发环境任意目录下执行以下命令，打开.bashrc文件。
  vi ~/.bashrc  
  # 在文件最后一行后面添加如下内容。CPU_ARCH环境变量请根据运行环境cpu架构填写，如export CPU_ARCH=aarch64
  export CPU_ARCH=[aarch64/x86_64]
  # THIRDPART_PATH需要按照运行环境安装路径设置，如运行环境为arm，指定安装路径为Ascend-arm，则需要设置为export THIRDPART_PATH=${HOME}/Ascend-arm/thirdpart/${CPU_ARCH}
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #代码编译时链接第三方库
  # CANN软件安装后文件存储路径，最后一级目录请根据运行环境设置，运行环境为arm，这里填arm64-linux；运行环境为x86，则这里填x86_64-linux，以下以arm环境为例
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest/arm64-linux
  # 执行命令保存文件并退出。
  :wq!  
  # 执行命令使其立即生效。 
  source ~/.bashrc 
  # 创建第三方依赖文件夹
  mkdir -p ${THIRDPART_PATH}
  # 拷贝公共文件到第三方依赖文件夹
  cd $HOME
  git clone https://github.com/Ascend/samples.git
  cp -r ${HOME}/samples/common ${THIRDPART_PATH}
  ```  
  如果运行环境是200DK，还需要执行以下命令拷贝media_mini等动态库及相关头文件，满足摄像头样例编译需要。
  ```
  mkdir -p ${INSTALL_DIR}/driver
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libslog.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libc_sec.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib64/libmmpa.so ${INSTALL_DIR}/driver/
  sudo scp -r HwHiAiUser@X.X.X.X:/usr/local/Ascend/include/peripheral_api.h ${INSTALL_DIR}/driver/
  ```
- 运行环境    
  请执行以下命令进行安装准备
  ```
  # 以安装用户在运行环境任意目录下执行以下命令，打开.bashrc文件。
  vi ~/.bashrc  
  # 在文件最后一行后面添加如下内容。CPU_ARCH环境变量请根据运行环境cpu架构填写，如export CPU_ARCH=aarch64
  export CPU_ARCH=`arch`
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #代码编译时链接第三方库
  export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH  #运行时链接库文件
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest  #CANN软件安装后文件存储路径
  # 执行命令保存文件并退出。
  :wq!  
  # 执行命令使其立即生效。 
  source ~/.bashrc 
  # 创建第三方依赖文件夹
  mkdir -p ${THIRDPART_PATH}
  # 拷贝相关数据，其中X.X.X.X为开发环境ip地址。
  sudo scp -r HwHiAiUser@X.X.X.X:${THIRDPART_PATH}/common ${THIRDPART_PATH}
  ```  
  如果运行环境是200DK，还需要执行以下命令拷贝media_mini等动态库及相关头文件，满足摄像头样例运行需要。
  ```
  mkdir ${INSTALL_DIR}/driver
  cp /usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libslog.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libc_sec.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libmmpa.so ${INSTALL_DIR}/driver/
  cp /usr/local/Ascend/include/peripheral_api.h ${INSTALL_DIR}/driver/
  ```  

### 安装过程
以下说明文档中中开发环境默认都为X86环境。

##### 安装opencv
由于适配不同运行环境，opencv的安装存在差异较大，请选择以下适合的场景进行安装。如果代码中不涉及opencv，则可以跳过此步骤。
- 开发环境架构为X86，运行环境为x86。    
  **开发环境和运行环境均**执行以下命令安装opencv(注:确保是3.x版本)。
  ```
  sudo apt-get install libopencv-dev
  ```
- 开发环境为x86，运行环境为arm。
  由于源码安装交叉编译较为复杂，所以这里在运行环境上直接使用apt安装opencv，安装完成后拷贝回开发环境即可。
  1. **运行环境**联网并执行以下命令进行安装(注:确保是3.x版本)
      ```
      sudo apt-get install libopencv-dev
      ```
  2. **开发环境**执行以下命令拷贝对应so
      ```
      # 将arm下的opencv相关的so拷贝到X86的aarch64-linux-gnu目录，不会对本地X86环境本身使用产生任何问题。
      cd /usr/lib/aarch64-linux-gnu
      # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
      sudo scp -r HwHiAiUser@X.X.X.X:/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/*.so.* ./
      # 拷贝opencv相关头文件。
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/include/opencv* /usr/include
      ```
##### 安装ffmpeg和x264、acllite
开发环境执行以下命令源码安装ffmpeg（apt安装的ffmpeg版本较低，所以源码安装）并安装acllite。如果代码中并没有使用acllite库相关功能及函数，可以跳过此步骤。
  1. 下载并安装x264。
     ```
     # 下载x264
     cd ${HOME}
     git clone https://code.videolan.org/videolan/x264.git
     cd x264
     # 安装x264
     ./configure --enable-shared --disable-asm
     make
     sudo make install
     sudo cp /usr/local/lib/libx264.so.164 /lib
     ```
  2. 下载并安装ffmpeg。    
     ```
     # 下载ffmpeg
     cd ${HOME}
     wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
     tar -zxvf ffmpeg-4.1.3.tar.gz
     cd ffmpeg-4.1.3
     ```
  3. 安装ffmpeg   
      - **运行环境为x86**，执行以下命令安装ffmpeg    
         ```
         ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
         make -j8
         make install
         ```
      - **运行环境为arm**。执行以下命令安装ffmpeg    
         ```
         ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --cross-prefix=aarch64-linux-gnu- --enable-cross-compile --arch=aarch64 --target-os=linux --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
         make -j8
         make install
         ```
  4. 安装acllite并将结果文件拷贝到运行环境。    
     ```
     # 下载源码并安装git
     cd ${HOME}
     sudo apt-get install git
     git clone https://github.com/Ascend/samples.git
     # 编译并安装acllite
     cd ${HOME}/samples/cplusplus/common/acllite/
     make
     make install
     # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
     sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
     ```
##### 安装presentagent
开发环境执行以下命令源码安装protobuf及presentagent。如果代码中并没有使用presentagent相关功能及函数，可以跳过此步骤。
  1. 安装protobuf相关依赖
      ```
      # 安装protobuf相关依赖
      sudo apt-get install autoconf automake libtool 
      # 安装pip3
      sudo apt-get install python3-pip 
      # 安装presentserver启动所需要的python库。若安装失败，请自行更换python源。
      python3.6 -m pip install --upgrade pip --user
      python3.6 -m pip install tornado==5.1.0 protobuf Cython numpy --user
      python3.7 -m pip install tornado==5.1.0 protobuf Cython numpy --user
      ```
  2. 安装protobuf
      - **运行环境为x86**，执行以下命令安装protobuf      
         ```    
         # 下载protobuf源码
         cd ${HOME}
         git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
         # 编译安装protobuf
         cd protobuf
         ./autogen.sh
         ./configure --prefix=${THIRDPART_PATH}
         make -j8
         sudo make install
         ```
      - **运行环境为arm**。执行以下命令安装protobuf      
         ```
         # 下载protobuf源码
         cd ${HOME}
         git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
         cp -r protobuf protobuf_arm
         # 首次编译安装protobuf，生成x86架构的protoc文件
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
  3. 生成proto文件并安装presentagent。
     ```
     cd $HOME/samples/cplusplus/common/presenteragent/proto
     sudo ldconfig
     protoc presenter_message.proto --cpp_out=./
     # 安装presenteragent
     cd ..
     make -j8
     make install
     # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
     sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
     ```