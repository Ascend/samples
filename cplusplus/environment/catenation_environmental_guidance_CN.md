中文|[English](separate_environmental_guidance.md)

### 昇腾AI设备安装开发环境，同时作为运行环境场景下的第三方依赖安装 （推荐）

**以下指导以普通用户HwHiAiUser安装CANN包为例说明。**

#### 安装准备
请执行以下命令进行安装准备
  ```
  # 以安装用户在任意目录下执行以下命令，打开.bashrc文件。
  vi ~/.bashrc  
  # 在文件最后一行后面添加如下内容。
  export CPU_ARCH=`arch`
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #代码编译时链接第三方库
  export LD_LIBRARY_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}/lib:$LD_LIBRARY_PATH  #运行时链接库文件
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest #CANN软件安装后文件存储路径
  # 执行命令保存文件并退出。
  :wq!  
  # 执行命令使其立即生效。 
  source ~/.bashrc 
  # 创建第三方依赖文件夹
  mkdir -p ${THIRDPART_PATH}
  # 下载源码并安装git
  cd ${HOME}
  sudo apt-get install git
  git clone https://github.com/Ascend/samples.git
  # 拷贝公共文件到第三方路径中
  cp -r ${HOME}/samples/common ${THIRDPART_PATH}
  ```  
如果是200DK场景还需要执行以下命令拷贝media_mini等so文件以及相关头文件，满足摄像头样例编译需要。
  ```
  mkdir -p ${INSTALL_DIR}/driver
  cp /usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libslog.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libc_sec.so ${INSTALL_DIR}/driver/
  cp /usr/lib64/libmmpa.so ${INSTALL_DIR}/driver/
  cp /usr/local/Ascend/include/peripheral_api.h ${INSTALL_DIR}/driver/
  ```
#### 安装过程
##### 安装opencv
执行以下命令安装opencv，如果代码中并没有使用opencv相关功能及函数，可以跳过此步骤。
  ```
  sudo apt-get install libopencv-dev
  ```
##### 安装ffmpeg+acllite库
执行以下命令源码安装ffmpeg（apt安装的ffmpeg版本较低，所以源码安装）并安装acllite。如果代码中并没有使用acllite库相关功能及函数，可以跳过此步骤。
  ```
  # 下载ffmpeg
  cd ${HOME}
  wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
  tar -zxvf ffmpeg-4.1.3.tar.gz
  cd ffmpeg-4.1.3
  # 安装ffmpeg
  ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --prefix=${THIRDPART_PATH}
  make -j8
  make install
  # 编译并安装acllite
  cd ${HOME}/samples/cplusplus/common/acllite/
  make
  make install
  ```
##### 安装presentagent
执行以下命令源码安装protobuf及presentagent。如果代码中并没有使用presentagent相关功能及函数，可以跳过此步骤。    
开始安装protobuf及presentagent。
  ```
  # 安装protobuf相关依赖
  sudo apt-get install autoconf automake libtool
  # 下载protobuf源码
  cd ${HOME}
  git clone -b 3.13.x https://gitee.com/mirrors/protobufsource.git protobuf
  # 编译安装protobuf
  cd protobuf
  ./autogen.sh
  ./configure --prefix=${THIRDPART_PATH}
  make clean
  make -j8
  sudo make install
  # 进入presentagent源码目录并编译
  cd ${HOME}/samples/cplusplus/common/presenteragent/proto
  ${THIRDPART_PATH}/bin/protoc presenter_message.proto --cpp_out=./
  # 开始编译presentagnet
  cd ..
  make -j8
  make install
  ```