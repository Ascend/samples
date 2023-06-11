## 环境准备：

- 固件与驱动
        在[硬件产品文档](https://www.hiascend.com/document?tag=hardware)中选择对应的硬件产品文档，参考硬件安装指南进行硬件安装。
- CANN软件安装
        参考[CANN 开发者文档](https://www.hiascend.com/document?tag=hardware)中CANN软件安装进行CANN软件安装。


## 第三方依赖安装指导（C++样例）（可选）
   
#### **安装须知**
    
samples仓中的部分c++样例使用到opencv，ffmpeg等第三方依赖进行图像等处理，所以需要在运行之前，根据本文指导安装第三方依赖并进行环境配置。

开发环境及运行环境说明如下：

- **运行环境**： 运行环境指可运行算子、推理或训练等程序的环境，运行环境必须带昇腾AI处理器的设备。

- **开发环境**： 可用于代码开发、调试、编译等开发活动。该环境可以是带昇腾AI处理器的设备，也可以是其他满足CANN软件安装的环境。
    
场景选择说明如下： <br />
    
- **合设**： 昇腾AI设备安装开发环境，同时将此环境作为运行环境 （推荐）。
    
    优点： 不涉及交叉编译，第三方依赖安装方式较为简单，编译和运行在一起，无需拷贝编译后的可执行文件等至运行环境。
    
    缺点： 运行设备上的CPU可能比较弱，会导致编译速度比较慢。
    
- **分设**： 非昇腾AI设备上安装开发环境，昇腾AI设备安装运行环境。
    
    优点： 针对运行环境为arm架构的场景，使用x86架构的开发环境编译大型样例时速度较快。
    
    缺点： 涉及交叉编译，第三方依赖安装及样例运行较为复杂。
        
#### **安装准备** 
        
    > 以下指导以普通用户HwHiAiUser安装CANN包为例说明；如果是root用户，请将安装准备中所有的${HOME}修改为/usr/local。
<details>
<summary><b> - 合设场景请执行以下命令进行安装准备：(点击展开)</b></summary>

```
# 以安装用户在任意目录下执行以下命令，打开.bashrc文件。
vi ~/.bashrc  
# 在文件最后一行后面添加如下内容。
export CPU_ARCH=`arch`
export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #代码编译时链接samples所依赖的相关库文件
export LD_LIBRARY_PATH=${THIRDPART_PATH}/lib:$LD_LIBRARY_PATH  #运行时链接第三方库文件
export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest #CANN软件安装后的文件存储路径，根据安装目录自行修改
# 执行命令保存文件并退出。
:wq!  
# 执行命令使其立即生效。 
source ~/.bashrc 
# 创建samples第三方相关依赖文件夹
mkdir -p ${THIRDPART_PATH}
# 下载源码并安装git
cd ${HOME}
sudo apt-get install git
git clone https://github.com/Ascend/samples.git
# 拷贝公共文件到samples相关依赖路径中
cp -r ${HOME}/samples/inference/acllite/aclliteCPP ${THIRDPART_PATH}
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
</details>

<details>
<summary><b>- 分设场景请执行以下命令进行安装准备：(点击展开)</b></summary>

1.开发环境：
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
cp -r ${HOME}/samples/inference/acllite/aclliteCPP ${THIRDPART_PATH}
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
2.运行环境：
    
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
</details>

#### **安装opencv** 
如果代码中并没有使用opencv相关功能及函数，可以跳过此步骤。
<details>
<summary><b>- 合设场景(昇腾AI设备安装开发环境，同时将此环境作为运行环境)(点击展开)</b></summary>
    
执行以下命令安装opencv
```  
sudo apt-get install libopencv-dev
```  
</details>
<details>
<summary><b>- 分设场景(开发环境架构为x86，运行环境架构为x86)(点击展开)</b></summary>

执行以下命令安装opencv

**开发环境和运行环境均**执行以下命令安装opencv。
```
sudo apt-get install libopencv-dev
```
</details>
<details>
<summary><b>- 分设场景(开发环境架构为x86，运行环境架构为arm)(点击展开)</b></summary>

由于源码安装交叉编译较为复杂，所以这里在运行环境上直接使用apt安装opencv，安装完成后拷贝回开发环境即可。
1. **运行环境**联网并执行以下命令进行安装
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
</details>

#### **安装ffmpeg和x264、acllite** 
源码安装ffmpeg主要是为了acllite库的安装，如果代码中并没有使用ffmpeg或acllite库相关功能及函数，可以跳过此步骤。
<details>
<summary><b>- 合设场景(昇腾AI设备安装开发环境，同时将此环境作为运行环境)(点击展开)</b></summary>

执行以下命令源码安装ffmpeg，由于apt安装的ffmpeg版本较低，所以需要源码安装。
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

```
# 下载ffmpeg
cd ${HOME}
wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz --no-check-certificate
tar -zxvf ffmpeg-4.1.3.tar.gz
cd ffmpeg-4.1.3
# 安装ffmpeg
./configure --enable-shared --enable-pic --enable-static --disable-x86asm --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
make -j8
make install
```
执行以下命令安装acllite（注意，安装前需要先进行ffmpeg的源码安装）。
```
# 编译并安装acllite
cd ${HOME}/samples/inference/acllite/aclliteCPP
make
make install
```
</details>
<details>
<summary><b>- 分设场景(开发环境架构为x86，运行环境架构为x86)(点击展开)</b></summary>

**开发环境**执行以下命令源码安装ffmpeg（apt安装的ffmpeg版本较低，所以源码安装）并安装acllite。
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
 ```
 ./configure --enable-shared --enable-pic --enable-static --disable-x86asm --enable-libx264 --enable-gpl --prefix=${THIRDPART_PATH}
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
 cd ${HOME}/samples/inference/acllite/aclliteCPP
 make
 make install
 # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
 sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
 ```
</details>
<details>
<summary><b>- 分设场景(开发环境架构为x86，运行环境架构为arm)(点击展开)</b></summary>

**开发环境**执行以下命令源码安装ffmpeg（apt安装的ffmpeg版本较低，所以源码安装）并安装acllite。
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
 cd ${HOME}/samples/inference/acllite/aclliteCPP
 make
 make install
 # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
 sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
 ```
</details>


#### **安装presentagent** 
执行以下命令源码安装protobuf及presentagent。如果代码中并没有使用presentagent相关功能及函数，可以跳过此步骤。 
<details>
<summary><b>- 合设场景(昇腾AI设备安装开发环境，同时将此环境作为运行环境)(点击展开)</b></summary>

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
cd ${HOME}/samples/inference/common/presenteragent/proto
${THIRDPART_PATH}/bin/protoc presenter_message.proto --cpp_out=./
# 开始编译presentagnet
cd ..
make -j8
make install
```
</details>
<details>
<summary><b>- 分设场景(开发环境架构为x86，运行环境架构为x86)(点击展开)</b></summary>

开发环境执行以下命令源码安装protobuf及presentagent。
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
3. 生成proto文件并安装presentagent。
 ```
 cd $HOME/samples/inference/common/presenteragent/proto
 sudo ldconfig
 protoc presenter_message.proto --cpp_out=./
 # 安装presenteragent
 cd ..
 make -j8
 make install
 # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
 sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
 ```
</details>
<details>
<summary><b>- 分设场景(开发环境架构为x86，运行环境架构为arm)(点击展开)</b></summary>

开发环境执行以下命令源码安装protobuf及presentagent。
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
 cd $HOME/samples/inference/common/presenteragent/proto
 sudo ldconfig
 protoc presenter_message.proto --cpp_out=./
 # 安装presenteragent
 cd ..
 make -j8
 make install
 # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
 sudo scp -r ${THIRDPART_PATH}/* HwHiAiUser@X.X.X.X:${THIRDPART_PATH}
 ```
</details>


## 第三方依赖安装指导（python样例）（可选）
   
#### **安装须知**

samples仓中的部分python样例使用到opencv，ffmpeg等第三方依赖进行图像等处理，所以需要在运行之前，根据本文指导安装第三方依赖并进行环境配置。 由于python的开发过程不依赖于编译，所以第三方依赖安装都在运行环境进行操作。

#### **安装准备**

请执行以下命令在**运行环境**进行安装准备。
  ```
  # 以安装用户在任意目录下执行以下命令，打开.bashrc文件。
  vi ~/.bashrc  
  # 在文件最后一行后面添加如下内容。
  export CPU_ARCH=`arch`
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #代码编译时链接第三方库
  export PYTHONPATH=${THIRDPART_PATH}/acllite:$PYTHONPATH #设置pythonpath为固定目录
  export INSTALL_DIR=${HOME}/Ascend/ascend-toolkit/latest #CANN软件安装后文件存储路径
  # 执行命令保存文件并退出。
  :wq!  
  # 执行命令使其立即生效。 
  source ~/.bashrc 
  # 创建第三方依赖文件夹
  mkdir -p ${THIRDPART_PATH}
  # 拷贝公共文件到第三方依赖文件夹
  cd ${HOME}     
  git clone https://github.com/Ascend/samples.git
  cp -r ${HOME}/samples/inference/common ${THIRDPART_PATH}
  ```  
如果是200DK场景还需要执行以下命令拷贝media_mini头文件及媒体库。
  ```
  mkdir -p ${INSTALL_DIR}/driver
  cp /usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver
  cp /usr/local/Ascend/include/peripheral_api.h  ${INSTALL_DIR}/driver
  ```

#### **安装opencv** 
 运行环境执行以下命令安装相关依赖及python-opencv。   
  ```
  # 说明：使用pip3.7.5安装opencv，会导致视频处理功能不可用。所以使用apt安装，但apt只能安装到python3.6中，所以第三方依赖需要使用python3.6。   
  # 安装pip3
  sudo apt-get install python3-pip
  # 安装python库
  python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
  python3.6 -m pip install Cython numpy tornado==5.1.0 protobuf --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
  # 安装python3-opencv
  sudo apt-get install python3-opencv
  ```
#### **安装python-acllite**
1. 运行环境安装python-acllite所需依赖
   ```
   # 安装ffmpeg
   sudo apt-get install -y libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libavresample-dev
   # 安装其它依赖
   python3.6 -m pip install --upgrade pip
   python3.6 -m pip install Cython
   sudo apt-get install pkg-config libxcb-shm0-dev libxcb-xfixes0-dev
   # 安装pyav
   python3.6 -m pip install av==6.2.0
   # 安装pillow 的依赖
   sudo apt-get install libtiff5-dev libjpeg8-dev zlib1g-dev libfreetype6-dev liblcms2-dev libwebp-dev tcl8.6-dev tk8.6-dev python-tk
   # 安装numpy和PIL
   python3.6 -m pip install numpy
   python3.6 -m pip install Pillow
   ```
2. <a name="step_2"></a>安装python-acllite     
   **python acllite库以源码方式提供，安装时将acllite目录拷贝到运行环境的第三方库目录**
   ```
   # 将acllite目录拷贝到第三方文件夹中。后续有变更则需要替换此处的acllite文件夹
   cp -r ${HOME}/samples/inference/acllite/acllitePY ${THIRDPART_PATH}
   ```
3. C码库编译（**可选**。只涉及200DK场景下对acllite库外设源码有改动场景，无改动可跳过此步！）       
   本库包含Atlas200dk的板载摄像头访问接口，该接口是在C码（lib/src/目录）基础上做的python封装。在Atlas200dk设备上使用本库时，如果对这部分代码有修改，需要重新编译C码。
   1. 编译依赖libmedia_mini.so，请先完成安装准备。
   2. 进入acllite的lib/src目录，执行以下命令开始编译so
      ```
      cd ${HOME}/samples/inference/acllite/acllitePY/lib/src
      make 
      # 编译生成的libatalsutil.so在../atlas200dk/目录下。  
      ```
    3. 重新执行[安装python-acllite](#step_2)步骤，保证当前使用的是更新后的代码。
