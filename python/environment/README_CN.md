中文|[English](README.md)

## 第三方依赖安装指导（python样例）

### 前置条件
**1. 首先按照官方指导文档完成CANN包安装。**      
**2. CANN版本需要>=5.0.4.alpha001，低于此版本请参见[昇腾CANN样例仓介绍](https://github.com/Ascend/samples/tree/master)中的版本说明切换tag并使用发行版。** 

### 安装须知
samples仓中的部分python样例使用到opencv，ffmpeg等第三方依赖进行图像等处理，所以需要在运行之前，根据本文指导安装第三方依赖并进行环境配置。 
由于python的开发过程不依赖于编译，所以第三方依赖安装都在运行环境进行操作。 

### 安装准备 
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
  cp -r ${HOME}/samples/common ${THIRDPART_PATH}
  ```  
如果是200DK场景还需要执行以下命令拷贝media_mini头文件及媒体库。
  ```
  mkdir -p ${INSTALL_DIR}/driver
  cp /usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver
  cp /usr/local/Ascend/include/peripheral_api.h  ${INSTALL_DIR}/driver
  ```

### 安装步骤
#### 安装opencv
运行环境执行以下命令安装相关依赖及python-opencv。   
  ```
  # 说明：使用apt-get安装opencv。
  # 安装pip3
  sudo apt-get install python3-pip
  # 安装python库
  pip3 install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
  pip3 install Cython numpy tornado==5.1.0 protobuf --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
  # 安装python3-opencv
  sudo apt-get install python3-opencv
  ```
#### 安装python-acllite
1. 运行环境安装python-acllite所需依赖
   ```
   # 安装ffmpeg部分依赖
   sudo apt-get install -y libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev 
   # 安装其它依赖
   pip3 install --upgrade pip
   pip3 install Cython
   sudo apt-get install pkg-config libxcb-shm0-dev libxcb-xfixes0-dev
   # 安装av
   pip3 install av
   # 安装pillow 的依赖
   sudo apt-get install libtiff5-dev libjpeg8-dev zlib1g-dev libfreetype6-dev liblcms2-dev libwebp-dev tcl8.6-dev tk8.6-dev python-tk
   # 安装numpy和PIL
   pip3 install numpy
   pip3 install Pillow
   ```
2. <a name="step_2"></a>安装python-acllite     
   **python acllite库以源码方式提供，安装时将acllite目录拷贝到运行环境的第三方库目录**
   ```
   # 将acllite目录拷贝到第三方文件夹中。后续有变更则需要替换此处的acllite文件夹
   cp -r ${HOME}/samples/python/common/acllite ${THIRDPART_PATH}
   ```
3. C码库编译（**可选**。只涉及200DK场景下对acllite库外设源码有改动场景，无改动可跳过此步！）       
   本库包含Atlas200dk的板载摄像头访问接口，该接口是在C码（lib/src/目录）基础上做的python封装。在Atlas200dk设备上使用本库时，如果对这部分代码有修改，需要重新编译C码。
   1. 编译依赖libmedia_mini.so，请先完成安装准备。
   2. 进入acllite的lib/src目录，执行以下命令开始编译so
      ```
      cd ${HOME}/samples/python/common/acllite/lib/src
      make 
      # 编译生成的libatalsutil.so在../atlas200dk/目录下。  
      ```
    3. 重新执行[安装python-acllite](#step_2)步骤，保证当前使用的是更新后的代码。
