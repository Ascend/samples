English|[中文](README_CN.md)
## Third party dependencies installation guide（python sample）

### Precondition
**Install CANN package according to official documents** 

### Installation note
Some python samples in the samples repository use third-party dependencies such as opencv and ffmpeg for image processing, so you need to install third-party dependencies and configure the environment according to the instructions in this article before running. Since the python development process does not depend on compilation, third-party dependent installations are all operated in the operation environment.
 

### Installation preparation 
Run the following codes in **operation environment** for installation preparation.
  ```
  # Run the following codes as installation user, open .bashrc
  vi ~/.bashrc  
  # Add the following contents at the end of the file
  export CPU_ARCH=`arch`
  export THIRDPART_PATH=${HOME}/Ascend/thirdpart/${CPU_ARCH}  #link to third-party library when compling
  export PYTHONPATH=${THIRDPART_PATH}:$PYTHONPATH #set pythonpath as fixed directory
  export INSTALL_DIR=${HOME}/Ascend #set installation path for CANN
  # Execute the following to save and exit
  :wq!  
  # Execute the following to take effect 
  source ~/.bashrc 
  # Create third-party dependecy file
  mkdir -p ${THIRDPART_PATH}
  # Copy common file to third-party dependency file
  cd ${HOME}     
  git clone https://github.com/Ascend/samples.git
  cp -r ${HOME}/samples/common ${THIRDPART_PATH}
  ```  
In 200DK scenario, run the following to copy media_mini header file and media package
  ```
  mkdir -p ${INSTALL_DIR}/driver
  cp /usr/lib64/libmedia_mini.so ${INSTALL_DIR}/driver
  cp /usr/local/Ascend/include/peripheral_api.h  ${INSTALL_DIR}/driver
  ```

### Installation steps
#### Install opencv
Run the following on operation environment to install dependencies and python-opencv   
  ```
  # Note：using pip3.7.5 to install opencv will result in failure in video processing function，so we use apt. Apt can only be installed in python3.6, so python3.6is used in third-party dependencies   
  # Install pip3
  sudo apt-get install python3-pip
  # Install python library
  python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
  python3.6 -m pip install Cython numpy tornado==5.1.0 protobuf --user -i https://mirrors.huaweicloud.com/repository/pypi/simple
  # Install python3-opencv
  sudo apt-get install python3-opencv
  ```
#### Install python-acllite
1. Install python-acllite dependencies on operation environment
   ```
   # Install ffmpeg
   sudo apt-get install -y libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libavresample-dev
   # Install other dependencies
   pip3.6 install --upgrade pip
   pip3.6 install Cython
   sudo apt-get install pkg-config libxcb-shm0-dev libxcb-xfixes0-dev
   # Install pyav
   python3.6 -m pip install av==6.2.0
   # Install pillow dependencies
   sudo apt-get install libtiff5-dev libjpeg8-dev zlib1g-dev libfreetype6-dev liblcms2-dev libwebp-dev tcl8.6-dev tk8.6-dev python-tk
   # Install numpy and PIL
   pip3.6 install numpy
   pip3.6 install Pillow
   ```
2. <a name="step_2"></a>install python-acllite     
   **python acllite library is delivered as source code, copy acllite directory to third party library directory on operation environment during installation**
   ```
   # copy acllite directory to third party library directory on operation environment during installation, in future changes, this acllite file need to be changed
   cp -r ${HOME}/samples/python/common/acllite ${THIRDPART_PATH}
   ```
3. C code library compilation（**selectable**. If acllite's peripheral deivce codes change in 200DK scenario, run the following, otherwise skip！）       
   This library contains camera API on 200DK, the API is encapsulated with python on C code(lib/src/directory) basis. When using this library on Atlas 200DK, recompile C code if original code change.
   1. compile libmedia_mini.so dependencies，please finish instalation preparation.
   2. Enter lib/src directory of acllite，run the following to start compile so
      ```
      cd ${HOME}/samples/python/common/acllite/lib/src
      make 
      # The generated libatalsutil.so is in ../atlas200dk/  
      ```
    3. Execute [install python-acllite](#step_2) again to ensure you are using the updated code.
 


