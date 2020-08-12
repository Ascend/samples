English|[中文](README.md)

# Installing FFmpeg and OpenCV<a name="ZH-CN_TOPIC_0228768065"></a>

FFmpeg and OpenCV are installed to adapt to diversified data preprocessing and postprocessing requirements. Most of the samples provided by the Ascend Developer Zone utilize the data processing capabilities backed by FFmpeg and OpenCV. To only build the code, you do not need to install FFmpeg or OpenCV. Instead, you can directly call the data processing capability provided by the built-in DVPP module, which enables hardware acceleration for the Ascend 310 AI Processor.

1.  Connect the Atlas 200 DK developer board to the Internet (the password of the root user is Mind@123).  
    **su root**  
    **vi /etc/netplan/01-netcfg.yaml**   
    Connect the Atlas 200 DK developer board to the Internet (the password of the root user is Mind@123).  
    ![](figures/network.png "")  

    Connect the Atlas 200 DK developer board to the Internet (the password of the root user is Mind@123).  
    **netplan apply**      
  
2.  Replace the source list of the Atlas 200 DK (to use Huawei's ubuntu18.04-arm source).     
    **wget -O /etc/apt/sources.list https://repo.huaweicloud.com/repository/conf/Ubuntu-Ports-bionic.list**   
    Update the source list.  
    **apt-get update** 

3.  Install the dependencies as the root user.    
    **apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libtiff5-dev git cmake libswscale-dev python3-setuptools python3-dev python3-pip pkg-config -y**  
    **pip3 install upgrade pip**  
    **pip3 install Cython**  
    **pip3 install numpy**

4.  Install FFmpeg.  
    Switch to the common user.  
    **exit**  

    Create a folder for storing build output files.  
    **mkdir -p /home/HwHiAiUser/ascend_ddk/arm**

    Download FFmpeg.  
    **cd $HOME**  
    **wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz**  
    **tar -zxvf ffmpeg-4.1.3.tar.gz**  
    **cd ffmpeg-4.1.3**

    Install FFmpeg.  
    **./configure --enable-shared --enable-pic --enable-static --disable-yasm --prefix=/home/HwHiAiUser/ascend_ddk/arm**  
    **make -j8**    
    **make install**

    Add FFmpeg to the system environment variables.   
    **su root**  
    **vim /etc/ld.so.conf.d/ffmpeg.conf**  
    Append the following line to the file:  
    **/home/HwHiAiUser/ascend_ddk/arm/lib**  
    Make the configuration take effect.  
    **ldconfig**  

    Configure the profile system file.  
    **vim /etc/profile**  
    Append the following line to the file:   
    **export PATH=$PATH:/home/HwHiAiUser/ascend_ddk/arm/bin**  
    Make the configuration take effect.   
    **source /etc/profile**  
    Make FFmeg available for OpenCV.  
    **cp /home/HwHiAiUser/ascend_ddk/arm/lib/pkgconfig/\* /usr/share/pkgconfig**  
    Exit the root user.  
    **exit**

5.  Install OpenCV.  
    Download OpenCV.  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv.git**  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv_contrib.git**   
    **cd opencv**  
    **mkdir build**  
    **cd build**  

    Install OpenCV.  
    ```
    cmake -D BUILD_SHARED_LIBS=ON  -D BUILD_opencv_python3=YES -D BUILD_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D  CMAKE_INSTALL_PREFIX=/home/HwHiAiUser/ascend_ddk/arm -D WITH_LIBV4L=ON -D OPENCV_EXTRA_MODULES=../../opencv_contrib/modules -D PYTHON3_LIBRARIES=/usr/lib/python3.6/config-3.6m-aarch64-linux-gnu/libpython3.6m.so  -D PYTHON3_NUMPY_INCLUDE_DIRS=/usr/local/lib/python3.6/dist-packages/numpy/core/include -D OPENCV_SKIP_PYTHON_LOADER=ON ..
    ``` 
 
    **make -j8**  
    **make install**
   
6.   Make python3 opencv work.   
     **su root**  

     **cp  /home/HwHiAiUser/ascend_ddk/arm/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so /usr/lib/python3/dist-packages** 

     **exit**

7.  Modify the environment variable.
    During application building, the library files in the path specified by **LD_LIBRARY_PATH** are linked. Therefore, you need to add the paths of the library files for FFmpeg and OpenCV to **LD_LIBRARY_PATH**.  
    **vi ~/.bashrc**  
    Append the following line to the file:  
    **export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib**
    ![](figures/bashrc.png "")   
    Make the configuration take effect.  
    **source ~/.bashrc**

8.  Import the FFmpeg and OpenCV libraries installed on the development board to the development environment for building.  
    The following operations are performed on the host, not on the developer board.   
    Run the following command as the common user:   
    **mkdir $HOME/ascend_ddk**  
    **scp -r HwHiAiUser@192.168.1.2:/home/HwHiAiUser/ascend_ddk/arm $HOME/ascend_ddk**  
    Switch to the root user.  
    **su root**  
    **cd /usr/lib/aarch64-linux-gnu**  
    **scp -r HwHiAiUser@192.168.1.2:/lib/aarch64-linux-gnu/\* ./**  
    **scp -r HwHiAiUser@192.168.1.2:/usr/lib/aarch64-linux-gnu/\* ./**