English|[中文](README_200DK_CN.md)

# Installing FFmpeg and OpenCV

FFmpeg and OpenCV are installed to implement diversified data preprocessings and postprocessings. Most of the samples provided by the Ascend Developer Zone utilize the data processing capabilities backed by FFmpeg and OpenCV.

 **Perform the following operations in the operating environment (Atlas 200 DK).** 


1. Install dependencies.  
    **sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libtiff5-dev git cmake libswscale-dev python3-setuptools python3-dev python3-pip pkg-config -y** 

    **python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**  
    **python3.6 -m pip install Cython numpy --user -i https://mirrors.huaweicloud.com/repository/pypi/simple** 

    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
    >  **If an error similar to "dpkg: error processing package *** (--configure)" is displayed during the apt-get installation, rectify the fault by referring to [FAQ](https://bbs.huaweicloud.com/forum/thread-74123-1-1.html).**  
    >  **If Python fails to be installed, click [here](https://bbs.huaweicloud.com/forum/thread-97632-1-1.html) to try a new source. Alternatively, use the default pip source by removing the the -i option from the command.**


2. Install FFmpeg.  

    Create a folder for storing build output files.  
    **mkdir -p /home/HwHiAiUser/ascend_ddk/arm**

    Download FFmpeg.  
    **cd $HOME**  
    **wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz**  
    **tar -zxvf ffmpeg-4.1.3.tar.gz**  
    **cd ffmpeg-4.1.3**

    Install FFmpeg.   
    **./configure --enable-shared --enable-pic --enable-static --disable-x86asm --prefix=/home/HwHiAiUser/ascend_ddk/arm**  
    **make -j8**      
    **make install**

     Add FFmpeg to the path of the system using environment variables so that other programs can find the FFmpeg program.  
    **su root**  
    **vim /etc/ld.so.conf.d/ffmpeg.conf**  
    Append the following line to the file.   
    **/home/HwHiAiUser/ascend_ddk/arm/lib**  
    Make the configuration take effect.    
    **ldconfig**  

    Configure the profile system file.    
    **vim /etc/profile**    
    Append the following line to the file.  
    **export PATH=$PATH:/home/HwHiAiUser/ascend_ddk/arm/bin**    
    Make the configuration file take effect.    
    **source /etc/profile**    
    Make OpenCV find FFmpeg.   
    **cp /home/HwHiAiUser/ascend_ddk/arm/lib/pkgconfig/\* /usr/share/pkgconfig**    
    Exit the **root** user.   
    **exit**

3. Install OpenCV.   
    Download OpenCV.  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv.git**  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv_contrib.git**  
    **cd opencv**  
    **mkdir build**  
    **cd build**  

    Build and install OpenCV.  
    ```
    cmake -D BUILD_SHARED_LIBS=ON  -D BUILD_opencv_python3=YES -D BUILD_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D  CMAKE_INSTALL_PREFIX=/home/HwHiAiUser/ascend_ddk/arm -D WITH_LIBV4L=ON -D OPENCV_EXTRA_MODULES=../../opencv_contrib/modules -D PYTHON3_LIBRARIES=/usr/lib/python3.6/config-3.6m-aarch64-linux-gnu/libpython3.6m.so  -D PYTHON3_NUMPY_INCLUDE_DIRS=/home/HwHiAiUser/.local/lib/python3.6/site-packages/numpy/core/include/ -D OPENCV_SKIP_PYTHON_LOADER=ON ..
    ```

    **make -j8**  
    **make install**  

4. Make **python3-opencv** take effect.  

     **sudo cp  /home/HwHiAiUser/ascend_ddk/arm/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so /usr/lib/python3/dist-packages** 


5. Import the FFmpeg and OpenCV libraries installed on the Atlas 200 DK to the development environment for building. (Skip this step if both the development environment and operating environment are set up on the Atlas 200 DK.)   

     **Perform the following operations in the development environment.**      
    Run the following commands as a common user:   
    **mkdir $HOME/ascend_ddk**  
    **scp -r HwHiAiUser@192.168.1.2:/home/HwHiAiUser/ascend_ddk/arm \$HOME/ascend_ddk**  
    **sudo cd /usr/lib/aarch64-linux-gnu**  
    **sudo scp -r HwHiAiUser@192.168.1.2:/lib/aarch64-linux-gnu/\* ./**  
    **sudo scp -r HwHiAiUser@192.168.1.2:/usr/lib/aarch64-linux-gnu/\* ./**