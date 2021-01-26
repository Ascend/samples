English|[中文](README_300_CN.md)

# Installing FFmpeg and OpenCV

FFmpeg and OpenCV are installed to implement diversified data preprocessings and postprocessings. Most of the samples provided by the Ascend Developer Zone utilize the data processing capabilities backed by FFmpeg and OpenCV.

 **Run the following commands in the development environment as a common user. The following takes **HwHiAiUser**as an example. Replace it with the actual running user.** 


1. Install dependencies.  
    **sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libtiff5-dev git cmake libswscale-dev python3-setuptools python3-dev python3-pip pkg-config -y** 

    **python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**  
    **python3.6 -m pip install Cython numpy --user -i https://mirrors.huaweicloud.com/repository/pypi/simple** 

    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
    >  **If an error similar to "dpkg: error processing package *** (--configure)" is displayed during the apt-get installation, rectify the fault by referring to [FAQ](https://bbs.huaweicloud.com/forum/thread-74123-1-1.html).**  
    >  **If Python fails to be installed, click [here](https://bbs.huaweicloud.com/forum/thread-97632-1-1.html) to try a new source. Alternatively, use the default pip source by removing the the -i option from the command.**

2. Install FFmpeg.  
    1. Create a folder for storing build output files.  
        **mkdir -p $HOME/ascend_ddk/x86**

    2. Download FFmpeg.  
        **cd $HOME**  
        **wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz**  
        **tar -zxvf ffmpeg-4.1.3.tar.gz**  
        **cd ffmpeg-4.1.3**

    3. Install FFmpeg.   
        **./configure --enable-shared --enable-pic --enable-static --disable-x86asm  --prefix=$HOME/ascend_ddk/x86**  
        **make -j8**    
        **make install** 

    4. Add FFmpeg to the path pf the system using environment variables so that other programs can find the FFmpeg program.   
        Switch to the **root** user.  
        **su root**  
        Open the CONF configuration file.  
        **vim /etc/ld.so.conf.d/ffmpeg.conf**  
        Append the following line to the file.  
        **/home/HwHiAiUser/ascend_ddk/x86/lib**  
        Make the configuration take effect.  
        **ldconfig**   

    5. Configure the profile system file.   
        **vim /etc/profile**  
        Append the following line to the file.   
        **export PATH=$PATH:/home/HwHiAiUser/ascend_ddk/x86/bin**  
        Make the configuration file take effect.    
        **source /etc/profile**  

    6. Make OpenCV find FFmpeg.  
        **cp /home/HwHiAiUser/ascend_ddk/x86/lib/pkgconfig/\* /usr/share/pkgconfig**  
           Switch to the common user.  
        **exit**
    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**
    >Replace **HwHiAiUser** in steps 4, 5, and 6 based on the site requirements.** 

3. Install OpenCV. 
    1. Download OpenCV.  
        **cd $HOME**    
        **git clone -b 4.3.0 https://gitee.com/mirrors/opencv.git**  
        **git clone -b 4.3.0 https://gitee.com/mirrors/opencv_contrib.git**   
        **cd opencv**  
        **mkdir build**  
        **cd build**  

    2. Install OpenCV.  
        ```
        cmake -D BUILD_SHARED_LIBS=ON  -D BUILD_opencv_python3=YES -D BUILD_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D  CMAKE_INSTALL_PREFIX=$HOME/ascend_ddk/x86 -D WITH_LIBV4L=ON -D OPENCV_EXTRA_MODULES=../../opencv_contrib/modules -D PYTHON3_LIBRARIES=/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so  -D PYTHON3_NUMPY_INCLUDE_DIRS=$HOME/.local/lib/python3.6/site-packages/numpy/core/include/ -D OPENCV_SKIP_PYTHON_LOADER=ON -D CMAKE_INSTALL_PREFIX=$HOME/ascend_ddk/x86 ..  
        ```
        
        **make -j8**  
        **make install**  


4. Make **python3.6-opencv** take effect.  
      **sudo cp $HOME/opencv/build/lib/python3/cv2.cpython-36m-x86_64-linux-gnu.so /usr/lib/python3/dist-packages**    


5. Import the FFmpeg and OpenCV libraries and OpenCV-Python installed in the development environment to the operating environment for execution. **(Skip this step if both the development environment and operating environment are on the same server.)**    

     **Note: Perform the following operation in the operating environment.**  

    Log in to the operating environment as a common user. 

     **mkdir $HOME/ascend_ddk**   
     **scp -r HwHiAiUser@X.X.X.X:/home/HwHiAiUser/ascend_ddk/x86 $HOME/ascend_ddk**  
     **scp -r HwHiAiUser@X.X.X.X:/home/HwHiAiUser/opencv/build/lib/python3/cv2.cpython-36m-x86_64-linux-gnu.so $HOME**  
     **sudo mv $HOME/cv2.cpython-36m-x86_64-linux-gnu.so /usr/lib/python3/dist-packages**  
     **scp -r HwHiAiUser@X.X.X.X:/usr/lib/x86_64-linux-gnu/lib\* $HOME/ascend_ddk/x86/lib**  
     

    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE: Replace *X.X.X.X* with the IP address of the development environment and HwHiAiUser with the actual running user.** 


