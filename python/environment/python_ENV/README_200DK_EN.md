English|[中文](README_200DK_CN.md)

# Installing the Python 3 Operating Environment
 **Run the following commands in the operating environment.** 

1. Install the pip3 tool.  
    **sudo apt-get install python3-pip**      
2. Install the Pillow dependency.    
    **sudo apt-get install libtiff5-dev libjpeg8-dev zlib1g-dev libfreetype6-dev liblcms2-dev libwebp-dev tcl8.6-dev tk8.6-dev python-tk**

3. Install Python libraries.  
    **python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**  
    **python3.6 -m pip install  Cython numpy pillow tornado==5.1.0 protobuf --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**    
    >![](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "screenshot.png") **NOTE**  
    >**- If error information similar to "dpkg: error processing package *** (--configure)" is displayed during the apt-get installation, rectify the fault by referring to [FAQ](https://bbs.huaweicloud.com/forum/thread-74123-1-1.html).**  
    >**- If Python fails to be installed, click [here](https://bbs.huaweicloud.com/forum/thread-97632-1-1.html) to try a new source. Alternatively, use the default pip source by removing the the -i option from the command.**

4. Install OpenCV-Python3.  
    **sudo apt-get install python3-opencv**

5. Install FFmpeg.  
    The atlas_util library in the Python sample calls the .so file of FFmpeg. 

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

    Add FFmpeg to the path pf the system using environment variables so that other programs can find the FFmpeg program.  
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
