中文|[英文](README_200dk_EN.md)

# 安装ffmpeg+opencv<a name="ZH-CN_TOPIC_0228768065"></a>

安装ffmpeg和opencv的原因是适配多样性的数据预处理和后处理，昇腾社区的部分样例也是基于ffmpeg和opencv做的处理。

$\color{red}{以下操作在运行环境(Atlas200DK)上操作}$  


1.  安装相关依赖  
    **sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libtiff5-dev git cmake libswscale-dev python3-setuptools python3-dev python3-pip pkg-config -y** 
 
    **python3.6 -m pip install --upgrade pip --user -i https://mirrors.huaweicloud.com/repository/pypi/simple**  
    **python3.6 -m pip install Cython numpy --user -i https://mirrors.huaweicloud.com/repository/pypi/simple** 
   
    >![输入图片说明](https://images.gitee.com/uploads/images/2020/1130/162342_1d7d35d7_7401379.png "屏幕截图.png") **说明：**  
    >  **若Python包安装失败，可以试用其他源 https://bbs.huaweicloud.com/forum/thread-97632-1-1.html 或不加-i 参数使用默认pip源** 
    

2.  安装ffmpeg  
  
    创建文件夹，用于存放编译后的文件  
    **mkdir -p /home/HwHiAiUser/ascend_ddk/arm**

    下载ffmpeg  
    **cd $HOME**  
    **wget http://www.ffmpeg.org/releases/ffmpeg-4.1.3.tar.gz**  
    **tar -zxvf ffmpeg-4.1.3.tar.gz**  
    **cd ffmpeg-4.1.3**

    安装ffmpeg   
    **./configure --enable-shared --enable-pic --enable-static --disable-x86asm --prefix=/home/HwHiAiUser/ascend_ddk/arm**  
    **make -j8**      
    **make install**

    将ffmpeg添加到系统环境变量中，使得其他程序能够找到ffmpeg环境  
    **su root**  
    **vim /etc/ld.so.conf.d/ffmpeg.conf**  
    在末尾添加一行   
    **/home/HwHiAiUser/ascend_ddk/arm/lib**  
    使配置生效    
    **ldconfig**  

    配置profile系统文件    
    **vim /etc/profile**    
    在末尾添加一行  
    **export PATH=$PATH:/home/HwHiAiUser/ascend_ddk/arm/bin**    
    使配置文件生效    
    **source /etc/profile**    
    使opencv能找到ffmpeg   
    **cp /home/HwHiAiUser/ascend_ddk/arm/lib/pkgconfig/\* /usr/share/pkgconfig**    
    退出root用户   
    **exit**

3.  安装opencv   
    下载opencv  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv.git**  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv_contrib.git**  
    **cd opencv**  
    **mkdir build**  
    **cd build**  

    编译并安装opencv  
    ```
    cmake -D BUILD_SHARED_LIBS=ON  -D BUILD_opencv_python3=YES -D BUILD_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D  CMAKE_INSTALL_PREFIX=/home/HwHiAiUser/ascend_ddk/arm -D WITH_LIBV4L=ON -D OPENCV_EXTRA_MODULES=../../opencv_contrib/modules -D PYTHON3_LIBRARIES=/usr/lib/python3.6/config-3.6m-aarch64-linux-gnu/libpython3.6m.so  -D PYTHON3_NUMPY_INCLUDE_DIRS=/home/HwHiAiUser/.local/lib/python3.6/site-packages/numpy/core/include/ -D OPENCV_SKIP_PYTHON_LOADER=ON ..
    ``` 
 
    **make -j8**  
    **make install**  

4.   使python3-opencv生效  
   
     **sudo cp  /home/HwHiAiUser/ascend_ddk/arm/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so /usr/lib/python3/dist-packages** 


5.  将开发板上安装的ffmpeg和opencv库导入开发环境中，以提供编译使用。 (如开发环境与运行环境都在Atlas200DK上，请忽略此步)   
    
    $\color{red}{以下操作在开发环境执行}$     
    使用普通用户执行   
    **mkdir $HOME/ascend_ddk**  
    **scp -r HwHiAiUser@192.168.1.2:/home/HwHiAiUser/ascend_ddk/arm $HOME/ascend_ddk**  
    **sudo cd /usr/lib/aarch64-linux-gnu**  
    **sudo scp -r HwHiAiUser@192.168.1.2:/lib/aarch64-linux-gnu/\* ./**  
    **sudo scp -r HwHiAiUser@192.168.1.2:/usr/lib/aarch64-linux-gnu/\* ./**