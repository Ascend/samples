中文|[英文](README_EN.md)

# 安装ffmpeg+opencv<a name="ZH-CN_TOPIC_0228768065"></a>

安装ffmpeg和opencv的原因是适配多样性的数据预处理和后处理，昇腾社区的部分样例也是基于ffmpeg和opencv做的处理。如果用户仅需要构建自己的代码，可以不安装ffmpeg+opencv，直接使用自带的dvpp能力。值得一提的是，dvpp在Ascend310芯片中是有硬件方面的加速能力的，处理速度也会更快。

1.  用户权限配置
    普通用户安装开发套件，需要有sudo权限，所以首先需要给普通用户配置权限。

    切换为root用户  
    **su root**

    给sudoer文件配置写权限，并打开该文件  
    **chmod u+w /etc/sudoers**   
    **vi /etc/sudoers** 

    在该文件 **“ # User privilege specification”** 下面增加如下内容：  
     **HwHiAiUser    ALL=(ALL:ALL) ALL** 

    ![输入图片说明](https://images.gitee.com/uploads/images/2020/1128/121157_37d3b82d_7401379.png "屏幕截图.png")

2.  开发板设置联网（root用户密码：Mind@123）  
     
    **sudo vi /etc/netplan/01-netcfg.yaml**   
    填写以下配置  
    **注：需要注意这里的缩进格式，netplan配置时和python类似，对缩进有强限制** 

    ```
    network:
      version: 2
    #  renderer: NetworkManager
      renderer: networkd
      ethernets:
        eth0:
          dhcp4: yes 
   
        usb0:
          dhcp4: no 
          addresses: [192.168.1.2/24] 
          gateway4: 192.168.0.1
    ``` 
    ![](figures/network.png "")  

    将开发板网口接上可正常联网的网线，填写后执行以下命令使配置生效   
    **sudo netplan apply**      
  
3.  开发板换源

     **以下给出两种源，选择其中一种使用，如更新源失败，请自行更换可用Ubuntu 18.04 arm源** 
    - ubuntu18.04-arm华为源  
 
        执行以下换源操作  
        **sudo wget -O /etc/apt/sources.list https://repo.huaweicloud.com/repository/conf/Ubuntu-Ports-bionic.list**   

        更新源  
        **sudo apt-get update** 

    - ubuntu18.04-arm官方源 

        修改源文件  
        **sudo vi /etc/apt/sources.list**   
         
        将源文件内容替换为以下ubuntu-arm官方源。
        ```
        deb http://ports.ubuntu.com/ bionic main restricted universe multiverse
        deb-src http://ports.ubuntu.com/ bionic main restricted universe multiverse
        deb http://ports.ubuntu.com/ bionic-updates main restricted universe multiverse
        deb-src http://ports.ubuntu.com/ bionic-updates main restricted universe multiverse
        deb http://ports.ubuntu.com/ bionic-security main restricted universe multiverse
        deb-src http://ports.ubuntu.com/ bionic-security main restricted universe multiverse
        deb http://ports.ubuntu.com/ bionic-backports main restricted universe multiverse
        deb-src http://ports.ubuntu.com/ bionic-backports main restricted universe multiverse
        deb http://ports.ubuntu.com/ubuntu-ports/ bionic main universe restricted
        deb-src http://ports.ubuntu.com/ubuntu-ports/ bionic main universe restricted  
        ```


        更新源。  
        **sudo apt-get update** 

4.  安装相关依赖  
    **sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg-dev libtiff5-dev git cmake libswscale-dev python3-setuptools python3-dev python3-pip pkg-config -y** 
 
    **python3 -m pip install --upgrade pip --user**  
    **python3 -m pip install Cython --user**  
    **python3 -m pip install numpy --user**

5.  安装ffmpeg  
  
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

6.  安装opencv   
    下载opencv  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv.git**  
    **git clone -b 4.3.0 https://gitee.com/mirrors/opencv_contrib.git**  
    **cd opencv**  
    **mkdir build**  
    **cd build**  

    安装opencv  
    ```
    cmake -D BUILD_SHARED_LIBS=ON  -D BUILD_opencv_python3=YES -D BUILD_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D  CMAKE_INSTALL_PREFIX=/home/HwHiAiUser/ascend_ddk/arm -D WITH_LIBV4L=ON -D OPENCV_EXTRA_MODULES=../../opencv_contrib/modules -D PYTHON3_LIBRARIES=/usr/lib/python3.6/config-3.6m-aarch64-linux-gnu/libpython3.6m.so  -D PYTHON3_NUMPY_INCLUDE_DIRS=/usr/local/lib/python3.6/dist-packages/numpy/core/include -D OPENCV_SKIP_PYTHON_LOADER=ON ..
    ``` 
 
    **make -j8**  
    **make install**  

7.   使python3-opencv生效  
   
     **sudo cp  /home/HwHiAiUser/ascend_ddk/arm/lib/python3.6/dist-packages/cv2.cpython-36m-aarch64-linux-gnu.so /usr/lib/python3/dist-packages** 


8.  修改环境变量
    程序编译时会链接LD_LIBRARY_PATH环境变量地址中的库文件，所以要将ffmpeg和opencv安装的库文件地址加到该环境变量中。  
    **vi ~/.bashrc**  
    在最后添加  
    **export LD_LIBRARY_PATH=/home/HwHiAiUser/Ascend/acllib/lib64:/home/HwHiAiUser/ascend_ddk/arm/lib**
    ![](figures/bashrc.png "")   
    执行以下命令使环境变量生效。  
    **source ~/.bashrc**

9.  将开发板上安装的ffmpeg和opencv库导入开发环境中，以提供编译使用。  
     **以下操作在host侧执行，不在开发板上。**    
    使用普通用户执行   
    **mkdir $HOME/ascend_ddk**  
    **scp -r HwHiAiUser@192.168.1.2:/home/HwHiAiUser/ascend_ddk/arm $HOME/ascend_ddk**  
    切换至root用户  
    **su root**  
    **cd /usr/lib/aarch64-linux-gnu**  
    **scp -r HwHiAiUser@192.168.1.2:/lib/aarch64-linux-gnu/\* ./**  
    **scp -r HwHiAiUser@192.168.1.2:/usr/lib/aarch64-linux-gnu/\* ./**