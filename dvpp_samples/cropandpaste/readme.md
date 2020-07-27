## 实现图片cropandpaste功能

### 软件准备

   **cd $HOME/AscendProjects**

​	**wget** **https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/cropandpaste.zip**

​     **unzip cropandpaste.zip**

 >![](public_sys-resources/icon-note.gif) **说明：**   
   >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/cropandpaste.zip** 
   >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。

### 编译代码
 
1、以HwHiAiUser登陆开发环境 

2、切换到“cropandpaste”目录，创建目录用于存放编译文件

**cd cropandpaste**

第一次需要 到build目录执行cmake生成编译文件

创建的目录为“build/intermediates/host”。

**mkdir -p build/intermediates/host**

第一次需要 到build目录执行cmake生成编译文件

**cd build/intermediates/host**

**cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

### 代码执行

1、到out目录执行脚本生成把输入图片抠图贴图后生成yuv文档在cropandpaste/out目录

**cd ../../../out**

**bash runcropandpaste.sh**

注意：执行时可能会报：No module named  ‘PIL’。是因为缺少pillow模块。  

**pip install pillow**

安装pillow时如果报错：ERROR setup.py install for pillow ...error...等一长串可以执行一下代码安装依赖

**sudo yum install python-devel**

**sudo yum install zlib-devel**

**sudo yum install libjpeg-turbo-devel**  

再执行安装命令  
**pip install pillow**

### 查看结果

1、out目录有一个output.yuv他是根据resize的宽度高度生成的yuv420文件。