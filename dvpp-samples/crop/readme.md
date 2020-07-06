## 实现图片crop功能

### 软件准备
1、获取源码包

  **cd $HOME/AscendProjects**

  **wget** **https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/crop.zip**

  **unzip crop.zip**

 >![](public_sys-resources/icon-note.gif) **说明：**   
    >- 如果使用wget下载失败，可使用如下命令下载代码。  
    **curl -OL https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/crop.zip** 
   >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。

### 编译代码

1、以HwHiAiUser登陆开发环境

2、切换到“crop”目录，创建目录用于存放编译文件

**cd crop**

创建的目录为“build/intermediates/host”。

**mkdir -p build/intermediates/host**

第一次需要 到build目录执行cmake生成编译文件

**cd build/intermediates/host**

**cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

**make**

### 代码执行

1、到out目录执行可执行文件，把输入图片resize后生成yuv文档在crop/out目录(图片样例自行下载)

**cd ../../../out**

**./crop ../data/input.jpg 1024 1061 output.yuv 350 280 200 200**



​    >![](public_sys-resources/icon-note.gif) **说明：**   

> - input.jpg、output.yuv文件名自行更改，./crop ../data/input.jpg   w   h  output.yuv  x  y  w  h。w、h分别为照片宽度和高度。x、y为位置。
>   

### 查看结果

1、out目录有一个output.yuv他是根据resize的宽度高度生成的yuv420文件。