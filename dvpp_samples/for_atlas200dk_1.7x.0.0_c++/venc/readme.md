## 实现视频编码功能     

### 软件准备    
1、获取源码包

  **cd $HOME/AscendProjects**

  **wget** **https://c7xcode.obs.cn-north-4.myhuaweicloud.com/DVPP-sample/venc.zip**

  **unzip venc.zip**

 >![](E:/v32_40g/C32share/samples/dvpp-samples/crop/public_sys-resources/icon-note.gif) **说明：**   
 >
 >- 如果使用wget下载失败，可使用如下命令下载代码。  
 >  **curl -OL https://c7xcode.obs.cn-north-4.myhuaweicloud.com/DVPP-sample/venc.zip** 
 >- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。
### 编译代码

1、以ascend登陆开发环境

2、切换到“venc”目录，创建目录用于存放编译文件，例如，本文中，创建的文件夹为“build”

 **mkdir build** 

3、切换到build目录

 **cd build** 

4、设置下环境变量，在命令行内执行，下面的路径请根据实际目录层级修改

 **export DDK_PATH=/home/ascend/Ascend/ascend-toolkit/20.0.RC1/acllib_centos7.6.aarch64 && export NPU_HOST_LIB=/home/ascend/Ascend/ascend-toolkit/20.0.RC1/acllib_centos7.6.aarch64/acllib/lib64/stub** 

5、在build目录下，执行如下的命令生成编译文件。“../src”表示 CMakeLists.txt文件所在的目录，请根据实际目录层级修改 

 **cmake  ../src -DCMAKE_SKIP_RPATH=TRUE -Dtarget= -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++-5** 

6、在编译文件夹目录执行make命令，生成的可执行文件main 在“venc/out”目录下

 **make** 

### 运行应用
1、在Atlas 200DK开发板上连接摄像头，上电开机

2、以HwHiAiUser（运行用户）将开发环境的“venc”目录下的out目录上传到运行环境的一个目录下，例如 **“/home/HwHiAiUser/venc/out”** 。   
运行环境： **mkdir venc**    
开发环境： **scp -r /home/ascend/AscendProjects/venc/out HwHiAiUser@192.168.1.2:venc/out** 

3、以HwHiAiUser（运行用户）登录运行环境。

4、切换到可执行文件main所在的目录，例如“/home/HwHiAiUser/venc/out”

给该目录下的main文件加执行权限。

```
chmod +x main
```

运行可执行文件。

```
./main
```

在venc/out文件夹下，生成xxxxxxxx.h265 以日期和时刻为文件名的编码视频文件，案例中摄像头默认采集了100帧图像，可根据需要修改。

