## 实现视频解码功能

### 软件准备

1、获取源码包

  **cd $HOME/AscendProjects**

  **wget** **https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/vdec.zip**

  **unzip vdec.zip**

>![](E:/v32_40g/C32share/samples/dvpp-samples/crop/public_sys-resources/icon-note.gif) **说明：**   
>
>- 如果使用wget下载失败，可使用如下命令下载代码。  
>  **curl -OL https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/vdec.zip** 
>- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。



### 编译代码

1、在命令行输入：   
**export DDK_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/20.0.0.RC1/acllib_centos7.6.x86_64 && export NPU_HOST_LIB=/home/HwHiAiUser/Ascend/ascend-toolkit/20.0.0.RC1/acllib_centos7.6.x86_64/acllib/lib64/stub**

2、切换到“vdec”目录，创建目录用于存放编译文件，例如，本文中，创建的文件夹为“build”

**cd vdec**

**mkdir build**

3、切换到build目录，执行cmake生成编译文件。 “../src”表示 CMakeLists.txt文件所在的目录，请根据实际目录层级修改

**cd build**

**cmake ../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

4、在编译文件夹目录执行make命令，生成的可执行文件main 在“vdec/out”目录下

**make**

### 运行应用


1、给该目录下的main文件加执行权限。

```
chmod +x main
```

运行可执行文件。

```
./main
```

在vdec/out文件夹下，生成vdec_h265_1frame_rabbit_1280x720.h265 解码的图片。

