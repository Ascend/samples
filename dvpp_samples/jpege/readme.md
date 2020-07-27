## 实现图片编码（JPEGE）功能

### 软件准备

1、获取源码包

  **cd $HOME/AscendProjects**

  **wget** **https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/jpege.zip**

  **unzip jpege.zip**

>![](E:/v32_40g/C32share/samples/dvpp-samples/crop/public_sys-resources/icon-note.gif) **说明：**   
>
>- 如果使用wget下载失败，可使用如下命令下载代码。  
>  **curl -OL https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/jpege.zip** 
>- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。

### 编译代码

1、以HwHiAiUser登陆开发环境

2、切换到“jpege”目录，创建目录用于存放编译文件

**cd jpege**

例如，本文中，创建的目录为“build/intermediates/host”。

**mkdir -p build/intermediates/host**

3、切换到“build/intermediates/host”目录，执行cmake生成编译文件。
“../../../src”表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

**cd build/intermediates/host**

**cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

4、执行make命令，生成的可执行文件jpege在“jpege/out”目录下。

**make**

5、到out目录执行可执行文件，生成jpg文件在jpege/out目录

**cd ../../../out**

**./jpege**

### 查看结果

6、out目录下有根据data目录中的文件生成的jpg文件。
