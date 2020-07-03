## 实现图片resize功能

### 软件准备

1、获取源码包

  **cd $HOME/AscendProjects**

  **wget** **https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/resize.zip**

  **unzip resize.zip**

>![](E:/v32_40g/C32share/samples/dvpp-samples/crop/public_sys-resources/icon-note.gif) **说明：**   
>
>- 如果使用wget下载失败，可使用如下命令下载代码。  
>  **curl -OL https://c7xcode.obs.myhuaweicloud.com/DVPP-sample/resize.zip** 
>- 如果curl也下载失败，可复制下载链接到浏览器，手动上传至服务器。

### 编译执行

1、以HwHiAiUser登陆开发环境

2、切换到“resize”目录，创建目录用于存放编译文件

**cd resize**

**mkdir -p build/intermediates/host**

第一次需要 到build目录执行cmake生成编译文件

**cd build/intermediates/host**

**cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

3、到out目录执行脚本生成把输入图片resize后生成yuv文档在resize/out目录

**cd ../../../out**

**bash runresize.sh**

### 查看结果

1、out目录有一个output.yuv,他是根据resize的宽度高度生成的yuv420文件。