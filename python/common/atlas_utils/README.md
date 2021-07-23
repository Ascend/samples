# python atlasutil 使用说明

## 使用约束

1.本库仅供当前社区开源样例使用，不覆盖ascend平台应用开发的所有场景，不作为用户应用开发的标准库；

2.本库仅在Atlas200DK和Atlas300（x86）服务器上做了验证。

## C码库编译

本库包含Atlas200dk的板载摄像头访问接口，该接口是在C码（lib/src/目录）基础上做的python封装。在Atlas200dk设备上使用本库时，如果对这部分代码有修改，需要重新编译C码。编译依赖libmedia_mini.so, 部署方法参见[环境准备和依赖安装](../../../cplusplus/environment)Atlas200DK基础环境配置部分

### 编译步骤

1.进入lib/src目录；

2.执行编译安装命令：

```
make 
```

编译生成的libatalsutil.so在../atlas200dk/目录下。

Atlas300上使用本库时不涉及该C码部分，不需要编译

## 部署方法

执行应用前需要将本库部署到运行环境。

python atlasutil库依赖pyav, numpy和PIL。在运行环境中需要安装这些第三方库

### 安装pyav

1. 安装ffmpeg。因为apt-get安装的ffmpeg版本很低，所以需要采用源码编译的方式在运行环境中部署ffmpeg。ffmpeg的编译部署参考[环境准备和依赖安装](../../../cplusplus/environment)安装ffmpeg章节

2. 安装其他依赖：

```
apt-get install python3-pip
pip3.6 install --upgrade pip
pip3.6 install Cython
apt-get install pkg-config libxcb-shm0-dev libxcb-xfixes0-dev
cp /home/HwHiAiUser/ascend_ddk/<arch>/lib/pkgconfig/* /usr/share/pkgconfig/
```

其中arch参数在Atlas200dk上使用arm, 即：

`cp /home/HwHiAiUser/ascend_ddk/arm/lib/pkgconfig/* /usr/share/pkgconfig/`

在Atlas300上，根据服务器CPU是arm还是x86_64，分别取arm或者x86

3. 源码安装pyav

```
git clone https://gitee.com/mirrors/PyAV.git
cd PyAV
python3.6 setup.py build --ffmpeg-dir=/home/HwHiAiUser/ascend_ddk/<arch>
python3.6 setup.py install
```

arch参数的选择同上

```
安装过程中常见报错：

错误1：apt-get报错Job for nginx.service failed because the control process exited with error code.
解决方法：将/etc/nginx/sites-enabled/default中
   listen       80 default_server;
   listen       [::]:80 default_server;
   改为：
   listen       80;
   #listen       [::]:80 default_server;

错误2：编译PyAv报错
   Could not find libavdevice with pkg-config.
   Could not find libavfilter with pkg-config.
解决方法：
   步骤1.确认cp /home/HwHiAiUser/ascend_ddk/<arch>/lib/pkgconfig/* /usr/share/pkgconfig/ 执行成功
   步骤2.设置环境变量:
   export PKG_CONFIG_PATH=/usr/share/pkgconfig/
```

4. 测试pyav安装是否成功 

```
cd ..
python3.6
import av
```

 注意：不要再PyAv目录下测试，否则报错

 ModuleNotFoundError: No module named 'av._core'

###  安装numpy和PIL

```
pip3.6 install numpy
pip3.6 install Pillow
```

   ### 安装python atlasutil库

python atlasutil库以源码方式提供，安装时将atlas_utils目录拷贝到运行环境，并将该路径加入PYTHONPATH环境变量即可。例如将整个samples仓拷贝到运行环境$HOME目录下，在~/.bashrc文件中添加：

```
export PYTHONPATH=$HOME/samples/python/common/:$PYTHONPATH
```

并保存，然后执行

```
source ~/.bashrc
```

或者单独将atlas_utils目录拷贝到运行环境$HOME/ascend_ddk/目录下，在~/.bashrc文件中添加：

```
export PYTHONPATH=$HOME/ascend_ddk/:$PYTHONPATH
```

并保存，然后执行

```
source ~/.bashrc
```

在应用代码中调用atlasutil库的接口时导入，例如：

```
import atlas_utils.presenteragent.presenter_channel as presenter_channel

chan = presenter_channel.open_channel(config_file)
```

