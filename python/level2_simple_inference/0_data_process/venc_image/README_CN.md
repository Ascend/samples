# 媒体数据处理（视频编码）

## 功能描述

该样例将一张YUV420SP NV12格式的图片连续编码n次，生成一个H265格式的视频码流（n可配，通过运行应用时设置入参来配置，由代码中的venc_cnt参数来控制，默认为16次）。

## 目录结构

如下为模型文件转换后的示例目录结构，model文件夹是转换后生成的。

```
venc_image
├──src
│ └── acl_venc.py //视频编码实现文件
└── data
  └── dvpp_venc_128x128_nv12.yuv //用户待处理的图片文件
```

## <span id = "env">环境变量配置 </span>

在安装完CANN软件包之后，请务必自行配置pyACL相关的环境变量，否则，将无法正常“import acl”。

1. 若环境中安装了cann-toolkit软件包：
```
# 以root用户安装toolkit包
. /usr/local/Ascend/ascend-toolkit/set_env.sh
# 以非root用户安装toolkit包
. ${HOME}/Ascend/ascend-toolkit/set_env.sh
```
2. 若环境中安装了cann-nnrt软件包：
```
# 以root用户安装nnrt包
. /usr/local/Ascend/nnrt/set_env.sh
# 以非root用户安装nnrt包
. ${HOME}/Ascend/nnrt/set_env.sh
```
3. 若环境中安装了cann-nnae软件包：
```
# 以root用户安装nnae包
. /usr/local/Ascend/nnae/set_env.sh
# 以非root用户安装nnae包
. ${HOME}/Ascend/nnae/set_env.sh
```

-   **Atlas 500 智能小站：**
    1.  配置PYTHON环境变量，将“/home/data/miniD/driver/lib64”加入到PYTHON中。命令示例如下：
        ```
        export PYTHONPATH=/home/data/miniD/driver/lib64:$PYTHONPATH
        ```

    2. 配置LD\_LIBRARY\_PATH，将“/home/data/miniD/driver/lib64”加入到LD_LIBRARY_PATH中。命令示例如下：
        ```
        export LD_LIBRARY_PATH=/home/data/miniD/driver/lib64:$LD_LIBRARY_PATH
        ```

-   **Atlas 200 AI加速模块（ RC场景）：**
    1.  安装minirc版本的runtime包，以安装在默认路径“/usr/local/Ascend”为例：配置LD\_LIBRARY\_PATH，将“/usr/local/Ascend/runtime/lib64”加入到LD\_LIBRARY\_PATH中。命令示例如下：
        ```
        export LD_LIBRARY_PATH=/usr/local/Ascend/runtime/lib64:$LD_LIBRARY_PATH
        ```

    2.  获取aarch64架构的cann-nnrt软件包，在服务器上解压，在软件包所在目录下的解压命令如下：
        ```
        ./Ascend-cann-nnrt_xxx_linux-aarch64.run --noexec --extract="解压目标路径"
        ```
        在解压目标路径下找到“run_package/Ascend-pyACL-xxx-linux.aarch64.run”，在软件包所在目录下的解压命令如下：
        ```
        ./Ascend-pyACL-xxx-linux.aarch64.run --noexec --extract="解压目标路径"
        ```
        在解压路径下找到“python/site-packages/acl/acl.so”，将acl.so上传至200RC上，例如，上传至“ /usr/local/Ascend/pyACL/python/site-packages/acl”目录下，配置PYTHON环境变量，将“ /usr/local/Ascend/pyACL/python/site-packages/acl”加入到PYTHON中。命令示例如下：
        ```
        export PYTHONPATH=/usr/local/Ascend/pyACL/python/site-packages/acl:$PYTHONPATH
        ```

## 运行应用

1.  运行应用
    1.  登录运行环境。
    2.  准备输入数据。
        从[Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dvpp_venc_128x128_nv12.yuv)上获取输入图片文件dvpp_venc_128x128_nv12.yuv，并以HwHiAiUser（运行用户）上传至开发环境的“venc_image/data”目录下。  
    3.  参照[环境变量配置](#env)完成运行环境的配置。
    4.  在venc_image路径下执行如下命令：
        ```
        python3 ./src/acl_venc.py
        ```
