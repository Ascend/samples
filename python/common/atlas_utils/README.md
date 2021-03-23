# python atlasutil 使用说明

## 使用约束

1.本库仅供当前社区开源样例使用，不覆盖ascend平台应用开发的所有场景，不作为用户应用开发的标准库；

2.本库仅在Atlas200DK和Atlas300（x86）服务器上做了验证。

## C码库编译

本库包含Atlas200dk的摄像头的访问接口和视频解码接口，这部分接口是在C码实现的基础上做的python封装。C代码在lib/src/目录下。如果对这部分代码有修改，需要重新编译C码库。C码库的编译依赖：

1. 依赖acl库，使用前需要安装ascend开发环境

2. 视频解码使用ffmpeg+dvpp，依赖ffmpeg库。ffmpeg的编译和部署参见[环境准备和依赖安装](../../../../../cplusplus/environment)

### 编译步骤

1.进入lib/src目录；

2.执行编译安装命令。

Atlas200DK：

```
make 
```

编译生成的libatalsutil.so在../atlas200dk/目录下

其他昇腾AI设备:

```
make mode=ASIC
```

编译生成的libatalsutil.so在../asic/x86（服务器为x86_64）或者../asic/arm（服务器为arm）目录下



