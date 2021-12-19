中文|[English](README_EN.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 麦克风样例

功能：录制麦克风音频数据

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。

| 适配项     | 适配条件                                                     | 备注                                                         |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 适配版本   | >=5.0.4                                                    | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件   | Atlas200DK | 摄像头样例仅在Atlas200DK测试，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | opencv                                     | 请参考[第三方依赖安装指导（C++样例）](../../../environment)完成对应安装 |

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。 
      ```  
      # 开发环境，非root用户命令行中执行以下命令下载源码仓。   
      cd $HOME   
      git clone https://github.com/Ascend/samples.git
      ```
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
      ```
      # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。   
      # 2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 【$HOME/ascend-samples-master.zip】。   
      # 3. 开发环境中，执行以下命令，解压zip包。   
      cd $HOME  
      unzip ascend-samples-master.zip
      ```

### 样例部署
1. 切换到mic目录，创建目录用于存放编译文件，例如，本文中，创建的目录为 **build/intermediates/host**。
   ```
   cd $HOME/samples/cplusplus/level1_single_api/5_200dk_peripheral/mic
   mkdir -p build/intermediates/host
   ```

2. 切换到 **build/intermediates/host** 目录，执行cmake生成编译文件。
   ```
   cd build/intermediates/host
   make clean
   cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
   ```

3. 执行make命令，生成的可执行文件main在 **mic/out** 目录下。
   ```
   make
   ```
### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤3](#step_3)即可。**   

1. 执行以下命令,将开发环境的 **gpio** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
   ```
   # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2。
   scp -r $HOME/samples/cplusplus/level1_single_api/5_200dk_peripheral/mic HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx    
   ```
2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令切换目录。
      ```
      cd $HOME/samples/cplusplus/level1_single_api/5_200dk_peripheral/mic/out
      ```
    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
      ```
      cd $HOME/mic/out
      ```
    切换目录后，执行以下命令运行样例。
    ```
    ./main
    ```

### 查看结果

运行完成后，会在运行环境的命令行中打印出运行结果。
