## 目录

  - [样例介绍](#样例介绍)
  - [样例运行](#样例运行)
  - [其他资源](#其他资源)
  - [更新说明](#更新说明)
  - [已知issue](#已知issue)
    
## 样例介绍

功能：HelloWord for AscendCL, 一个简单样例快速理解AscendCL基础概念。  

## 样例运行
  - 获取源码包
    
    可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

       ```    
       # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```
       **注：如果需要切换到其它tag版本，以v0.5.0为例，可执行以下命令。**
       ```
       git checkout v0.5.0
       ```   
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
       **注：如果需要下载其它版本代码，请先请根据前置条件说明进行samples仓分支切换。**   
       ``` 
        # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
        # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip
        ```

  - 样例编译

    设置环境变量，配置程序编译依赖的头文件与库文件路径。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。
    ```
    export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
    export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub 
    ```

    执行以下命令，执行编译脚本，开始样例编译。
    ```
    cd $HOME/samples/inference/ACLHelloWorld/scripts
    bash sample_build.sh
    ```

  - 样例运行
    
    执行运行脚本，开始样例运行。
    ```
    bash sample_run.sh
    ```
  - 样例结果展示
    
    执行成功后，在屏幕上的提示信息示例如下：

    ```
    [INFO] The sample starts to run
    [INFO] Acl Init Success
    [INFO] Acl Set Device Success,Current DeviceID:0
    [INFO] Acl Create Context Success
    [INFO] Acl Create Stream Success
    [INFO] Acl Destroy Stream Success
    [INFO] Acl Destroy Context success
    [INFO] Acl Reset Device Success
    [INFO] Acl Finalize Success
    [INFO] The program runs successfully

    ```

## 其他资源

以下资源提供了对该sample更深入的理解：

**Documentation**
- [AscendCL Samples介绍](../README_CN.md)
- [昇腾文档](https://www.hiascend.com/document?tag=community-developer)

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/02/17 | 新增ACLHelloWorld |
  

## 已知issue

  暂无
