**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**

## ascendcamera摄像头样例

**注：案例详细介绍请参见[ascendcamera_wiki](https://gitee.com/ascend/samples/wikis/%E8%A7%86%E9%A2%91googlenet%E5%88%86%E7%B1%BB?sort_id=3164845)。**

功能：使用摄像头拍摄照片或视频。

样例输入：摄像头。

样例输出：presenter界面展现推理结果，或者数据保存至本地。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](https://gitee.com/ascend/samples/tree/dev/cplusplus/environment)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。   
        开发环境，非root用户命令行中执行以下命令下载源码仓。   
       **cd $HOME**   
       **git clone https://gitee.com/ascend/samples.git**

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。   
        2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。   
        3. 开发环境中，执行以下命令，解压zip包。   
            **cd $HOME**   
            **unzip ascend-samples-master.zip**



### 样例部署

1. 修改present相关配置文件。

    将样例目录下**script/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为开发环境中可以ping通运行环境的ip地址，使用以下两种情况举例说明。

     - 使用产品为200DK开发者板。   
        1. 开发环境中使用ifconfig查看可用ip。   
        2. 在开发环境中将**script/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为该ip地址。   
        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 1.开发环境和运行环境分离部署，一般使用配置的虚拟网卡ip，例如192.168.1.223。
        > - 2.开发环境和运行环境合一部署，一般使用200dk固定ip，例如192.168.1.2。

    - 使用产品为300加速卡（ai1s云端推理环境）。   
        1. ECS弹性云服务器控制台中查看ai1s云端环境可用内网ip，例如192.168.0.198。   
        2. 在开发环境中将**script/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为该ip地址。   
        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 也可以在ai1s云端环境中使用ifconfig查看内网ip。
        > - 登录ai1s云端环境时的ip地址为此环境的公网ip，ai1s云端环境中ifconfig查看到的ip为此环境的内网ip。
 
2. 开发环境命令行中设置编译依赖的环境变量。

   可以在命令行中执行 **uname -a**，查看开发环境和运行环境的cpu架构。如果回显为x86_64，则为x86架构。如果回显为arm64，则为Arm架构。基于开发环境与运行环境CPU架构是否相同，请仔细看下面的步骤：

   - 当开发环境与运行环境CPU架构相同时，执行以下命令导入环境变量。

     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux**

     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**

     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 如果是20.0版本，此处 **DDK_PATH** 环境变量中的 **x86_64-linux** 应修改为 **x86_64-linux_gcc7.3.0**。
        > - 可以在命令行中执行 **uname -a**，查看开发环境和运行环境的cpu架构。如果回显为x86_64，则为x86架构。如果回显为arm64，则为Arm架构。

   - 当开发环境与运行环境CPU架构不同时，执行以下命令导入环境变量。例如开发环境为X86架构，运行环境为Arm架构，由于开发环境上同时部署了X86和Arm架构的开发套件，后续编译应用时需要调用Arm架构开发套件的ACLlib库，所以此处需要导入环境变量为Arm架构的ACLlib库路径。 
  
     **export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux**  
 
     **export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub**   
     ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 如果是20.0版本，此处 **DDK_PATH** 环境变量中的 **arm64-liunx** 应修改为 **arm64-linux_gcc7.3.0**。

3. 切换到ascendcamera目录，创建目录用于存放编译文件，例如，本文中，创建的目录为 **build/intermediates/host**。

    **cd $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcamera**

    **mkdir -p build/intermediates/host**

4. 切换到 **build/intermediates/host** 目录，执行cmake生成编译文件。

    - 当开发环境与运行环境操作系统架构相同时，执行如下命令编译。   
      **cd build/intermediates/host**  
      **make clean**   
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE**

    - 当开发环境与运行环境操作系统架构不同时，需要使用交叉编译器编译。例如开发环境为X86架构，运行环境为Arm架构，执行以下命令进行交叉编译。   
      **cd build/intermediates/host**   
      **make clean**   
      **cmake \.\./\.\./\.\./src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE**

5. 执行make命令，生成的可执行文件main在 **ascendcamera/out** 目录下。

    **make**

### 样例运行（图片保存至本地）

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **ascendcamera** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcamera HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcamera/out**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/ascendcamera/out**

    切换目录后，执行以下命令运行样例。

    **./main -i -c 1 -o ./output/filename.jpg --overwrite**

参数说明：

 -   -i：代表获取jpg格式的图片。

   -   -c：表示摄像头所在的channel，此参数有“0”和“1”两个选项，“0“对应“Camera1“，“1“对应“Camera2“，如果不填写，默认为“0”。

   -   -o：表示文件存储位置，此处output为本地已存在的文件夹名称，filename.jpg为保存的图片名称，可用户自定义。

   -   --overwrite：覆盖已存在的同名文件。

### 查看结果

运行完成后，会在运行环境的命令行中打印出运行结果,并在将运行结果保存在$HOME/ascendcamera/out/output。

### 样例运行（视频保存至本地）

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **ascendcamera** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcamera HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcamera/out**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/ascendcamera/out**

    运行之前需要在out文件夹下新建output文件夹

    **cd $HOME/ascendcamera/out**

    **mkdir output**

    切换目录后，执行以下命令运行样例。

    **./main**

### 查看结果

运行完成后，会在运行环境的命令行中打印出运行结果,并在将运行结果保存在$HOME/ascendcamera/out/output。

### 样例运行（presenterserver）

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
> - 以下出现的**xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

1. 执行以下命令,将开发环境的 **ascendcamera** 目录上传到运行环境中，例如 **/home/HwHiAiUser**。   

    **开发环境与运行环境合一部署，请跳过此步骤！**   

    **scp -r $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcamera HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. 启动presenterserver并登录运行环境。

    - 使用产品为200DK开发者板。   
        1. 开发环境中执行以下命令启动presentserver。   
            **cd $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcamera**   
            **bash script/run_presenter_server.sh**   
        2. 执行以下命令登录运行环境。   
            **开发环境与运行环境合一部署，请跳过此步骤！**   
            **ssh HwHiAiUser@xxx.xxx.xxx.xxx** 

    - 使用产品为300加速卡（ai1s云端推理环境）。   
        1. 执行以下命令登录运行环境。   
           **开发环境与运行环境合一部署，请跳过此步骤！**   
           **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    
        2. 运行环境中执行以下命令启动presenterserver。   
            **cd $HOME/ascendcamera**   
            **bash script/run_presenter_server.sh**   

3. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。   
      **export LD_LIBRARY_PATH=**   
      **source ~/.bashrc**     
      **cd $HOME/samples/c++/level1_single_api/5_200dk_peripheral/ascendcameraout**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。   
      **cd $HOME/ascendcamera/out**

    切换目录后，执行以下命令运行样例。并将ip和xxxx修改为对应的ip和端口号。

    **./main -v -c 1 -t 60 --fps 20 -w 704 -h 576 -s ip:xxxx/presentername**

    参数说明：
    

    -   -v：代表获取摄像头的视频，用来在Presenter Server端展示。

    -   -c：表示摄像头所在的channel，此参数有0”和1两个选项，0对应Camera0，1对应Camera1，如果不填写，默认为0。   
 
    -   -t：表示获取60s的视频文件，如果不指定此参数，则获取视频文件直至程序退出。

    -   -fps：表示存储视频的帧率，取值范围为1\~20，如果不设置此参数，则默认存储的视频帧率为10fps。

    -   -w：表示存储视频的宽。

    -   -h：表示存储视频的高。

    -   -s：后面的ip值为运行环境IP地址，**xxxx**为Ascendcamera应用对应的Presenter Server服务器的端口号。

    -   _presentername_：为在Presenter Server端展示的“View Name“，用户自定义，需要保持唯一，只能为大小写字母、数字、“\_”的组合，位数3\~20。

### 查看结果

1. 打开presentserver网页界面。

   - 使用产品为200DK开发者板。

      打开启动Presenter Server服务时提示的URL即可。
      
   - 使用产品为300加速卡（ai1s云端推理环境）。

      **以300加速卡（ai1s）内网ip为192.168.0.194，公网ip为124.70.8.192举例说明。**

      启动Presenter Server服务时提示为Please visit http://192.168.0.194:7009 for display server。

      只需要将URL中的内网ip：192.168.0.194替换为公网ip：124.70.8.192，则URL为 http://124.70.8.192:7009。

      然后在windows下的浏览器中打开URL即可。

2. 等待Presenter Agent传输数据给服务端，单击“Refresh“刷新，当有数据时相应的Channel 的Status变成绿色。

3. 单击右侧对应的View Name链接，查看结果。



