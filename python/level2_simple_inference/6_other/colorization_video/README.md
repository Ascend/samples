**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.0.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行视频样例wiki](https://gitee.com/ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E8%A7%86%E9%A2%91%E6%A0%B7%E4%BE%8B?sort_id=3170138)。**


## 视频黑白图像上色样例

**注：案例详细介绍请参见[视频黑白图像上色_wiki](https://gitee.com/ascend/samples/wikis/%E8%A7%86%E9%A2%91%E9%BB%91%E7%99%BD%E5%9B%BE%E5%83%8F%E4%B8%8A%E8%89%B2?sort_id=3170478)。**

功能：使用黑白图像上色模型对输入的黑白视频进行推理。。

样例输入：黑白mp4视频。

样例输出：presenter界面展现推理结果。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](https://gitee.com/ascend/samples/tree/dev/python/environment)准备好环境。

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

2. 获取此应用中所需要的原始网络模型。

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/colorization_video。
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  colorization| 黑白视频上色推理模型。是基于Caffe的colorization模型。  |  请参考[https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/colorization/ATC_colorization_caffe_AE](https://gitee.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/colorization/ATC_colorization_caffe_AE)目录中README.md下载原始模型章节下载模型和权重文件。 |

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - modelzoo中提供了转换好的om模型，但此模型不匹配当前样例，所以需要下载原始模型和权重文件后重新进行模型转换。

3. 将原始模型转换为Davinci模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](https://gitee.com/ascend/samples/tree/dev/python/environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export install_path=$HOME/Ascend/ascend-toolkit/latest**    
        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下atc命令进行模型转换。

        **cd $HOME/models/colorization_video**  

        **atc --input_shape=data_l:1,1,224,224 --weight=colorization.caffemodel --input_format=NCHW --output="./colorization" --soc_version=Ascend310 --framework=0 --model=colorization.prototxt**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./colorization.om $HOME/samples/python/level2_simple_inference/6_other/colorization_video/model/**

4. 获取样例需要的测试文件。

    执行以下命令，进入样例的data文件夹中，下载对应的测试文件，完成后返回样例文件夹。

    **cd $HOME/samples/python/level2_simple_inference/6_other/colorization_video/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/colorization_video/black-white_video.mp4**


### 样例部署

1. 修改present相关配置文件。

    将样例目录下**scripts/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为开发环境中可以ping通运行环境的ip地址，使用以下两种情况举例说明。

     - 使用产品为200DK开发者板。   
        1. 开发环境中使用ifconfig查看可用ip。   
        2. 在开发环境中将**scripts/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为该ip地址。   
        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 1.开发环境和运行环境分离部署，一般使用配置的虚拟网卡ip，例如192.168.1.223。
        > - 2.开发环境和运行环境合一部署，一般使用200dk固定ip，例如192.168.1.2。

    - 使用产品为300加速卡（ai1s云端推理环境）。   
        1. ECS弹性云服务器控制台中查看ai1s云端环境可用内网ip，例如192.168.0.198。   
        2. 在开发环境中将**scripts/param.conf**中的 presenter_server_ip、presenter_view_ip 修改为该ip地址。   
        ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
        > - 也可以在ai1s云端环境中使用ifconfig查看内网ip。
        > - 登录ai1s云端环境时的ip地址为此环境的公网ip，ai1s云端环境中ifconfig查看到的ip为此环境的内网ip。
 


### 样例运行

![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
> - 以下出现的**xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

1. 执行以下命令,将开发环境的 **colorization_video** 目录上传到运行环境中，例如 **/home/HwHiAiUser**。   

    **开发环境与运行环境合一部署，请跳过此步骤！**   

    **scp -r $HOME/samples/python/level2_simple_inference/6_other/colorization_video HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

2. 登录运行环境并启动presenterserver。

    - 使用产品为200DK开发者板。   
        1. 执行以下命令登录运行环境。   
           **开发环境与运行环境合一部署，请跳过此步骤！**   
           **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    
        2. 运行环境中启动presenterserver。   
进入工程所在目录（如$HOME/samples/common），执行以下命令

            **bash run_presenter_server.sh /home/HwHiAiUser/samples/python/level2_simple_inference/6_other/colorization_video/scripts/param.conf**


    - 使用产品为300加速卡（ai1s云端推理环境）。   
        1. 执行以下命令登录运行环境。   
           **开发环境与运行环境合一部署，请跳过此步骤！**   
           **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    
      2. 运行环境中启动presenterserver。   
进入工程所在目录（如$HOME/samples/common），执行以下命令

            **bash run_presenter_server.sh /home/HwHiAiUser/samples/python/level2_simple_inference/6_other/colorization_video/scripts/param.conf**

3. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。   
      **export LD_LIBRARY_PATH=**   
      **source ~/.bashrc**     
      **cd $HOME/samples/python/level2_simple_inference/6_other/colorization_video/src**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。   
      **cd $HOME/colorization_video/src**

    切换目录后，执行以下命令运行样例。

    **python3.6 colorize.py ../data/black-white_video.mp4**

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
