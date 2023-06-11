中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio运行图片样例?sort_id=3164874)。**

##  dehaze_picture 样例
功能：使用deploy_vel模型对输入图片进行去雾。   
样例输入：jpg图像。   
样例输出：去雾图像。   

### 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件       | 要求                                                         | 备注                                                         |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 硬件要求   | Atlas200 DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))/Atlas200I DK A2 | 当前已在Atlas200 DK、Atlas300、Atlas200I DK A2测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配 |
| CANN版本   | Atlas200 DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))：6.3.RC1 Atlas200I DK A2：6.2.RC1 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#安装)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#版本说明)切换samples仓到对应CANN版本 |
| 第三方依赖 | opencv, ffmpeg+acllite                                       | 请参考[第三方依赖安装指导(C++样例)](../../../environment)完成对应安装 |

### 样例准备

1. 获取源码包。

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

2. 获取此应用中所需要的模型
    | **模型名称** | **模型说明**          | **模型下载路径**                                             |
    | ------------ | --------------------- | ------------------------------------------------------------ |
    | deploy_vel          | 基于tensorflow的去雾处理。 | 请参考https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/dehaze/ATC_deploy_vel_tf_AE 原始模型章节，下载**原始模型**。 |
    为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。    
    
    模型下载。
    
    ```
    cd ${HOME}/samples/python/contrib/dehaze_picture/model    
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/SingleImageDehaze/output_graph.pb   
    ```
    
    模型转换。
    
    - 如果使用的是Atlas200 DK或者Atlas300，请使用如下命令：
    
      ```
      atc --model=output_graph.pb --framework=3 --input_shape="t_image_input_to_DHGAN_generator:1,512,512,3" --output=deploy_vel --soc_version=Ascend310 --input_fp16_nodes="t_image_input_to_DHGAN_generator" --output_type=FP32
      ```
    
    - 如果使用的是Atlas200I DK A2，请使用如下命令：
    
      ```
      atc --model=output_graph.pb --framework=3 --input_shape="t_image_input_to_DHGAN_generator:1,512,512,3" --output=deploy_vel --soc_version=Ascend310B1 --input_fp16_nodes="t_image_input_to_DHGAN_generator" --output_type=FP32
      ```
    
3. 获取样例需要的测试图片
    ```
    cd $HOME/samples/python/contrib/dehaze_picture/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/SingleImageDehaze/test_image/10992_04_0.8209.png 
    ```

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行步骤2即可。**

1. 执行以下命令,将开发环境的**dehaze_picture**目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
   ```
   # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK或200IDKA2在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
   scp -r $HOME/samples/python/contrib/dehaze_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
   ssh HwHiAiUser@xxx.xxx.xxx.xxx   
   ```

2. 运行可执行文件。

    - 如果执行过**步骤1**，请运行以下命令。

      ```
      cd ${HOME}/dehaze_picture/src
      python3.6 main.py ../data/
      ```

    - 如果没有，请执行以下命令。

      ```
      cd $HOME/samples/python/contrib/dehaze_picture/src
      python3.6 main.py ../data/
      ```

### 查看结果

运行完成后，会在运行环境的命令行中打印出推理结果。

### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。