**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## 人体动作识别样例
功能：该系统使用连续的图像作为输入，实时检测视频中的行人，基于AI算法实时检测人体关键节点，并获取关节的角度。将检测数据转换为识别的动作打印到屏幕
由机器人完成对人体动作的模仿。      
样例输入：原始图片jpg文件。    
样例输出：推理结果。     

### 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E)切换samples仓到对应CANN版本 |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配|
| 第三方依赖 | presentagent, opencv, ffmpeg+acllite | 请参考[第三方依赖安装指导（C++样例）](../../environment)完成对应安装 |

### 安装libeigen3-dev
由于只有该样例使用了此依赖，所以运行该样例前，还需要安装libeigen3-dev。
- 开发环境架构与运行环境架构相同。    
  **开发环境和运行环境均**执行以下命令安装libeigen3-dev。
  ```
  sudo apt-get install libeigen3-dev
  ```
- 开发环境为x86，运行环境为arm。
  1. **运行环境**联网并执行以下命令进行安装
      ```
      sudo apt-get install libeigen3-dev
      ```
  2. **开发环境**执行以下命令拷贝对应so
      ```
      # 将arm下的相关的so拷贝到X86的aarch64-linux-gnu目录，不会对本地X86环境本身使用产生任何问题。
      cd /usr/lib/aarch64-linux-gnu
      # 拷贝相关so，其中X.X.X.X为运行环境ip地址。
      sudo scp -r HwHiAiUser@X.X.X.X:/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/aarch64-linux-gnu/* ./
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/lib/*.so.* ./
      # 拷贝相关头文件
      sudo scp -r HwHiAiUser@X.X.X.X:/usr/include/eigen3 /usr/include
      ```

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

2. 模型转换。      
   | **模型名称** | **模型说明**             | **模型下载路径**                                             |
   | ------------ | ------------------------ | ------------------------------------------------------------ |
   | openpose     | 人体骨骼关键点检测模型。 | https://github.com/Ascend/modelzoo/blob/9bf8402aca694dd602be536b5d7ff782c5e8c4e4/contrib/TensorFlow/Research/cv/%20gesturedetection/ATC_OpenPose_caffe_AE |
   | STGCN        | 人体动作识别模型。       | https://github.com/Ascend/modelzoo/tree/9bf8402aca694dd602be536b5d7ff782c5e8c4e4/contrib/TensorFlow/Research/cv/%20gesturedetection/ATC_STGCN_tf_AE |

   ```
   # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
   
   cd $HOME/samples/cplusplus/contrib/gesturedetection/model    
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_iter_440000.caffemodel 
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/pose_deploy.prototxt
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/insert_op.cfg
   wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/gesturedetection/stgcn_fps30_sta_ho_ki4.pb
   atc --input_shape="input_features:1,2,30,14" --input_format=NCHW --output="./stgcn_fps30_sta_ho_ki4" --soc_version=Ascend310 --framework=3 --model=./stgcn_fps30_sta_ho_ki4.pb
   atc --input_shape="data:1,3,128,128" --weight="./pose_iter_440000.caffemodel" --input_format=NCHW --output="./pose_deploy" --soc_version=Ascend310 --insert_op_conf=./insert_op.cfg --framework=0 --model=./pose_deploy.prototxt 

### 样例部署
执行以下命令，执行编译脚本，开始样例编译。      
   ```
   cd  $HOME/samples/cplusplus/contrib/gesturedetection/scripts    
   bash sample_build.sh
   ```

### 样例运行
1. 执行以下命令,将开发环境的 **gesturedetection** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。    
   ```
   # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
   scp -r $HOME/samples/cplusplus/contrib/gesturedetection/ HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser    
   ssh HwHiAiUser@xxx.xxx.xxx.xxx     
   cd $HOME/samples/cplusplus/contrib/gesturedetection/scripts
   ```

2. <a name="step_2"></a>执行运行脚本，开始样例运行。         
   ```
   bash sample_run.sh
   ```

### 查看结果
运行完成后，会在运行环境的命令行中打印出推理结果。

### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。