中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导**

## 手写数字分类样例

功能：使用lenet模型对输入图片进行分类推理。   
样例输入：待推理的jpg图片。    
样例输出：存放图片中数字的txt文本。   

训练过程参考 [lennet mindspore训练](https://gitee.com/mindspore/models/tree/master/official/cv/lenet)；
 在Ascend910环境， 使用scripts/convert.py 脚本将训练好的checkpoint_lenet-1_1875.ckpt 转换为mnist.air模型文件

### 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E)切换samples仓到对应CANN版本 |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配|
| 第三方依赖 | opencv, numpy | 请参考[第三方依赖安装指导（python样例）](../../../environment)选择需要的依赖完成安装 |

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

2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    | mnist | 图片分类推理模型。是基于mindspore的lenet模型。 | https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/model    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air
    atc --framework=1 --model=mnist.air  --output=mnist --soc_version=Ascend310
    ```

3. 获取样例需要的测试图片。
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/data    
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test1.png
    ```

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **lenet_mindspore_picture** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/lenet_mindspore_picture/src    
    ```

2. <a name="step_2"></a>运行样例。
   ```
   cd ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/src
   python3.6 classify.py ./data/
   ```

### 查看结果

运行时，会在命令行打印推理结果，显示如下
```
init resource stage:
Init resource success
Init model resource start...
[Model] create model output dataset:
malloc output 0, size 40
Create model output dataset success
Init model resource success
(32, 32)
post process
images:test1.png
======== top5 inference results: =============
label:1  confidence: 0.993403, class: 1
label:9  confidence: 0.001830, class: 9
label:8  confidence: 0.001219, class: 8
label:4  confidence: 0.001122, class: 4
label:7  confidence: 0.000977, class: 7
acl resource release all resource
Model release source success
acl resource release stream
acl resource release context
Reset acl device  0
Release acl resource success
run success
```
并在outputs文件夹下生成txt文件，存放置信度最高的数字。

### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。
