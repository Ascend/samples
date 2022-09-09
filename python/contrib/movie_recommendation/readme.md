**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导**


系统致力于深入研究深度学习和强化学习算法，提升营销精准度，营销效率和多业务、多场景的适应能力，构建对特定用户的个性化影视推荐系统。整个系统可以分为构建多模态特征融合数据库、构建基于深度神经网络的精准推荐技术、构建基于营销反馈的自动化增强学习框架等多个部分。

功能：使用推荐模型对用户进行产品推荐。

样例输入：用户的历史记录

样例输出：为其推荐的产品。

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas300T  | 当前已在Atlas300T训练通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product)) |
| 适配硬件 | Atlas300I([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas300I测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | pandas，numpy, tf-slim, networkx | 请参考[第三方依赖安装指导（python样例）](../../environment)完成对应安装 |

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。   
    - 命令行方式下载（下载时间较长，但步骤简单）。
       ```    
       # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```   
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
       ``` 
        # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
        # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip

2. 下载原始数据
    ```
    针对推荐系统，需要对数据特征进行提取，首先下载链接中的代码 链接：https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/xidian/feature.tar
    在安装好npu环境和外部环境以后
    按照如下步骤运行程序

### 样例运行
1. 提取特征
    ``` 
    在Atlas300T910上，实现特征提取，解压feature.tar压缩包到当前目录下
    
    cd ${HOME}/movie_recommendation/huawei-movielens/
    
    运行 python feature.py 

2. 训练模型和强化学习
    
    （1）训练模型：在Atlas300T910上，实现模型训练，解压network.tar压缩包到当前目录下 
    ``` 
    cd ${HOME}/movie_recommendation/ARLMR-new/

    python AC_DDPG.py  

    ``` 
    （2）强化学习：当代码跳出输入时，输入feedback则进行强化学习，输入break则代码停止运行

    <img src="https://gitee.com/gs_zhang/picture/raw/master/4.png" width="800px">

    ```
    通过不断更新 ${HOME}/movie_recommendation/ARLMR-new/feedback/hit_users_1m.txt 中用户点击的产品，不断实现强化学习生成新的推荐结果
    ``` 

2. 推理测试结果
    ```
    在Atlas300I上，实现模型推理，解压evaluation.tar压缩包到当前目录下 
    
    首先采用一下命令实现模型转换
    atc --model=./MM.pb --framework=3 --output=./model --soc_version=Ascend310

    再通过一下命令实现模型推理
    cd ${HOME}/movie_recommendation/ARLMR-npu/
    python base_model.py  

### 查看结果

（1）特征提取结果保存在  

    ${HOME}/movie_recommendation/huawei-movielens/output
<img src="https://gitee.com/gs_zhang/picture/raw/master/1.png" width="500px">

（2）采用训练模型实现推荐的结果保存在  

    ${HOME}/movie_recommendation/ARLMR-new/result/final_res.csv
    训练模型得到的结果如下
<img src="https://gitee.com/gs_zhang/picture/raw/master/2.png" width="400px">

    训练模型然后强化学习得到的结果如下
<img src="https://gitee.com/gs_zhang/picture/raw/master/3.png" width="300px">

（3）推理准确率在结束后产生

<img src="https://gitee.com/gs_zhang/picture/raw/master/5.png" width="500px">
