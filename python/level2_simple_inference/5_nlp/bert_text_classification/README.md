**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导**

## bert文本分类样例

功能：使用bert模型对文本进行分类。

样例输入：待推理的分类的文本。

样例输出：文本的类别。 

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | numpy, python-acllite | 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

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
        # 2. 将zip包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip    

2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  bert_text_classification | 文本分类推理模型 | 请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/nlp/bert_text_classification/ATC_bert_classification_tf_AE](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/nlp/bert_text_classification/ATC_bert_classification_tf_AE)目录中README.md下载原始模型章节下载模型和权重文件 |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    cd ${HOME}/samples/python/level2_simple_inference/5_nlp/bert_text_classification/model     
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/bert_text_classification/bert_text_classification.pb        
    atc --model=bert_text_classification.pb --framework=3 --input_format="ND" --output=bert_text_classification --input_shape="input_1:1,300;input_2:1,300" --out_nodes=dense_1/Softmax:0 --soc_version=Ascend310 --op_select_implmode="high_precision"
    ```

3. 获取样例需要的测试图片。
    ```
    cd ${HOME}/samples/python/level2_simple_inference/5_nlp/bert_text_classification/data
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/bert_text_classification/sample.txt
    ``` 

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **bert_text_classification** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r ${HOME}/samples/python/level2_simple_inference/5_nlp/bert_text_classification  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/bert_text_classification/src
    ```

2. <a name="step_2"></a>运行工程。
    ```
    python3.6 bert_text_classification.py
    ```
​       
### 查看结果

运行完成后，会在运行环境的命令行中打印出推理结果，用户可以自己编辑data/sample.txt下的文本进行测试，目前模型支持"体育"、"健康"、"军事"、"教育"、"汽车"，五个类别的文本分类。
结果参考如下：
![输入图片说明](https://images.gitee.com/uploads/images/2021/1110/101306_d08122bf_8083019.png "txt.png")