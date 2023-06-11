# Blast算子运行--aclopExecute

功能：Blast单算子运行。
样例输入：构造的随机输入数据。
样例输出：Blast的输出结果。

## 前置条件

请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。

| 条件     | 要求                                                         | 备注                                                         |
| -------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| CANN版本 | CANN5.1.RC2.2(商用版)                                        | 有关CANN安装，请参考[昇腾官方文档](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha001/overview/index.html) |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://gitee.com/link?target=https%3A%2F%2Fsupport.huaweicloud.com%2Fproductdesc-ecs%2Fecs_01_0047.html%23ecs_01_0047__section78423209366)) | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://gitee.com/link?target=https%3A%2F%2Fascend.huawei.com%2Fzh%2F%23%2Fhardware%2Fproduct) ，其他产品可能需要另做适配 |

### 注：已参考**blast/blast_tik/README_CN.md**完成自定义算子的编译部署。


## 模型和数据准备

1.获取源码包。可以使用以下两种方式下载，请选择其中一种进行源码准备

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
```

2.模型转换。将算子模型转换为适配昇腾310处理器的离线模型（*.om文件）。

```
# 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
cd ${HOME}/samples/best_practices/contrib/blast/blast_acl/run/out/test_data/config/
wget https://sharedata.obs.myhuaweicloud.com/blast-code/acl_op.json
cd ${HOME}/samples/best_practices/contrib/blast/blast_acl/run/out/
atc --singleop=test_data/config/acl_op.json --soc_version=Ascend310 --output=op_models
```

其中：

- singleop：算子描述的json文件。
- soc_version：昇腾AI处理器的型号，此处以Ascend310为例，请根据实际情况替换。
- --output=op_models：代表生成的模型文件存储在当前目录下的op_models文件夹下。

模型转换成功后，会生成如下文件：

在当前目录的op_models目录下生成单算子的模型文件**0_Blast_6_2_16_6_2_32_16_6_2_400_6_2_32.om**，命名规范为：序号+opType + 输入的描述(dateType_format_shape)+输出的描述。

dataType以及format对应枚举值请从CANN软件所在目录下的“include/graph/types.h”文件中查看，枚举值从0开始依次递增。

**说明：**模型转换时，会优先去查找自定义算子库去匹配模型文件中的算子。

 3.准备测试数据。请从以下链接获取该样例的输入图片，放在“/data“目录下。

```
cd ${HOME}/samples/best_practices/contrib/blast/blast_acl/run/out/test_data/data/
wget https://sharedata.obs.myhuaweicloud.com/blast-code/Test_Blast_001_case_001_ND_int16_input_0.bin
wget https://sharedata.obs.myhuaweicloud.com/blast-code/Test_Blast_001_case_001_ND_int16_input_1.bin
wget https://sharedata.obs.myhuaweicloud.com/blast-code/Test_Blast_001_case_001_ND_int16_input_2.bin
```



## 编译运行

1.编译代码并运行

```
cd ${HOME}/samples/best_practices/contrib/blast/blast_acl/scripts/
bash sample_build.sh 
bash sample_run.sh
```

执行完成后，会屏显打印输出数据，同时会生成结果二进制文件result_files/Test_Blast_001_case_001_ND_int16_output_0.bin。结果如下：

![image-20221203155941338](https://sharedata.obs.myhuaweicloud.com/blast-code/image-20221203155941338.png)

**注：在样例数据设置中，待测DNA数据与每一条基准DNA进行相似性的计算，得出分数越高说明两条DNA序列更相似。图中由于样例数据为随机生成，所以分数较低属于自然现象。**