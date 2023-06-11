# Blast

## 功能描述

本样例使用TIK方式进行了blast算子的实现。

BLAST (Basic Local Alignment Search Tool) 它是一个用来比对生物序列的一级结构（如不同蛋白质的氨基酸序列或不同基因的DNA序列）的算法。 已知一个包含若干序列的数据库，BLAST可以让研究者在其中寻找与其感兴趣的序列相同或类似的序列。例如如果某种非人动物的一个以前未知的基因被发现，研究者一般会在人类基因组中做一个BLAST搜索来确认人类是否包含类似的基因（通过序列的相似性）。

本项目中的BLAST算法属于简化后的算法（原本是sending-extending），简化了extending部分，采用动态规划的思想进行比对。

该算子具有seq1（待对比序列）、seq2（数据库中的标准序列）和scores（记录相似性的分数矩阵）三个关键输入，其功能为将seq1与seq2进行对比，通过计算结果填充scores矩阵。

计算过程：

1）seq1与seq2的范围为1-4，代表四种碱基A、G、C、T，每次对比从seq2中取出一段长度与seq1相等的序列进行意义对比；

2）如果值相同，则得分+2；如果碱基对为12,21,34,43中一种，则得分-5,；其余情况均得分-7。

3）用动态规划的思想将分数与scores矩阵当前位置左上角（i-1,j-1）相加并与(i-1,j)与（i,j-1）相比，将最大值填入矩阵中；

4）一段序列比对完成后将矩阵最后一个值也是最大值填入out结果中，循环1）到3）

三个输入的关系分别为：

- 张量seq1的shape为（n，）
- 张量seq2的shape为（m，n）
- scores的shape应满足大于（n+1）*（n+1）

Example：

```python
seq1 = np.random.randint(1, 5,(16,))
seq2 = np.random.randint(1, 5,(32, 16))
scores = np.zeros(shape=(400,))
out = np.zeros(shape=(32))
result= blast_tik(seq1,seq2,scores,out)
```

## 支持的芯片类型

ascend 310

## 环境要求

部署此sample前，需要准备好以下环境：

- 确认已按照环境准备和依赖安装准备好环境
- 已完成对应产品的开发环境和运行环境安装，
- 环境中已经安装对应版本Mindstudio。

如不满足请按照备注进行相应处理。

|    条件    | 要求                                                         | 备注                                                         |
| :--------: | ------------------------------------------------------------ | ------------------------------------------------------------ |
|  CANN版本  | CANN5.1.RC2.2(商用版)                                                 | 有关CANN安装，请参考[昇腾官方文档](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha001/overview/index.html) |
|  硬件要求  | Atlas200DK/Atlas300([ai1s](https://gitee.com/link?target=https%3A%2F%2Fsupport.huaweicloud.com%2Fproductdesc-ecs%2Fecs_01_0047.html%23ecs_01_0047__section78423209366)) | 当前已在Atlas200DK测试通过，产品说明请参考[硬件平台](https://gitee.com/link?target=https%3A%2F%2Fascend.huawei.com%2Fzh%2F%23%2Fhardware%2Fproduct) ，其他产品可能需要另做适配 |
| MindStudio | MindStudio 5.0.RC2(Release)                                  | 有关MindStudio安装，参考[昇腾官方文档](https://www.hiascend.com/document/detail/zh/mindstudio/50RC3/instg/instg_000002.html) |
|  其他依赖  | python3.7.5                                                  | CANN对python版本有[相关要求](https://www.hiascend.com/document/detail/zh/CANNCommunityEdition/600alpha001/softwareinstall/instg/atlasdeploy_03_0022.html#ZH-CN_TOPIC_0000001430292765) |

## 算子开发流程

1.明确算子的功能。

blast算子的功能时通过动态规划的思想计算DNA序列与标准序列的相似性，将相似性以分值形式输出，详细功能示例可参考功能描述。


2.明确输入和输出。

- Blast算子有三个输出，一个输出

  输入：seq1（待对比的序列），seq2（用于对比的标准序列），scores（存放分数）。

  输出：out（与每个标准序列对比的最大分）。

- 本样例中算子的输入seq1支持的数据类型为int16，float16；输入seq2支持的数据类型为int16，float16；输入scores支持的数据类型为int16，float16；输出out支持的数据类型为：int16，float16。

- 算子输入shape需满足功能描述中的要求，输出out的维度应与seq2的shape[0]相等。

- 算子输入支持的format为：ND。


3.确定算子开发方式及使用的计算流程设计。

由于Blast算子涉及对tensor的不同维度上的不同元素同时计算，TBE DSL接口无法满足此算子的计算要求，所以考虑使用TIK方式进行此算子的实现。

该算子实现核心的计算流程如下：

​	1）首先对所有的数据进行格式的校验，调用TIK原生接口check_shape_rule()，check_dtype_rule()等。如果不符合原型定义所规定，就终止程序并返回对应错误信息。

​	2）使用TIK提供的tik.data_move()接口将seq1，seq2，scores，out分别搬入到UB Buffer中，每次取出seq1与一行seq2进行计算，根据blast算法的原理得到计算结果，将其存入socres中，其中程序控制使用了TIK提供的if_scope,else_scope,for_range,any接口来代替python的相应功能。

​	3）采用动态规划的思想处理分值，采用当一轮对比结束时，将scores的最后一个元素值，也就是这一轮对比中的最大值存入out中。

​	 4）当seq2完全遍历完后将out从UB中搬出到GM内存中，得到最终的结算结果。

针对这个核心计算流程，我们要设计相应的schedule策略。schedule策略在设计的时候主要考虑两个  基本问题：

​	  1）首先是多核，当前昇腾AI处理器有多个AI Core可以做并行计算，可以极大的提升计算的性能。对于Blast这个算子，核心是遍历seq2，为了实现多核并行计算，我们将seq2进行分拆，让不同AI Core分别进行seq2的遍历。假设seq2的shape为（100，m），这时候我们将100个seq2的切片均分给每个核进行处理，最后根据是否有剩余切片再进行分配，这样就可以实现多个核的并行处理。

​	  2）其次由于我们的UB空间有大小限制，所以不能给每个seq2的切片都分配一个scores矩阵的空间。因此，只给每个核一个scores矩阵的空间用来计算，每计算完一条序列就将scores重新初始化来节省空间，采用tik.vec_dup()接口来快速填充scores矩阵完成初始化。

![image_20221205151133](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221205151133.png "image_20221205151133.png")

4.算子信息库定义：将算子的相关信息注册到算子信息库中，包括算子的输入（seq1，seq2，scores）输出（out）dtype、format以及输入shape信息。


5.算子ST测试：系统测试（System Test），Mindstudio提供了设置测试数据格式后自动生成测试文件的功能，完成json文件后可在真实的硬件环境中，验证算子的正确性。


## 算子工程准备

1.准备工程

获取源码包。可以使用以下两种方式下载，请选择其中一种进行源码准备

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

2.在HwHiAiUser 用户命令行中执行以下命令，打开MindStudio。

**cd ~/MindStudio/bin**

**./MindStudio.sh**

3.导入MindStudio工程

- 首次登录MindStudio：单击**Open or Import**，然后找到工程路径并选择。

- 非首次登录MindStudio：在顶部菜单栏中选择**File > Open**...，然后找到工程路径

  **${HOME}/samples/best_practices/contrib/blast/blast_tik**  并选择打开，在当前窗口或者新窗口。

4.配置编译环境

1）第一次打开工程文件需要检查环境是否选择正确，单击**File>Project Structure>Project Setting>Project** ，选择配置的python版本。

![image-20221129111746827](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image-20221129111746827.png "image-20221129111746827.png")


2）检查CANN版本已经开发套件是否配置正确：单击**Ascend>CANN Manager**，查看版本是否匹配。如果是在昇腾设备下运行直接在本地找到toolkit位置导入即可，如果是远程编译，需要先[配置ssh连接](https://www.hiascend.com/document/detail/zh/mindstudio/50RC3/instg/instg_000023.html)后再进行操作。

![image-20221129111621854](https://sharedata.obs.cn-north-4.myhuaweicloud.com/blast-code/tik_readme/image_20221206123737.png)

## 算子Debug和ST测试

### Debug操作以及结果

1.完成算子工程准备后，找到**tbe/impl/blast_tik.py**文件，右键单击“Tik Operator Debug”，在命令行中输入c后，按下回车；

![image-20221206145246666](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206151333.png)

2.计算结束后，结果如下图所示：

![image-20221206151035526](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206151341.png)

### ST测试

如果debug时间较长，除debug方式外，本样例还提供了ST测试的方式进行算子的验证。

1.完成算子工程准备后，找到testcases/st/blast_tik/ascend310目录下的json文件

![image-20221206145732543](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206151347.png)

2.右键单击运行即可。运行结果如下:

设置精度偏差为0.01，允许有5%不满足精度误差，测试结果对比100%通过，测试用例pass

![image-20221202184011728](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image-20221202184011728.png "image-20221202184011728.png")

算子计算过程耗时：

![image-20221202193237694](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206153605.png)


## 算子编译和部署

### 算子编译

1. 在MindStudio工程界面，选中算子工程。

2. 单击顶部菜单栏的“ Build > Edit Build Configuration...”。

3. 进入编译配置页面。单击![img](https://www.hiascend.com/doc_center/source/zh/mindstudio/50RC2/msug/figure/zh-cn_image_0000001302244780.png)添加新增配置，默认添加编译类型“Release(default)”，请参考进行编译配置

   ![image-20221206143439742](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206151256.png)

4. 单击![img](https://www.hiascend.com/doc_center/source/zh/mindstudio/50RC2/msug/figure/zh-cn_image_0000001355083813.png)或“Build>Build Project”进行工程编译**。**

5. 在界面最下方的窗口查看编译结果，并在算子工程的cmake-build目录下生成自定义算子安装包**custom_opp_Linux_\*Arch\*.run**。![image-20221206143549120](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206151312.png)

### 算子部署（本地）

1. 在MindStudio工程界面菜单栏依次选择“Ascend > Operator Deployment”，弹出算子部署界面。

2. 在弹出的界面中选择“Operator Deploy Locally”，在“Operator Package”中选择指定的算子库OPP包目录并单击“Operator deploy”。

   在下方Output页签出现如下信息，代表自定义算子部署成功。

   ![image-20221206144012689](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206151320.png)

   ![image-20221206144035011](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/image_20221206151326.png)

   ​	自定义算子包安装成功后，会将自定义算子部署在***Ascend-cann-toolkit安装目录\*/ascend-toolkit/latest****/opp**目录下或指定的算子库OPP目录下的对应文件夹中。

   **注：除MindStudio提供的部署方式外，也可以直接获取已经生成的[run包](https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/custom_opp_Linux_aarch64.run)，并直接在运行环境上执行以下命令完成部署。**

   ```
   wget https://sharedata.obs.myhuaweicloud.com/blast-code/tik_readme/custom_opp_Linux_aarch64.run
   ./custom_opp_Linux_aarch64.run
   ```

   

