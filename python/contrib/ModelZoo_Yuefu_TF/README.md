**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配20.1及以上版本，支持产品为Atlas800-9000、Atlas800-9010。**


### 乐府作诗样例

功能：一个高质量的古诗词生成系统。

样例输入：诗歌标题和诗歌格式。

样例输出：  对应格式的诗歌。

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](https://support.huaweicloud.com/instg-cli-cann/atlascli_03_0001.html)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。

1. 获取源码包。

   可以使用以下方式下载，进行源码的准备。

    - 开发环境，使用运行用户在命令行中执行以下命令下载。

       **cd $HOME**

       **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/yuefuzuoshi/ModelZoo_Yuefu_TF.zip**
        开发环境中，执行以下命令，解压zip包。

        **cd $HOME**

        **unzip ModelZoo_Yuefu_TF.zip**
2. 获取此应用中所需要的网络模型。
进入上一步中源码包解压后的models目录
    **cd $HOME/ModelZoo_Yuefu_TF/models**
    下载模型文件
    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/yuefuzuoshi/poetry.data-00000-of-00001**
    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/yuefuzuoshi/poetry.meta**
    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/yuefuzuoshi/poetry.index**
        
 ### 样例部署
**以下以开发环境与运行环境合一部署为例。**   
- 源码包中预置了跑单卡的脚本，在运行之前，我们需要做相应脚本的修改。
（1）修改device_table_1p.json中的device_id和device_ip。
使用命令
**cat /etc/hccn.conf**
 查看服务器上device的IP列表，如果没有配置，可以参考[修改NPU卡IP地址](https://support.huaweicloud.com/instg-cli-cann/atlascli_03_0084.html)进行配置。
 **将device_table_1p.json文件中的第16行的device_id和第17行的device_ip改成以上查到的device设备和IP。**
 （2）修改运行脚本main_1p.sh，配置参数。
    **DEVICE_ID改成上面修改的对应的device_id。**
    **max_decode_len：最大生成长度，默认设置为80**
    **title：输入想要生成诗歌的主题**
    **type：输入想要生成诗歌的题材，包括五言律诗, 五言绝句, 七言绝句, 七言律诗**


### 样例运行



执行以下命令

**bash main_1p.sh** 
​       

### 查看结果

运行完成后，界面打印以下内容：
```
七言绝句(格式)中秋(诗歌标题)
青山万古水千波，
此夕清光奈老何。
无限世间闲草木，
不知强有几人多。
```
